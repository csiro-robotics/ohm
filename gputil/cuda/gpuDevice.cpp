// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gputil/gpuDevice.h"

#include "gputil/cuda/gpuDeviceDetail.h"

#include "gputil/gpuApiException.h"
#include "gputil/gpuThrow.h"

#include <cuda.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cstring>
#include <sstream>

using namespace gputil;

namespace
{
  void initDeviceInfo(DeviceInfo &gputil_info, const cudaDeviceProp &cuda_info)
  {
    gputil_info.name = cuda_info.name;
    gputil_info.platform = "CUDA";  // cuda_info.name;
    gputil_info.version.major = cuda_info.major;
    gputil_info.version.minor = cuda_info.minor;
    gputil_info.version.patch = 0;
    gputil_info.type = kDeviceGpu;
  }

  bool selectDevice(DeviceDetail *detail, int device_id = -1)
  {
    cudaError_t err = cudaSuccess;

    if (device_id < 0)
    {
      // Select default device.
      err = cudaGetDevice(&device_id);
    }

    if (err == cudaSuccess)
    {
      cudaDeviceProp cuda_info;
      memset(&cuda_info, 0, sizeof(cuda_info));
      if (cudaGetDeviceProperties(&cuda_info, device_id) == cudaSuccess)
      {
        detail->device = device_id;
        initDeviceInfo(detail->info, cuda_info);
        return true;
      }
    }

    detail->device = -1;
    detail->info = DeviceInfo();

    return false;
  }
}  // namespace

Device::Device(bool default_device)
  : imp_(new DeviceDetail)
{
  imp_->device = -1;
  if (default_device)
  {
    selectDevice(imp_);
  }
}


Device::Device(const DeviceInfo &device_info)
  : imp_(new DeviceDetail)
{
  select(device_info);
}


Device::Device(int argc, const char **argv, const char *default_device, unsigned device_type_flags)
  : imp_(new DeviceDetail)
{
  select(argc, argv, default_device, device_type_flags);
}


Device::Device(const Device &other)
  : imp_(new DeviceDetail)
{
  *imp_ = *other.imp_;
}


Device::Device(Device &&other) noexcept
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Device::~Device()
{
  delete imp_;
}


unsigned Device::enumerateDevices(std::vector<DeviceInfo> &devices)
{
  int device_count = 0;
  cudaGetDeviceCount(&device_count);

  for (int i = 0; i < device_count; ++i)
  {
    cudaDeviceProp cuda_info;
    DeviceInfo gputil_info;
    memset(&cuda_info, 0, sizeof(cuda_info));
    if (cudaGetDeviceProperties(&cuda_info, i) == cudaSuccess)
    {
      initDeviceInfo(gputil_info, cuda_info);
      devices.push_back(gputil_info);
    }
  }

  return static_cast<unsigned>(device_count);
}


const char *Device::name() const
{
  return imp_->name.c_str();
}


const char *Device::description() const
{
  return imp_->name.c_str();
}


const DeviceInfo &Device::info() const
{
  return imp_->info;
}


Queue Device::defaultQueue() const
{
  return Queue(nullptr);
}


Queue Device::createQueue(unsigned /*flags*/) const
{
  cudaStream_t stream = nullptr;
  cudaError_t err;
  err = cudaSetDevice(imp_->device);
  GPUAPICHECK(err, cudaSuccess, Queue());
  err = cudaStreamCreate(&stream);
  GPUAPICHECK(err, cudaSuccess, Queue());
  return Queue(stream);
}


bool Device::select(int argc, const char **argv, const char *default_device, unsigned device_type_flags)
{
  if ((device_type_flags & kGpu) == 0)
  {
    // Only GPU supported for CUDA.
    return false;
  }

  // Resolve constraints:
  // - (Partial) device name match
  // - Compute capability version number.

  std::string name_constraint = default_device ? default_device : "";
  Version version_min;

  for (int i = 0; i < argc; ++i)
  {
    if (strncmp("--device", argv[i], 8) == 0)
    {
      // Read value from "--device="
      if (argv[i][8])
      {
        default_device = argv[i] + 9;
      }
    }
    else if (strncmp("--clver", argv[i], 7) == 0)
    {
      // Read version from "--clver="
      if (argv[i][7])
      {
        char dot;
        std::string ver_str = argv[i] + 8;
        std::istringstream in(ver_str);
        in >> version_min.major;
        in >> dot;
        in >> version_min.minor;
      }
    }
  }

  std::vector<DeviceInfo> devices;
  enumerateDevices(devices);

  std::transform(name_constraint.begin(), name_constraint.end(), name_constraint.begin(), ::tolower);

  const auto name_check = [](const std::string &device_name, const std::string &constraint) -> bool  //
  {
    if (constraint.empty())
    {
      return true;
    }

    std::string device_lower = device_name;
    std::transform(device_lower.begin(), device_lower.end(), device_lower.begin(), ::tolower);
    return device_lower.find(constraint) != std::string::npos;
  };

  const auto ver_check = [](const Version &device_version, const Version &constraint) -> bool  //
  {
    if (device_version.major > constraint.major)
    {
      return true;
    }
    else if (device_version.major == constraint.major)
    {
      if (device_version.minor > constraint.minor)
      {
        return true;
      }
      else if (device_version.minor == constraint.minor)
      {
        if (device_version.patch > constraint.patch)
        {
          return true;
        }
        else if (device_version.patch == constraint.patch)
        {
        }
      }
    }

    return false;
  };

  for (auto &&device : devices)
  {
    if (name_check(device.name, name_constraint) && ver_check(device.version, version_min))
    {
      return select(device);
    }
  }

  return false;
}


bool Device::select(const DeviceInfo &device_info)
{
  int device_count = 0;
  cudaGetDeviceCount(&device_count);

  for (int i = 0; i < device_count; ++i)
  {
    cudaDeviceProp cuda_info;
    DeviceInfo gputil_info;
    memset(&cuda_info, 0, sizeof(cuda_info));
    if (cudaGetDeviceProperties(&cuda_info, i) == cudaSuccess)
    {
      initDeviceInfo(gputil_info, cuda_info);
      if (gputil_info == device_info)
      {
        return selectDevice(imp_, i);
      }
    }
  }

  return false;
}


void Device::setDebugGpu(DebugLevel)
{
  // Ignored for CUDA.
}


Device::DebugLevel Device::debugGpu() const
{
  return kDlOff;
}


bool Device::supportsFeature(const char * /*feature_id*/) const
{
  // TODO(KS): find CUDA feature checking or use compute capabilities.
  return false;
}


void Device::addSearchPath(const char *)
{
  // Ignored for CUDA.
}


const char *Device::searchPaths() const
{
  return "";
}


bool Device::isValid() const
{
  return imp_ && imp_->device >= 0;
}


uint64_t Device::deviceMemory() const
{
  size_t free = 0;
  size_t total = 0;
  cudaError_t err = cudaSuccess;
  err = cudaSetDevice(imp_->device);
  GPUAPICHECK(err, cudaSuccess, 0u);
  err = cudaMemGetInfo(&free, &total);
  GPUAPICHECK(err, cudaSuccess, 0u);
  return total;
}


bool Device::unifiedMemory() const
{
  cudaDeviceProp cuda_info;
  memset(&cuda_info, 0, sizeof(cuda_info));
  cudaError_t err = cudaGetDeviceProperties(&cuda_info, imp_->device);
  GPUAPICHECK(err, cudaSuccess, false);
  // This is a bit of an assumption.
  return cuda_info.integrated != 0;
}


Device &Device::operator=(const Device &other)
{
  *imp_ = *other.imp_;
  return *this;
}


Device &Device::operator=(Device &&other) noexcept
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}
