// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpuDevice.h"

#include "gpuApiException.h"
#include "gpuDeviceDetail.h"

#include <clu/clu.h>
#include <clu/cluConstraint.h>

#include <sstream>
#include <vector>

using namespace gputil;

namespace
{
  void finaliseDetail(DeviceDetail &detail, const DeviceInfo *info)
  {
    if (info)
    {
      detail.info = *info;
    }
    else
    {
      detail.device.getInfo(CL_DEVICE_NAME, &detail.info.name);
      // cl::Platform platform = detail.device.pla
      detail.device.getInfo(CL_DEVICE_NAME, &detail.info.platform);

      std::string ver_string;
      detail.device.getInfo(CL_DEVICE_VERSION, &ver_string);

      // Parse the version string.
      cl_uint ver_major, ver_minor;
      clu::parseVersion(ver_string.c_str(), &ver_major, &ver_minor);
      detail.info.version.major = ver_major;
      detail.info.version.minor = ver_minor;

      cl_platform_id platform_id;
      detail.device.getInfo(CL_DEVICE_PLATFORM, &platform_id);

      cl_device_type device_type;
      detail.device.getInfo(CL_DEVICE_TYPE, &device_type);

      detail.info.type = kDeviceNull;
      if (device_type & CL_DEVICE_TYPE_GPU)
      {
        detail.info.type = kDeviceGpu;
      }
      else if (device_type & CL_DEVICE_TYPE_CPU)
      {
        detail.info.type = kDeviceCpu;
      }
      else if (device_type & CL_DEVICE_TYPE_ACCELERATOR)
      {
        detail.info.type = kDeviceOther;
      }

      size_t required_len = 0;
      clGetPlatformInfo(platform_id, CL_PLATFORM_NAME, 0, nullptr, &required_len);
      if (required_len)
      {
        std::vector<char> info_buffer(required_len + 1);
        info_buffer[0] = '\0';
        clGetPlatformInfo(platform_id, CL_PLATFORM_NAME, required_len + 1, info_buffer.data(), nullptr);
        detail.info.platform = std::string(info_buffer.data());
      }

      detail.device.getInfo(CL_DEVICE_EXTENSIONS, &detail.extensions);

      std::ostringstream str;
      clu::printDeviceInfo(str, detail.device, "");
      detail.description = str.str();
    }
  }


  unsigned enumerateDevices(std::vector<cl::Device> &cl_devices, std::vector<DeviceInfo> &device_infos)
  {
    std::vector<cl::Platform> platforms;
    std::vector<cl::Device> devices;
    std::string ver_string;
    cl::Platform::get(&platforms);
    unsigned added = 0;

    // API minimum version is 1.2.
    const auto platform_version_constraint = clu::platformVersionMin(1, 2);

    for (cl::Platform &platform : platforms)
    {
      DeviceInfo info;
      platform.getInfo(CL_PLATFORM_NAME, &info.platform);

      if (!platform_version_constraint(platform))
      {
        continue;
      }

      devices.clear();
      platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);

      for (cl::Device &device : devices)
      {
        device.getInfo(CL_DEVICE_NAME, &info.name);
        device.getInfo(CL_DEVICE_VERSION, &ver_string);

        // Parse the version string.
        cl_uint ver_major, ver_minor;
        clu::parseVersion(ver_string.c_str(), &ver_major, &ver_minor);
        info.version.major = ver_major;
        info.version.minor = ver_minor;

        cl_device_type device_type;
        device.getInfo(CL_DEVICE_TYPE, &device_type);

        info.type = kDeviceNull;
        if (device_type & CL_DEVICE_TYPE_GPU)
        {
          info.type = kDeviceGpu;
        }
        else if (device_type & CL_DEVICE_TYPE_CPU)
        {
          info.type = kDeviceCpu;
        }
        else if (device_type & CL_DEVICE_TYPE_ACCELERATOR)
        {
          info.type = kDeviceOther;
        }

        cl_devices.push_back(device);
        device_infos.push_back(info);
        ++added;
      }
    }

    return added;
  }
}  // namespace

Device::Device(bool default_device)
  : imp_(new DeviceDetail)
{
  if (default_device)
  {
    if (!clu::getPrimaryContext(imp_->context, imp_->device))
    {
      // Needs initialisation.
      // Empty constraints.
      const cl_device_type device_type = CL_DEVICE_TYPE_GPU | CL_DEVICE_TYPE_ACCELERATOR;
      const std::vector<clu::PlatformConstraint> platform_constraints;
      const std::vector<clu::DeviceConstraint> device_constraints;
      clu::initPrimaryContext(device_type, platform_constraints, device_constraints);
      clu::getPrimaryContext(imp_->context, imp_->device);

      imp_->queue = cl::CommandQueue(imp_->context, imp_->device);
    }

    if (isValid())
    {
      std::vector<cl::Device> devices;
      clu::listDevices(devices, imp_->context);
      imp_->device = devices[0];
      imp_->queue = cl::CommandQueue(imp_->context, imp_->device);
      finaliseDetail(*imp_, nullptr);
    }
  }
}


Device::Device(const DeviceInfo &device_info)
  : imp_(new DeviceDetail)
{
  select(device_info);
}


Device::Device(int argc, const char **argv, const char *default_device)
  : imp_(new DeviceDetail)
{
  select(argc, argv, default_device);
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


unsigned Device::enumerateDevices(std::vector<DeviceInfo> &enumerated_devices)
{
  std::vector<cl::Device> devices;
  return ::enumerateDevices(devices, enumerated_devices);
}


const char *Device::name() const
{
  return imp_->info.name.c_str();
}


const char *Device::description() const
{
  return imp_->description.c_str();
}


const DeviceInfo &Device::info() const
{
  return imp_->info;
}


Queue Device::defaultQueue() const
{
  // Given queue constructor will not increment the reference count. Explicitly do so.
  clRetainCommandQueue(imp_->queue());
  return Queue(imp_->queue());
}


Queue Device::createQueue(unsigned flags) const
{
  cl_int clerr = CL_SUCCESS;
#if CL_HPP_TARGET_OPENCL_VERSION >= 200
  // FIXME: The following should only be used with OpenCL 2 compatible devices,
  // even if the SDK is version 2.0 compatible.
  // We need a device.version() function to address this.
  cl_command_queue_properties queue_props[] = { 0, 0, 0 };
  if (flags & Queue::kProfile)
  {
    queue_props[0] = CL_QUEUE_PROPERTIES;
    queue_props[1] |= CL_QUEUE_PROFILING_ENABLE;
  }
  cl_command_queue queue = clCreateCommandQueueWithProperties(imp_->context(), imp_->device(), queue_props, &clerr);
#else   // CL_HPP_TARGET_OPENCL_VERSION >= 200
  cl_command_queue_properties queue_props = 0;
  if (flags & Queue::kProfile)
  {
    queue_props |= CL_QUEUE_PROFILING_ENABLE;
  }
  cl_command_queue queue = clCreateCommandQueue(imp_->context(), imp_->device(), queue_props, &clerr);
#endif  // CL_HPP_TARGET_OPENCL_VERSION >= 200
  GPUAPICHECK(clerr, CL_SUCCESS, Queue());

  // This constructor will not add a reference.
  return Queue(queue);
}


bool Device::select(int argc, const char **argv, const char *default_device)
{
  cl_device_type device_type = 0;
  std::vector<clu::PlatformConstraint> platform_constraints;
  std::vector<clu::DeviceConstraint> device_constraints;

  clu::constraintsFromCommandLine(argc, argv, device_type, platform_constraints, device_constraints);

  // Add a platform constraint to at least 1.2 for the minimum supported API.
  platform_constraints.push_back(clu::platformVersionMin(1, 2));

  if (device_constraints.empty() && default_device && default_device[0])
  {
    device_constraints.push_back(clu::deviceNameLike(default_device, true));
  }

  imp_->context =
    clu::createContext(&imp_->device, device_type, platform_constraints.data(), unsigned(platform_constraints.size()),
                       device_constraints.data(), unsigned(device_constraints.size()));

  if (imp_->context())
  {
    imp_->queue = cl::CommandQueue(imp_->context, imp_->device);
    finaliseDetail(*imp_, nullptr);
  }
  else
  {
    imp_->context = cl::Context();
    imp_->device = cl::Device();
    imp_->queue = cl::CommandQueue();
  }

  if (isValid())
  {
    cl::Context context;
    cl::Device device;
    if (!clu::getPrimaryContext(context, device))
    {
      clu::setPrimaryContext(imp_->context, imp_->device);
    }

    // Add debug flag.
    for (int i = 0; i < argc; ++i)
    {
      if (strstr(argv[i], "--gpu-debug") == argv[i])
      {
        // Set default level.
        setDebugGpu(kDlLow);
        // Look for =<number> for more specific value.
        const char *arg_val = strstr(argv[i], "=");
        if (arg_val && arg_val[1])
        {
          ++arg_val;
          std::istringstream str(arg_val);
          int level = kDlLow;
          str >> level;
          if (kDlOff <= level && level <= kDlFull)
          {
            setDebugGpu(DebugLevel(level));
          }
        }
        break;
      }
    }

    return true;
  }

  return false;
}


bool Device::select(const DeviceInfo &device_info)
{
  std::vector<cl::Device> devices;
  std::vector<DeviceInfo> infos;
  ::enumerateDevices(devices, infos);

  for (size_t i = 0; i < infos.size(); ++i)
  {
    if (infos[i] == device_info)
    {
      // Found the device.
      if (i > devices.size())
      {
        return false;
      }

      imp_->device = devices[i];
      imp_->context = cl::Context(imp_->device);
      imp_->queue = cl::CommandQueue(imp_->context, imp_->device);
      finaliseDetail(*imp_, &device_info);
      if (isValid())
      {
        cl::Context context;
        cl::Device device;
        if (!clu::getPrimaryContext(context, device))
        {
          clu::setPrimaryContext(imp_->context, imp_->device);
        }
      }
      return true;
    }
  }

  return false;
}


void Device::setDebugGpu(DebugLevel debug_level)
{
  if (imp_)
  {
    imp_->debug = debug_level;
  }
}


Device::DebugLevel Device::debugGpu() const
{
  return (imp_) ? DebugLevel(imp_->debug) : kDlOff;
}


bool Device::supportsFeature(const char *feature_id) const
{
  if (imp_->extensions.find(feature_id) != std::string::npos)
  {
    return true;
  }

  return false;
}


void Device::addSearchPath(const char *path)
{
  if (imp_)
  {
    if (!imp_->search_paths.empty())
    {
      imp_->search_paths += ",";
      imp_->search_paths += path;
    }
    else
    {
      imp_->search_paths = path;
    }
  }
}


const char *Device::searchPaths() const
{
  return (imp_) ? imp_->search_paths.c_str() : "";
}


bool Device::isValid() const
{
  return imp_ && imp_->context();
}


uint64_t Device::deviceMemory() const
{
  if (!isValid())
  {
    return 0;
  }

  cl_ulong mem = 0;
  clGetDeviceInfo(imp_->device(), CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(mem), &mem, nullptr);

  return static_cast<uint64_t>(mem);
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
