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
  void finaliseDetail(DeviceDetail &detail)
  {
    detail.device.getInfo(CL_DEVICE_NAME, &detail.name);
    std::ostringstream str;
    clu::printDeviceInfo(str, detail.device, "");
    detail.info = str.str();
  }
}

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
      const std::vector<clu::PlatformContraint> platform_constraints;
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
      finaliseDetail(*imp_);
    }
  }
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


const char *Device::name() const
{
  return imp_->name.c_str();
}


const char *Device::info() const
{
  return imp_->info.c_str();
}


Queue Device::defaultQueue() const
{
  // Given queue constructor will not increment the reference count. Explicitly do so.
  clRetainCommandQueue(imp_->queue());
  return Queue(imp_->queue());
}


#if defined(CL_VERSION_1_2)
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif  // defined(__GNUC__)
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif  // defined(_MSC_VER)
#endif  // defined(CL_VERSION_1_2) && defined __GNUC__

Queue Device::createQueue(unsigned flags) const
{
  cl_int clerr = CL_SUCCESS;
#if defined(CL_VERSION_2_0) && false
  // FIXME: The following should only be used with OpenCL 2 compatible devices,
  // even if the SDK is version 2.0 compatible.
  // We need a device.version() function to address this.
  cl_command_queue_properties queueProps[] = { 0, 0, 0 };
  if (flags & Queue::Profile)
  {
    queueProps[0] = CL_QUEUE_PROPERTIES;
    queueProps[1] |= CL_QUEUE_PROFILING_ENABLE;
  }
  cl_command_queue queue = clCreateCommandQueueWithProperties(_imp->context(), _imp->device(), queueProps, &clerr);
#else  // !defined(CL_VERSION_2_0)
  cl_command_queue_properties queue_props = 0;
  if (flags & Queue::kProfile)
  {
    queue_props |= CL_QUEUE_PROFILING_ENABLE;
  }
  cl_command_queue queue = clCreateCommandQueue(imp_->context(), imp_->device(), queue_props, &clerr);
#endif // !defined(CL_VERSION_2_0)
  GPUAPICHECK(clerr, CL_SUCCESS, Queue());

  // This constructor will not add a reference.
  return Queue(queue);
}

#if defined(CL_VERSION_1_2)
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif  // defined(__GNUC__)
#if defined(_MSC_VER)
#pragma warning(pop)
#endif  // defined(_MSC_VER)
#endif  // defined(CL_VERSION_1_2) && defined __GNUC__


bool Device::select(int argc, const char **argv, const char *default_device)
{
  cl_device_type device_type = 0;
  std::vector<clu::PlatformContraint> platform_constraints;
  std::vector<clu::DeviceConstraint> device_constraints;

  clu::constraintsFromCommandLine(argc, argv, device_type, platform_constraints, device_constraints);

  if (device_constraints.empty() && default_device && default_device[0])
  {
    device_constraints.push_back(clu::deviceNameLike(default_device, true));
  }

  imp_->context = clu::createContext(&imp_->device, device_type,
                                     platform_constraints.data(), unsigned(platform_constraints.size()),
                                     device_constraints.data(), unsigned(device_constraints.size()));

  if (imp_->context())
  {
    imp_->queue = cl::CommandQueue(imp_->context, imp_->device);
    finaliseDetail(*imp_);
    clu::setPrimaryContext(imp_->context, imp_->device);
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


Device &Device::operator=(Device &&other) noexcept {
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}
