// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpudevice.h"

#include "gpuapiexception.h"
#include "gpudevicedetail.h"

#include <clu/clu.h>
#include <clu/cluconstraint.h>

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

Device::Device(bool defaultDevice)
  : _imp(new DeviceDetail)
{
  if (defaultDevice)
  {
    if (!clu::getPrimaryContext(_imp->context, _imp->device))
    {
      // Needs initialisation.
      // Empty constraints.
      cl_device_type deviceType = CL_DEVICE_TYPE_GPU | CL_DEVICE_TYPE_ACCELERATOR;
      std::vector<clu::PlatformContraint> platformConstraints;
      std::vector<clu::DeviceConstraint> deviceConstraints;
      clu::initPrimaryContext(deviceType, platformConstraints, deviceConstraints);
      clu::getPrimaryContext(_imp->context, _imp->device);

      _imp->queue = cl::CommandQueue(_imp->context, _imp->device);
    }

    if (isValid())
    {
      std::vector<cl::Device> devices;
      clu::listDevices(devices, _imp->context);
      _imp->device = devices[0];
      _imp->queue = cl::CommandQueue(_imp->context, _imp->device);
      finaliseDetail(*_imp);
    }
  }
}


Device::Device(int argc, const char **argv, const char *defaultDevice)
  : _imp(new DeviceDetail)
{
  select(argc, argv, defaultDevice);
}


Device::Device(const Device &other)
  : _imp(new DeviceDetail)
{
  *_imp = *other._imp;
}


Device::Device(Device &&other)
  : _imp(other._imp)
{
  other._imp = nullptr;
}


Device::~Device()
{
  delete _imp;
}


const char *Device::name() const
{
  return _imp->name.c_str();
}


const char *Device::info() const
{
  return _imp->info.c_str();
}


Queue Device::defaultQueue() const
{
  // Given queue constructor will not increment the reference count. Explicitly do so.
  clRetainCommandQueue(_imp->queue());
  return Queue(_imp->queue());
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
  cl_command_queue_properties queueProps = 0;
  if (flags & Queue::Profile)
  {
    queueProps |= CL_QUEUE_PROFILING_ENABLE;
  }
  cl_command_queue queue = clCreateCommandQueue(_imp->context(), _imp->device(), queueProps, &clerr);
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


bool Device::select(int argc, const char **argv, const char *defaultDevice)
{
  cl_device_type deviceType = 0;
  std::vector<clu::PlatformContraint> platformConstraints;
  std::vector<clu::DeviceConstraint> deviceConstraints;

  clu::constraintsFromCommandLine(argc, argv, deviceType, platformConstraints, deviceConstraints);

  if (deviceConstraints.empty() && defaultDevice && defaultDevice[0])
  {
    deviceConstraints.push_back(clu::deviceNameLike(defaultDevice, true));
  }

  _imp->context = clu::createContext(&_imp->device, deviceType,
                                     platformConstraints.data(), unsigned(platformConstraints.size()),
                                     deviceConstraints.data(), unsigned(deviceConstraints.size()));

  if (_imp->context())
  {
    _imp->queue = cl::CommandQueue(_imp->context, _imp->device);
    finaliseDetail(*_imp);
    clu::setPrimaryContext(_imp->context, _imp->device);
  }
  else
  {
    _imp->context = cl::Context();
    _imp->device = cl::Device();
    _imp->queue = cl::CommandQueue();
  }

  if (isValid())
  {
    cl::Context context;
    cl::Device device;
    if (!clu::getPrimaryContext(context, device))
    {
      clu::setPrimaryContext(_imp->context, _imp->device);
    }

    // Add debug flag.
    for (int i = 0; i < argc; ++i)
    {
      if (strstr(argv[i], "--gpu-debug") == argv[i])
      {
        // Set default level.
        setDebugGpu(DL_Low);
        // Look for =<number> for more specific value.
        const char *argVal = strstr(argv[i], "=");
        if (argVal && argVal[1])
        {
          ++argVal;
          std::istringstream str(argVal);
          int level = DL_Low;
          str >> level;
          if (DL_Off <= level && level <= DL_Full)
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


void Device::setDebugGpu(DebugLevel debugLevel)
{
  if (_imp)
  {
    _imp->debug = debugLevel;
  }
}


Device::DebugLevel Device::debugGpu() const
{
  return (_imp) ? DebugLevel(_imp->debug) : DL_Off;
}


void Device::addSearchPath(const char *path)
{
  if (_imp)
  {
    if (!_imp->searchPaths.empty())
    {
      _imp->searchPaths += ",";
      _imp->searchPaths += path;
    }
    else
    {
      _imp->searchPaths = path;
    }
  }
}


const char *Device::searchPaths() const
{
  return (_imp) ? _imp->searchPaths.c_str() : "";
}


bool Device::isValid() const
{
  return _imp && _imp->context();
}


uint64_t Device::deviceMemory() const
{
  if (!isValid())
  {
    return 0;
  }

  cl_ulong mem = 0;
  clGetDeviceInfo(_imp->device(), CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(mem), &mem, nullptr);

  return (uint64_t)mem;
}


Device &Device::operator=(const Device &other)
{
  *_imp = *other._imp;
  return *this;
}


Device &Device::operator=(Device &&other)
{
  delete _imp;
  _imp = other._imp;
  other._imp = nullptr;
  return *this;
}
