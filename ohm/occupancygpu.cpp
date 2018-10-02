// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancygpu.h"

#include <gputil/gpudevice.h>

#include <iostream>
#include <mutex>

using namespace ohm;

namespace
{
  gputil::Device _gpuDevice;
  std::mutex _gpuMutex;
  bool _gpuInitialised = false;
}


namespace ohm
{
  bool configureGpuFromArgsInternal(int argc, const char **argv, bool showDevice)
  {
    if (!_gpuInitialised)
    {
      if (_gpuDevice.select(argc, argv, "Intel"))
      {
        if (showDevice)
        {
          std::cout << _gpuDevice.info() << std::endl;
        }
        _gpuInitialised = true;
      }
    }
    return _gpuInitialised;
  }


  int configureGpuInternal(unsigned accel, const char *deviceName, bool showDevice)
  {
    std::string accelStr;
    std::string deviceStr;

    int argc = 0;
    const char *argv[2] = { nullptr };

    accelStr = "--accel=";
    if (accel == GpuAccel)
    {
      accelStr += "gpu";
      argv[argc] = accelStr.c_str();
      ++argc;
    }
    else if (accel == CpuAccel)
    {
      accelStr += "cpu";
      argv[argc] = accelStr.c_str();
      ++argc;
    }
    else if (accel == AnyAccel)
    {
      accelStr += "any";
      argv[argc] = accelStr.c_str();
      ++argc;
    }

    if (deviceName)
    {
      deviceStr = "--device=";
      deviceStr += deviceName;
      argv[argc] = deviceStr.c_str();
      ++argc;
    }

    return configureGpuFromArgsInternal(argc, (const char **)argv, showDevice);
  }


  int configureGpuFromArgs(int argc, const char **argv, bool showDevice)
  {
    int err = 0;
    std::unique_lock<std::mutex> lock(_gpuMutex);
    if (!configureGpuFromArgsInternal(argc, argv, showDevice))
    {
      err = 1;
    }
    return err;
  }


  int configureGpu(unsigned accel, const char *deviceName, bool showDevice)
  {
    int err = 0;
    std::unique_lock<std::mutex> lock(_gpuMutex);
    if (!configureGpuInternal(accel, deviceName, showDevice))
    {
      err = 1;
    }
    return err;
  }


  gputil::Device &gpuDevice()
  {
    std::unique_lock<std::mutex> lock(_gpuMutex);
    if (!_gpuInitialised)
    {
      configureGpuInternal(GpuAccel, nullptr, true);
    }
    return _gpuDevice;
  }


  unsigned ohm_API gpuArgsInfo(const char **argsInfo, int *argType, unsigned maxPairs)
  {
    // clang-format off
    struct ArgInfo
    {
      const char *name;
      const char *desc;
      int type;
    };
#if OHM_GPU == OHM_GPU_OPENCL
    const ArgInfo kArgPairs[] =
    {
      { "accel", "Select the OpenCL accelerator type [any,cpu,gpu] (gpu).", 1 },
      { "cl-ver", "Device must support target OpenCL version. Format via the regex /[1-9][0-9]*(.[1-9][0-9]*)?/.", 1 },
      { "device", "OpenCL device name must contain the given string (case insensitive).", 1 },
      { "gpu-debug", "Compile OpenCL GPU code for full debugging.", 0 },
      { "platform", "OpenCL platform name must contain the given string (case insensitive).", 1 },
      { "vendor", "OpenCL vendor name must contain the given string (case insensitive).", 1 },
    };
    const unsigned kArgPairCount = sizeof(kArgPairs) / sizeof(kArgPairs[0]);
#else  // OHM_GPU == OHM_GPU_OPENCL
    const ArgInfo kArgPairs[1] = { "", "" };
    const unsigned kArgPairCount = 0;
#endif // OHM_GPU == OHM_GPU_OPENCL
    // clang-format on

    if (argsInfo)
    {
      for (unsigned i = 0; i < kArgPairCount && i < maxPairs; ++i)
      {
        argsInfo[(i << 1) + 0] = kArgPairs[i].name;
        argsInfo[(i << 1) + 1] = kArgPairs[i].desc;
        if (argType)
        {
          argType[i] = kArgPairs[i].type;
        }
      }
    }

    return kArgPairCount;
  }
}
