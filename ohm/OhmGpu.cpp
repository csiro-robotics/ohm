// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGpu.h"

#include <gputil/gpuDevice.h>

#include <iostream>
#include <mutex>

using namespace ohm;

namespace
{
  gputil::Device gpu_device;
  std::mutex gpu_mutex;
  bool gpu_initialised = false;
}


namespace ohm
{
  bool configureGpuFromArgsInternal(int argc, const char **argv, bool show_device)
  {
    if (!gpu_initialised)
    {
      if (gpu_device.select(argc, argv, "Intel"))
      {
        if (show_device)
        {
          std::cout << gpu_device.info() << std::endl;
        }
        gpu_initialised = true;
      }
    }
    return gpu_initialised;
  }


  int configureGpuInternal(unsigned accel, const char *device_name, bool show_device)
  {

    int argc = 0;
    const char *argv[2] = { nullptr };

    std::string accel_str = "--accel=";
    if (accel == GpuAccel)
    {
      accel_str += "gpu";
      argv[argc] = accel_str.c_str();
      ++argc;
    }
    else if (accel == CpuAccel)
    {
      accel_str += "cpu";
      argv[argc] = accel_str.c_str();
      ++argc;
    }
    else if (accel == AnyAccel)
    {
      accel_str += "any";
      argv[argc] = accel_str.c_str();
      ++argc;
    }

    if (device_name)
    {
      std::string device_str = "--device=";
      device_str += device_name;
      argv[argc] = device_str.c_str();
      ++argc;
    }

    return configureGpuFromArgsInternal(argc, static_cast<const char **>(argv), show_device);
  }


  int configureGpuFromArgs(int argc, const char **argv, bool show_device)
  {
    int err = 0;
    std::unique_lock<std::mutex> lock(gpu_mutex);
    if (!configureGpuFromArgsInternal(argc, argv, show_device))
    {
      err = 1;
    }
    return err;
  }


  int configureGpu(unsigned accel, const char *device_name, bool show_device)
  {
    int err = 0;
    std::unique_lock<std::mutex> lock(gpu_mutex);
    if (!configureGpuInternal(accel, device_name, show_device))
    {
      err = 1;
    }
    return err;
  }


  gputil::Device &gpuDevice()
  {
    std::unique_lock<std::mutex> lock(gpu_mutex);
    if (!gpu_initialised)
    {
      configureGpuInternal(GpuAccel, nullptr, true);
    }
    return gpu_device;
  }


  unsigned ohm_API gpuArgsInfo(const char **args_info, int *arg_type, unsigned max_pairs)
  {
    // clang-format off
    struct ArgInfo
    {
      const char *name;
      const char *desc;
      int type;
    };
#if OHM_GPU == OHM_GPU_OPENCL
    const ArgInfo arg_pairs[] =
    {
      { "accel", "Select the OpenCL accelerator type [any,cpu,gpu] (gpu).", 1 },
      { "cl-ver", "Device must support target OpenCL version. Format via the regex /[1-9][0-9]*(.[1-9][0-9]*)?/.", 1 },
      { "device", "OpenCL device name must contain the given string (case insensitive).", 1 },
      { "gpu-debug", "Compile OpenCL GPU code for full debugging.", 0 },
      { "platform", "OpenCL platform name must contain the given string (case insensitive).", 1 },
      { "vendor", "OpenCL vendor name must contain the given string (case insensitive).", 1 },
    };
    const unsigned arg_pair_count = sizeof(arg_pairs) / sizeof(arg_pairs[0]);
#else  // OHM_GPU == OHM_GPU_OPENCL
    const ArgInfo arg_pairs[1] = { "", "", 0 };
    unsigned arg_pair_count = 0;
#endif // OHM_GPU == OHM_GPU_OPENCL
    // clang-format on

    if (args_info)
    {
      for (unsigned i = 0; i < arg_pair_count && i < max_pairs; ++i)
      {
        args_info[(i << 1) + 0] = arg_pairs[i].name;
        args_info[(i << 1) + 1] = arg_pairs[i].desc;
        if (arg_type)
        {
          arg_type[i] = arg_pairs[i].type;
        }
      }
    }

    return arg_pair_count;
  }
}
