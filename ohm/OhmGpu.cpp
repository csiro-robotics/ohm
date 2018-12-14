// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGpu.h"

#include <gputil/cl/gpuDeviceDetail.h>
#include <gputil/gpuDevice.h>

#include <gputil/gpuProgram.h>

#include <iostream>
#include <mutex>
#include <sstream>

using namespace ohm;

namespace
{
  gputil::Device gpu_device;
  std::mutex gpu_mutex;
  std::string gpu_build_std_arg;
  unsigned gpu_std_major = 0;
  unsigned gpu_std_minor = 0;
  bool gpu_initialised = false;
}  // namespace


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
          std::cout << gpu_device.description() << std::endl;
        }

#if OHM_GPU == OHM_GPU_OPENCL
        // Is --clver on the command line?
        bool found_cl_ver = false;
        for (int i = 1; i < argc; ++i)
        {
          const char *cl_ver_arg = "--clver";
          const size_t cl_arg_len = strlen(cl_ver_arg);
          if (strncmp(argv[i], cl_ver_arg, cl_arg_len) == 0)
          {
            // Found --clver.
            // Read the version string.
            cl_uint ver_major, ver_minor;
            std::string cl_ver = argv[i] + cl_arg_len + 1;
            found_cl_ver = clu::parseVersion(cl_ver.c_str(), &ver_major, &ver_minor);
            gpu_std_major = ver_major;
            gpu_std_minor = ver_minor;
          }
        }

        if (!found_cl_ver)
        {
          if (strcmp(OHM_OPENCL_STD, "max") == 0)
          {
            // Requesting the maximum available version.
            // Query the device.
            gpu_std_major = gpu_device.info().version.major;
            gpu_std_minor = gpu_device.info().version.minor;
          }
          else
          {
            cl_uint ver_major, ver_minor;
            clu::parseVersion(OHM_OPENCL_STD, &ver_major, &ver_minor);
            gpu_std_major = ver_major;
            gpu_std_minor = ver_minor;
          }
        }

        // Validate OpenCL extensions for 2.x are available.
        std::ostringstream str;
        str << "-cl-std=CL" << gpu_std_major << "." << gpu_std_minor;
        gpu_build_std_arg = str.str();
        // Validate extensions.

        bool extensions_supported = true;
        if (gpu_std_major >= 2)
        {
          // Tokenise required OpenCL feature list
          const char *feature_str = OHM_OPENCL_2_FEATURES;
          do
          {
            const char *begin = feature_str;

            while (*feature_str != ';' && *feature_str)
            {
              ++feature_str;
            }

            if (*begin != '\0')
            {
              std::string feature(begin, feature_str);
              if (!gpu_device.supportsFeature(feature.c_str()))
              {
                std::cerr << "Missing OpenCL 2+ feature: " << feature << std::endl;
                extensions_supported = false;
              }
            }
          } while (*feature_str++ != '\0');
        }

        if (!extensions_supported)
        {
          std::cerr << "Fallback to OpenCL 1.2" << std::endl;
          gpu_build_std_arg = "-cl-std=CL1.2";
          gpu_std_major = 1;
          gpu_std_minor = 2;
        }
#endif  // OHM_GPU == OHM_GPU_OPENCL


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
    if (accel == kGpuAccel)
    {
      accel_str += "gpu";
      argv[argc] = accel_str.c_str();
      ++argc;
    }
    else if (accel == kCpuAccel)
    {
      accel_str += "cpu";
      argv[argc] = accel_str.c_str();
      ++argc;
    }
    else if (accel == kAnyAccel)
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
      configureGpuInternal(kGpuAccel, nullptr, true);
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
      { "clver", "Sets the OpenCL runtime version. Selected device must support target OpenCL version. Format via the regex /[1-9][0-9]*(.[1-9][0-9]*)?/.", 1 },
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


  const char ohm_API *gpuBuildStdArg() { return gpu_build_std_arg.c_str(); }


  void setGpuBuildVersion(gputil::BuildArgs &build_args)
  {
    // Resolve the requested from the configuration define.
    build_args.version_major = gpu_std_major;
    build_args.version_minor = gpu_std_minor;
  }
}  // namespace ohm
