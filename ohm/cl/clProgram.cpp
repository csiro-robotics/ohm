// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <clu/clu.h>
#include <clu/cluKernel.h>
#include <clu/cluProgram.h>

#include <gputil/cl/gpuDeviceDetail.h>
#include <gputil/gpuDevice.h>

#include <algorithm>
#include <sstream>

namespace ohm
{
  int initProgramFromSource(cl::Program &program, const gputil::Device &gpu, const char *source_file_name,
                            const std::vector<std::string> *build_args)
  {
    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    std::ostringstream debug_opt;
    std::string source_file = source_file_name;
    const char *source_file_opt = nullptr;

    std::string platform_name;
    cl_platform_id platform_id;

    gputil::DeviceDetail &ocl = *gpu.detail();
    ocl.device.getInfo(CL_DEVICE_PLATFORM, &platform_id);
    cl::Platform platform(platform_id);
    platform.getInfo(CL_PLATFORM_VENDOR, &platform_name);
    std::transform(platform_name.begin(), platform_name.end(), platform_name.begin(), ::tolower);
    // For Intel platforms, add debug compilation and source file option as we may debug
    // using the Intel SDK.
    if (platform_name.find("intel") != std::string::npos)
    {
      source_file_opt = "-s ";
      if (gpu.debugGpu())
      {
        switch (gpu.debugGpu())
        {
        case 2:
          debug_opt << "-cl-opt-disable ";
          // Don't break. Cascade to enable the next option.
        case 1:
          debug_opt << "-g ";
          break;

        default:
          break;
        }
      }
    }

    if (gpu.debugGpu())
    {
      debug_opt << "-D DEBUG";
    }

    std::ostringstream build_args_str;
    if (build_args)
    {
      bool first_arg = true;
      for (auto &&arg : *build_args)
      {
        if (!first_arg)
        {
          build_args_str << ' ';
        }
        build_args_str << arg;
      }
    }

    clerr = clu::buildProgramFromFile(program, ocl.context, source_file, std::cerr, build_args_str.str().c_str(),
                                      debug_opt.str().c_str(), source_file_opt, gpu.searchPaths());

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      return clerr;
    }

    return clerr;
  }


  int initProgramFromString(cl::Program &program, const gputil::Device &gpu, const char *source_string,
                            const char *reference_name, const std::vector<std::string> *build_args)
  {
    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    std::ostringstream debug_opt, build_opt;

    std::string platform_name;
    cl_platform_id platform_id;

    gputil::DeviceDetail &ocl = *gpu.detail();
    ocl.device.getInfo(CL_DEVICE_PLATFORM, &platform_id);
    cl::Platform platform(platform_id);
    platform.getInfo(CL_PLATFORM_VENDOR, &platform_name);
    std::transform(platform_name.begin(), platform_name.end(), platform_name.begin(), ::tolower);
    // For Intel platforms, add debug compilation and source file option as we may debug
    // using the Intel SDK.
    if (platform_name.find("intel") != std::string::npos)
    {
      if (gpu.debugGpu())
      {
        if (gpu.debugGpu())
        {
          switch (gpu.debugGpu())
          {
          case 2:
            debug_opt << "-cl-opt-disable ";
            // Don't break. Cascade to enable the next option.
            break;
          case 1:
            debug_opt << "-g ";
            break;

          default:
            break;
          }
        }
      }
    }

    if (gpu.debugGpu())
    {
      debug_opt << "-D DEBUG";
    }

    if (build_args)
    {
      bool first_arg = true;
      for (auto &&arg : *build_args)
      {
        if (!first_arg)
        {
          build_opt << ' ';
        }
        build_opt << arg;
        first_arg = false;
      }
    }

    std::string build_opt_str = build_opt.str();
    clerr = clu::buildProgramFromString(program, ocl.context, source_string, 0u, std::cerr, reference_name,
                                        build_opt_str.c_str(), debug_opt.str().c_str());

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      return clerr;
    }

    return clerr;
  }


  void calculateGrid(clu::KernelGrid &grid, clu::Kernel &kernel, const gputil::Device &gpu, const cl_int3 &calc_extents)
  {
    const size_t calc_volume = calc_extents.x * calc_extents.y * calc_extents.z;
    const size_t target_group_size = std::min(kernel.calculateOptimalWorkGroupSize(), calc_volume);

    // Try to setup the workgroup as a cubic spatial division.
    // However we have to consider where the max work item dimensions don't allow this.
    cl_uint group_dim = 3;
    clGetDeviceInfo(gpu.detail()->device(), CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(group_dim), &group_dim, nullptr);
    size_t *max_work_size = static_cast<size_t *>(alloca(sizeof(size_t) * group_dim));
    clGetDeviceInfo(gpu.detail()->device(), CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(*max_work_size) * group_dim,
                    max_work_size, nullptr);
    max_work_size[0] = std::min<size_t>(max_work_size[0], calc_extents.x);
    max_work_size[1] = std::min<size_t>(max_work_size[1], calc_extents.y);
    max_work_size[2] = std::min<size_t>(max_work_size[2], calc_extents.z);

    unsigned target_dimension_value = unsigned(std::floor(std::pow(float(target_group_size), 1.0f / 3.0f)));
    if (target_dimension_value < 1)
    {
      target_dimension_value = 1;
    }

    // Set the target dimensions to the minimum of the target and the max work group size.
    grid.work_group_size[2] = std::min<size_t>(max_work_size[2], target_dimension_value);
    target_dimension_value =
      unsigned(std::floor(std::pow(float(target_group_size / grid.work_group_size[2]), 1.0f / 2.0f)));
    grid.work_group_size[1] = std::min<size_t>(max_work_size[1], target_dimension_value);
    target_dimension_value =
      unsigned(std::max<size_t>(target_group_size / (grid.work_group_size[1] * grid.work_group_size[2]), 1));
    grid.work_group_size[0] = std::min<size_t>(max_work_size[0], target_dimension_value);

    // Reduce size to <= targetGroupSize
    int i = 2;  // Start by reducing 3rd dimension.
    while (i >= 0 && grid.work_group_size.volume() > target_group_size)
    {
      if (grid.work_group_size[i] > 1)
      {
        grid.work_group_size[i] = grid.work_group_size[i] - 1;
      }
      else
      {
        // Don't reduce to less than 1. Next dimension.
        --i;
      }
    }

    // max work item sizes
    grid.global_size[0] = calc_extents.x;
    grid.global_size[1] = calc_extents.y;
    grid.global_size[2] = calc_extents.z;

    grid.global_size = grid.adjustedGlobal();
  }
}  // namespace ohm
