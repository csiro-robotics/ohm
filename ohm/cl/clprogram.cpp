// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <clu/clu.h>
#include <clu/clukernel.h>
#include <clu/cluprogram.h>

#include <gputil/cl/gpudevicedetail.h>
#include <gputil/gpudevice.h>

#include <algorithm>
#include <sstream>

namespace ohm
{
  int initProgramFromSource(cl::Program &program, const gputil::Device &gpu, const char *sourceFileName,
                            const std::vector<std::string> *buildArgs)
  {
    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    std::ostringstream debugOpt;
    std::string sourceFile = sourceFileName;
    const char *sourceFileOpt = nullptr;

    std::string platformName;
    cl_platform_id platformId;

    gputil::DeviceDetail &ocl = *gpu.detail();
    ocl.device.getInfo(CL_DEVICE_PLATFORM, &platformId);
    cl::Platform platform(platformId);
    platform.getInfo(CL_PLATFORM_VENDOR, &platformName);
    std::transform(platformName.begin(), platformName.end(), platformName.begin(), ::tolower);
    // For Intel platforms, add debug compilation and source file option as we may debug
    // using the Intel SDK.
    if (platformName.find("intel") != std::string::npos)
    {
      sourceFileOpt = "-s ";
      if (gpu.debugGpu())
      {
        switch (gpu.debugGpu())
        {
        case 2:
          debugOpt << "-cl-opt-disable ";
          // Don't break. Cascade to enable the next option.
        case 1:
          debugOpt << "-g ";
          break;

        default:
          break;
        }
      }
    }

    if (gpu.debugGpu())
    {
      debugOpt << "-D DEBUG";
    }

    clerr = clu::buildProgramFromFile(program, ocl.context, sourceFile, std::cerr, buildArgs, debugOpt.str().c_str(),
                                      sourceFileOpt, gpu.searchPaths());

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      return clerr;
    }

    return clerr;
  }


  int initProgramFromString(cl::Program &program, const gputil::Device &gpu, const char *sourceString,
                            const char *referenceName,
                            const std::vector<std::string> *buildArgs)
  {
    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    std::ostringstream debugOpt, buildOpt;

    std::string platformName;
    cl_platform_id platformId;

    gputil::DeviceDetail &ocl = *gpu.detail();
    ocl.device.getInfo(CL_DEVICE_PLATFORM, &platformId);
    cl::Platform platform(platformId);
    platform.getInfo(CL_PLATFORM_VENDOR, &platformName);
    std::transform(platformName.begin(), platformName.end(), platformName.begin(), ::tolower);
    // For Intel platforms, add debug compilation and source file option as we may debug
    // using the Intel SDK.
    if (platformName.find("intel") != std::string::npos)
    {
      if (gpu.debugGpu())
      {
        if (gpu.debugGpu())
        {
          switch (gpu.debugGpu())
          {
          case 2:
            debugOpt << "-cl-opt-disable ";
            // Don't break. Cascade to enable the next option.
            break;
          case 1:
            debugOpt << "-g ";
            break;

          default:
            break;
          }
        }
      }
    }

    if (gpu.debugGpu())
    {
      debugOpt << "-D DEBUG";
    }

    if (buildArgs)
    {
      bool firstArg = true;
      for (auto &&arg : *buildArgs)
      {
        if (!firstArg)
        {
          buildOpt << ' ';
        }
        buildOpt << arg;
        firstArg = false;
      }
    }

    std::string buildOptStr = buildOpt.str();
    clerr = clu::buildProgramFromString(program, ocl.context, sourceString, 0u, std::cerr, referenceName,
                                        buildOptStr.c_str(), debugOpt.str().c_str());

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      return clerr;
    }

    return clerr;
  }


  void calculateGrid(clu::KernelGrid &grid, clu::Kernel &kernel, const gputil::Device &gpu, const cl_int3 &calcExtents)
  {
    const size_t calcVolume = calcExtents.x * calcExtents.y * calcExtents.z;
    const size_t targetGroupSize = std::min(kernel.calculateOptimalWorkGroupSize(), calcVolume);
    size_t maxWorkGroupSize = 0;

    // Try to setup the workgroup as a cubic spatial division.
    // However we have to consider where the max work item dimensions don't allow this.
    cl_uint groupDim = 3;
    clGetDeviceInfo(gpu.detail()->device(), CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(groupDim), &groupDim, nullptr);
    size_t *maxWorkSize = (size_t *)alloca(sizeof(size_t) * groupDim);
    clGetDeviceInfo(gpu.detail()->device(), CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(*maxWorkSize) * groupDim, maxWorkSize, nullptr);
    maxWorkSize[0] = std::min<size_t>(maxWorkSize[0], calcExtents.x);
    maxWorkSize[1] = std::min<size_t>(maxWorkSize[1], calcExtents.y);
    maxWorkSize[2] = std::min<size_t>(maxWorkSize[2], calcExtents.z);

    unsigned targetDimensionValue = unsigned(std::floor(std::pow(float(targetGroupSize), 1.0f / 3.0f)));
    if (targetDimensionValue < 1)
    {
      targetDimensionValue = 1;
    }

    // Set the target dimensions to the minimum of the target and the max work group size.
    grid.workGroupSize[2] = std::min<size_t>(maxWorkSize[2], targetDimensionValue);
    targetDimensionValue = unsigned(std::floor(std::pow(float(targetGroupSize / grid.workGroupSize[2]), 1.0f / 2.0f)));
    grid.workGroupSize[1] = std::min<size_t>(maxWorkSize[1], targetDimensionValue);
    targetDimensionValue = (unsigned)std::max<size_t>(targetGroupSize / (grid.workGroupSize[1] * grid.workGroupSize[2]), 1);
    grid.workGroupSize[0] = std::min<size_t>(maxWorkSize[0], targetDimensionValue);

    // Reduce size to <= targetGroupSize
    int i = 2; // Start by reducing 3rd dimension.
    while (i >= 0 && grid.workGroupSize.volume() > targetGroupSize)
    {
      if (grid.workGroupSize[i] > 1)
      {
        grid.workGroupSize[i] = grid.workGroupSize[i] -1;
      }
      else
      {
        // Don't reduce to less than 1. Next dimension.
        --i;
      }
    }

    // max work item sizes
    grid.globalSize[0] = calcExtents.x;
    grid.globalSize[1] = calcExtents.y;
    grid.globalSize[2] = calcExtents.z;

    grid.globalSize = grid.adjustedGlobal();
  }
}
