// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuProgram.h"

#include "gpuDeviceDetail.h"
#include "gpuProgramDetail.h"

#include <clu/cluProgram.h>

#include <algorithm>
#include <sstream>

using namespace gputil;

namespace
{
  void prepareDebugBuildArgs(const gputil::Device &gpu, const BuildArgs &build_args, std::ostream &debug_opt,
                             std::ostream &build_opt, std::string &source_file_opt)
  {
    // Compile and initialise.
    source_file_opt = std::string();

    std::string platform_name;
    cl_platform_id platform_id;

    gputil::DeviceDetail &ocl = *gpu.detail();
    ocl.device.getInfo(CL_DEVICE_PLATFORM, &platform_id);
    cl::Platform platform(platform_id);
    platform.getInfo(CL_PLATFORM_VENDOR, &platform_name);
    std::transform(platform_name.begin(), platform_name.end(), platform_name.begin(), ::tolower);

    int debug_level = std::max<int>(gpu.debugGpu(), build_args.debug_level);

    // FIXME: work out how to resolve additional debug arguments, such as the source file argument
    // for Intel, but not the beignet drivers.
    // For Intel platforms, add debug compilation and source file option as we may debug
    // using the Intel SDK.
    if (platform_name.find("intel") != std::string::npos)
    {
#ifdef WIN32
      source_file_opt = "-s";
#endif // WIN32
      switch (gpu.debugGpu())
      {
      case 2:
        debug_opt << "-cl-opt-disable ";
        // Don't break. Cascade to enable the next option.
        /* fall through */
      case 1:
#ifdef WIN32
        debug_opt << "-g ";
#endif // WIN32
        break;

      default:
        break;
      }
    }

    if (debug_level)
    {
      debug_opt << "-D DEBUG";
    }

    bool first_arg = true;
    if (build_args.version_major > 0 && build_args.version_minor >= 0)
    {
      build_opt << "-cl-std=CL";
      build_opt << build_args.version_major << "." << build_args.version_minor;
      first_arg = false;
    }

    if (build_args.args)
    {
      for (auto &&arg : *build_args.args)
      {
        if (!first_arg)
        {
          build_opt << ' ';
        }
        build_opt << arg;
        first_arg = false;
      }
    }
  }
}  // namespace


Program::Program()
  : imp_(nullptr)
{}


Program::Program(Device &device, const char *program_name)
  : imp_(new ProgramDetail)
{
  imp_->device = device;
  imp_->program_name = program_name;
}


Program::Program(Program &&other)
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Program::~Program()
{
  delete imp_;
}


bool Program::isValid() const
{
  return imp_ && imp_->program();
}


const char *Program::programName() const
{
  return imp_->program_name.c_str();
}


Device Program::device()
{
  if (isValid())
  {
    return imp_->device;
  }

  return Device();
}


int Program::buildFromFile(const char *file_name, const BuildArgs &build_args)
{
  std::ostringstream debug_opt, build_opt;
  std::string source_file_opt;
  cl::Context &ocl_context = imp_->device.detail()->context;
  prepareDebugBuildArgs(imp_->device, build_args, debug_opt, build_opt, source_file_opt);

  std::string source_file_name(file_name);
  cl_int clerr =
    clu::buildProgramFromFile(imp_->program, ocl_context, source_file_name, std::cerr, build_opt.str().c_str(),
                              debug_opt.str().c_str(), source_file_opt.c_str(), imp_->device.searchPaths());

  if (clerr != CL_SUCCESS)
  {
    imp_->program = cl::Program();
    return clerr;
  }

  return clerr;
}


int Program::buildFromSource(const char *source, size_t source_length, const BuildArgs &build_args)
{
  std::ostringstream debug_opt, build_opt;
  std::string source_file_opt;
  cl::Context &ocl_context = imp_->device.detail()->context;
  prepareDebugBuildArgs(imp_->device, build_args, debug_opt, build_opt, source_file_opt);

  cl_int clerr = clu::buildProgramFromString(imp_->program, ocl_context, source, source_length, std::cerr,
                                             programName(), debug_opt.str().c_str(), source_file_opt.c_str());

  if (clerr != CL_SUCCESS)
  {
    imp_->program = cl::Program();

    return clerr;
  }

  return clerr;
}


Program &Program::operator=(Program &&other)
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}
