// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuProgramRef.h"
#include "OhmGpu.h"

#include <gputil/gpuKernel.h>

namespace ohm
{
GpuProgramRef::GpuProgramRef(const char *name, SourceType source_type, const char *source_str, size_t source_str_length)
  : name_(name)
  , source_str_(source_str, source_str_length)
  , source_type_(source_type)
{
  // Handle no length provided by setting again. Constructor will not have copied anything.
  if (!source_str_length)
  {
    source_str_ = source_str;
  }
}


GpuProgramRef::GpuProgramRef(const char *name, SourceType source_type, const char *source_str, size_t source_str_length,
                             const std::initializer_list<std::string> &build_args)
  : GpuProgramRef(name, source_type, source_str, source_str_length)
{
  build_args_ = build_args;
}


GpuProgramRef::~GpuProgramRef()
{
  releaseReference();
}


bool GpuProgramRef::addReference(gputil::Device &gpu)
{
  std::unique_lock<std::mutex> guard(program_mutex_);
  if (program_ref_ == 0)
  {
    gputil::BuildArgs build_args;
    ohm::setGpuBuildVersion(build_args);
    build_args.args = &build_args_;

    int err = 0;
    program_ = gputil::Program(gpu, name_.c_str());
    // Lint(KS): macro may generate the same code, but depends on GPU API.
    if (source_type_ == kSourceFile)  // NOLINT(bugprone-branch-clone)
    {
      err = GPUTIL_BUILD_FROM_FILE(program_, source_str_.c_str(), build_args);
    }
    else
    {
      err = GPUTIL_BUILD_FROM_SOURCE(program_, source_str_.c_str(), source_str_.size(), build_args);
    }

    if (err)
    {
      program_ = gputil::Program();
      return false;
    }
  }

  ++program_ref_;
  return true;
}


void GpuProgramRef::releaseReference()
{
  std::unique_lock<std::mutex> guard(program_mutex_);

  if (program_ref_ > 0)
  {
    --program_ref_;
    if (!program_ref_)
    {
      program_ = gputil::Program();
    }
  }
}


bool GpuProgramRef::isValid()
{
  std::unique_lock<std::mutex> guard(program_mutex_);
  return program_ref_ > 0;
}
}  // namespace ohm
