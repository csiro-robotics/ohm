// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuProgram.h"

#include "gputil/cuda/gpuProgramDetail.h"

namespace gputil
{
Program::Program() = default;


Program::Program(Device &device, const char *program_name)
  : imp_(std::make_unique<ProgramDetail>())
{
  imp_->device = device;
  imp_->program_name = program_name;
}


Program::Program(Program &&other) noexcept
  : imp_(std::move(other.imp_))
{}


Program::Program(const Program &other)
{
  *this = other;
}


Program::~Program() = default;


bool Program::isValid() const
{
  return imp_ != nullptr && imp_->device.isValid();
}

const char *Program::programName() const
{
  return imp_->program_name.c_str();
}

Device Program::device()
{
  return (imp_) ? imp_->device : Device();
}

// Lint(KS): API implementation compatibility requirement. Cannot make static
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
int Program::buildFromFile(const char * /*file_name*/, const BuildArgs & /*build_args*/)
{
  return 0;
}

// Lint(KS): API implementation compatibility requirement. Cannot make static
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
int Program::buildFromSource(const char * /*source*/, size_t /*source_length*/, const BuildArgs & /*build_args*/)
{
  return 0;
}

Program &Program::operator=(const Program &other)
{
  if (this != &other)
  {
    if (other.imp_)
    {
      if (!imp_)
      {
        imp_ = std::make_unique<ProgramDetail>();
      }
      imp_->device = other.imp_->device;
      imp_->program_name = other.imp_->program_name;
    }
    else
    {
      imp_.reset();
    }
  }
  return *this;
}

Program &Program::operator=(Program &&other) noexcept
{
  imp_ = std::move(other.imp_);
  return *this;
}
}  // namespace gputil
