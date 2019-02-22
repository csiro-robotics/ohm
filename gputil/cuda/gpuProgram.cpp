// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuProgram.h"

#include "gputil/cuda/gpuProgramDetail.h"

using namespace gputil;

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

int Program::buildFromFile(const char * /*file_name*/, const BuildArgs &/*build_args*/)
{
  return 0;
}

int Program::buildFromSource(const char * /*source*/, size_t /*source_length*/, const BuildArgs &/*build_args*/)
{
  return 0;
}

Program &Program::operator=(const Program &other)
{
  if (other.imp_)
  {
    if (!imp_)
    {
      imp_ = new ProgramDetail;
    }
    imp_->device = other.imp_->device;
    imp_->program_name = other.imp_->program_name;
  }
  else
  {
    delete imp_;
    imp_ = nullptr;
  }
  return *this;
}

Program &Program::operator=(Program &&other)
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}
