// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuKernel.h"

#include "gpuApiException.h"
#include "gpuKernelDetail.h"
#include "gpuProgram.h"
#include "gpuProgramDetail.h"
#include "gpuThrow.h"

using namespace gputil;

Kernel::Kernel()
  : imp_(new KernelDetail)
{
  imp_->kernel.setLocalMemFirst(false);
}


Kernel::Kernel(Kernel &&other)
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Kernel::~Kernel()
{
  delete imp_;
}


bool Kernel::isValid() const
{
  return imp_ && imp_->kernel.isValid();
}


void Kernel::addLocal(const std::function<size_t(size_t)> &local_calc)
{
  imp_->kernel.addLocal(local_calc);
}


size_t Kernel::calculateOptimalWorkGroupSize()
{
  return imp_->kernel.calculateOptimalWorkGroupSize();
}


size_t Kernel::optimalWorkGroupSize() const
{
  return imp_->kernel.optimalWorkGroupSize();
}


Device Kernel::device()
{
  if (!isValid())
  {
    return Device();
  }

  return imp_->program.device();
}


Kernel &Kernel::operator=(Kernel &&other)
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}


namespace gputil
{
  Kernel openCLKernel(Program &program, const char *kernel_name)
  {
    Kernel kernel;
    cl_int err = kernel.detail()->kernel.setEntry(program.detail()->program, kernel_name);
    GPUTHROW(ApiException(err), Kernel());
    return kernel;
  }
}  // namespace gputil
