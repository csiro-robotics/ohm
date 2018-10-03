// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUACCESSEXCEPTION_H
#define GPUACCESSEXCEPTION_H

#include "gpuConfig.h"

#include "gpuException.h"

namespace gputil
{
  class gputilAPI AccessException : public Exception
  {
  public:
    AccessException(const char *msg = nullptr);
    AccessException(AccessException &&other) noexcept;
  };
}

#endif // GPUACCESSEXCEPTION_H
