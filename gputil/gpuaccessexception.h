// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUACCESSEXCEPTION_H_
#define GPUACCESSEXCEPTION_H_

#include "gpuconfig.h"

#include "gpuexception.h"

namespace gputil
{
  class gputilAPI AccessException : public Exception
  {
  public:
    AccessException(const char *msg = nullptr);
    AccessException(AccessException &&other);
  };
}

#endif // GPUACCESSEXCEPTION_H_
