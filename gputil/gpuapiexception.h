// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUGPUAPICHECK_H_
#define GPUGPUAPICHECK_H_

#include "gpuconfig.h"

#include "gpuexception.h"

namespace gputil
{
  /// Raised due to an exception in the underlying GPU SDK.
  class gputilAPI ApiException : public Exception
  {
  public:
    ApiException(int errorCode, const char *msg = nullptr);
    ApiException(ApiException &&other);

    static const char *errorCodeString(int errorCode);

    inline int errorCode() const { return _errorCode; }

  private:
    int _errorCode;
  };
}

#endif // GPUGPUAPICHECK_H_
