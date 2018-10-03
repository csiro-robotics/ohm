// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUGPUAPICHECK_H
#define GPUGPUAPICHECK_H

#include "gpuConfig.h"

#include "gpuException.h"

namespace gputil
{
  /// Raised due to an exception in the underlying GPU SDK.
  class gputilAPI ApiException : public Exception
  {
  public:
    ApiException(int error_code, const char *msg = nullptr);
    ApiException(ApiException &&other) noexcept;

    static const char *errorCodeString(int error_code);

    inline int errorCode() const { return error_code_; }

  private:
    int error_code_;
  };
}

#endif // GPUGPUAPICHECK_H
