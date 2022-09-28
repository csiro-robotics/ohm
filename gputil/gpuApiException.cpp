// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuApiException.h"

#include <sstream>
#include <utility>

namespace gputil
{
ApiException::ApiException(const int error_code, const char *const msg, const char *const filename,
                           const int line_number)
  : error_code_(error_code)
{
  if (msg)
  {
    setMessage(msg, (filename == nullptr) ? "" : filename, line_number);
  }
  else
  {
    std::ostringstream str;
    str << "API error (" << error_code << ") " << errorCodeString(error_code);
    setMessage(str.str(), (filename == nullptr) ? "" : filename, line_number);
  }
}


ApiException::ApiException(ApiException &&other) noexcept
  : Exception(std::move(other))
  , error_code_(other.error_code_)
{}


ApiException::ApiException(const ApiException &other) noexcept
  : Exception(other)
  , error_code_(other.error_code_)
{}
}  // namespace gputil
