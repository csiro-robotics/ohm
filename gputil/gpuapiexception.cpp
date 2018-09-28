// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuapiexception.h"

#include <sstream>
#include <utility>

using namespace gputil;

#define DEFAULT_MSG "Attempting write access to read only memory."

ApiException::ApiException(int errorCode, const char *msg)
  : Exception(msg)
  , _errorCode(errorCode)
{
  if (!msg)
  {
    std::ostringstream str;
    str << "API error " << errorCodeString(errorCode) << " (" << errorCode << ")";
    setMessage(str.str().c_str());
  }
}


ApiException::ApiException(ApiException &&other)
  : Exception(std::move(other))
  , _errorCode(other._errorCode)
{

}
