// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuapiexception.h"

#include <clu/clu.h>

using namespace gputil;

const char *ApiException::errorCodeString(int errorCode)
{
  return clu::errorCodeString(errorCode);
}
