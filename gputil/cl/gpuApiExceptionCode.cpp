// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuApiException.h"

#include <clu/clu.h>

using namespace gputil;

const char *ApiException::errorCodeString(int error_code)
{
  return clu::errorCodeString(error_code);
}
