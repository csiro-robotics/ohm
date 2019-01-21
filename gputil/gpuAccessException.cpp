// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuAccessException.h"

#include <utility>

using namespace gputil;

#define DEFAULT_MSG "Attempting write access to read only memory."

AccessException::AccessException(const char *msg)
  : Exception(msg ? msg : DEFAULT_MSG)
{}


AccessException::AccessException(AccessException &&other) noexcept
  : Exception(std::move(other))
{}
