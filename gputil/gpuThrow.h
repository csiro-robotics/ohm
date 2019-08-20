// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUTHROW_H
#define GPUTHROW_H

#include "gpuConfig.h"

#define GPU_EXCEPTIONS 1

#if GPU_EXCEPTIONS
// Return statement present to prevent compilation errors when switching GPU_EXCEPTIONS.
#define GPUTHROW(e, r) \
  {                    \
    throw(e);          \
    return r;          \
  }
#define GPUTHROW2(e) throw e
#else  // GPU_EXCEPTIONS
#define GPUTHROW(e, r)                  \
  {                                     \
    gputil::log(e, __FILE__, __LINE__); \
    return r;                           \
  }
#define GPUTHROW2(e)                    \
  {                                     \
    gputil::log(e, __FILE__, __LINE__); \
    return;                             \
  }
#endif  // GPU_EXCEPTIONS

#define GPUAPICHECK(err, expect, r)                                      \
  if ((err) != (expect))                                                 \
  {                                                                      \
    GPUTHROW(gputil::ApiException(err, nullptr, __FILE__, __LINE__), r); \
  }
#define GPUAPICHECK2(err, expect)                                      \
  if ((err) != (expect))                                               \
  {                                                                    \
    GPUTHROW2(gputil::ApiException(err, nullptr, __FILE__, __LINE__)); \
  }

namespace gputil
{
  class Exception;

  void gputilAPI log(const Exception &e);
  void gputilAPI log(const Exception &e, const char *file, int line);
}  // namespace gputil

#endif  // GPUTHROW_H