// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUVERSION_H
#define GPUVERSION_H

#include "gpuConfig.h"

#include <cinttypes>

namespace gputil
{
  struct Version
  {
    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;

    inline bool operator==(const Version &other) const
    {
      return major == other.major && minor == other.minor && patch == other.patch;
    }
  };
}  // namespace gputil

#endif  // GPUVERSION_H
