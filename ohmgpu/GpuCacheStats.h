// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPUCACHESTATS_H
#define OHMGPU_GPUCACHESTATS_H

#include "OhmGpuConfig.h"

#include <cinttypes>

namespace ohm
{
/// Running stats on a @c GpuLayerCache .
struct ohmgpu_API GpuCacheStats
{
  uint32_t hits = 0;    ///< Number of cache hits
  uint32_t misses = 0;  ///< Number of cache misses.
  uint32_t full = 0;    ///< Number of misses where the cache was full and something had to be dropped.
};
}  // namespace ohm

#endif  // OHMGPU_GPUCACHESTATS_H
