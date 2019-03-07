// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMTESTUTIL_H
#define OHMTESTUTIL_H

#include <glm/fwd.hpp>

namespace ohm
{
  class OccupancyMap;
}

namespace ohmtestutil
{
  enum CompareFlag
  {
    kCfGeneral = (1 << 0),
    kCfChunksGeneral = (1 << 1),
    /// Fine detail comparison of chunks. Requires @c CF_ChunksGeneral.
    kCfChunksFine = (1 << 2),
    kCfLayout = (1 << 3),
    kCfOccupancy = (1 << 4),
    kCfClearance = (1 << 5),
    kCfExpectClearance = (1 << 6),

    kCfDefault = kCfGeneral | kCfChunksGeneral | kCfOccupancy,
    kCfCompareAll = kCfGeneral | kCfChunksGeneral | kCfLayout | kCfOccupancy | kCfClearance,
    kCfCompareFine = kCfGeneral | kCfChunksGeneral | kCfChunksFine | kCfLayout | kCfOccupancy | kCfClearance
  };

  const char *applicationDir();
  void setApplicationPath(const char *path);

  bool compareLayout(const ohm::OccupancyMap &map, const ohm::OccupancyMap &reference_map);

  void compareMaps(const ohm::OccupancyMap &map, const ohm::OccupancyMap &reference_map,
                   unsigned compare_flags = kCfDefault);

  void compareMaps(const ohm::OccupancyMap &map, const ohm::OccupancyMap &reference_map, const glm::dvec3 &min_ext,
                   const glm::dvec3 &max_ext, unsigned compare_flags = kCfDefault);
}  // namespace ohmtestutil

#endif  // OHMTESTUTIL_H
