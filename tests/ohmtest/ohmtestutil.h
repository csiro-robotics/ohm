// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMTESTUTIL_H_
#define OHMTESTUTIL_H_

#include <glm/fwd.hpp>

namespace ohm
{
  class OccupancyMap;
}

namespace ohmtestutil
{
  enum CompareFlag
  {
    CF_General = (1 << 0),
    CF_ChunksGeneral = (1 << 1),
    /// Fine detail comparison of chunks. Requires @c CF_ChunksGeneral.
    CF_ChunksFine = (1 << 2),
    CF_Layout = (1 << 3),
    CF_Occupancy = (1 << 4),
    CF_Clearance = (1 << 5),
    CF_ExpectClearance = (1 << 6),

    CF_Default = CF_General | CF_ChunksGeneral | CF_Occupancy,
    CF_CompareAll = CF_General | CF_ChunksGeneral | CF_Layout | CF_Occupancy | CF_Clearance,
    CF_CompareFine = CF_General | CF_ChunksGeneral | CF_ChunksFine | CF_Layout | CF_Occupancy | CF_Clearance
  };

  bool compareLayout(const ohm::OccupancyMap &map, const ohm::OccupancyMap &referenceMap);

  void compareMaps(const ohm::OccupancyMap &map, const ohm::OccupancyMap &referenceMap,
                   unsigned compareFlags = CF_Default);

  void compareMaps(const ohm::OccupancyMap &map, const ohm::OccupancyMap &referenceMap, const glm::dvec3 &minExt,
                   const glm::dvec3 maxExt, unsigned compareFlags = CF_Default);
}

#endif  // OHMTESTUTIL_H_
