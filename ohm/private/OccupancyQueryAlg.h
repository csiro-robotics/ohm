// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYQUERYALG_H
#define OCCUPANCYQUERYALG_H

#include "OhmConfig.h"

#include "OccupancyMap.h"

#include "private/OccupancyMapDetail.h"
#include "private/QueryDetail.h"

#ifdef OHM_THREADS
#include <tbb/blocked_range.h>
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>
#include <atomic>
#endif // OHM_THREADS

#include <functional>

// This file contains various algorithms to help execute queries on GPU.
// Common code, structures and contracts are presented here.

namespace ohm
{
  template <typename QUERY>
  unsigned occupancyQueryRegions(OccupancyMap &map, QUERY &query, ClosestResult &closest,
    const glm::dvec3 &queryMinExtents, const glm::dvec3 &queryMaxExtents,
    const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> &regionQueryFunc)
  {
    glm::i16vec3 minRegionKey;
    glm::i16vec3 maxRegionKey;
    glm::i16vec3 regionKey;
    unsigned currentNeighbours = 0;

    // Determine the maximum deltas in region indexing we can have based on the provided extents.
    minRegionKey = map.regionKey(queryMinExtents);
    maxRegionKey = map.regionKey(queryMaxExtents);

    // Iterate the regions, invoking regionQueryFunc for each.
    for (short z = minRegionKey.z; z <= maxRegionKey.z; ++z)
    {
      regionKey.z = z;
      for (short y = minRegionKey.y; y <= maxRegionKey.y; ++y)
      {
        regionKey.y = y;
        for (short x = minRegionKey.x; x <= maxRegionKey.x; ++x)
        {
          regionKey.x = x;
          currentNeighbours += regionQueryFunc(map, query, regionKey, closest);
        }
      }
    }

    return currentNeighbours;
  }

  template <typename QUERY> inline
    unsigned occupancyQueryRegions(OccupancyMap &map, QUERY &query, ClosestResult &closest,
      const glm::dvec3 &queryMinExtents, const glm::dvec3 &queryMaxExtents,
      unsigned(*regionQueryFunc)(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &))
  {
    const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> regionOp = regionQueryFunc;
    return occupancyQueryRegions(map, query, closest, queryMinExtents, queryMaxExtents, regionOp);
  }

  template <typename QUERY>
  unsigned occupancyQueryRegionsParallel(OccupancyMap &map, QUERY &query, ClosestResult &closest,
    const glm::dvec3 &queryMinExtents, const glm::dvec3 &queryMaxExtents,
    const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> &regionQueryFunc)
  {
#ifdef OHM_THREADS
    glm::i16vec3 minRegionKey;
    glm::i16vec3 maxRegionKey;
    std::atomic_uint currentNeighbours(0u);

    // Determine the maximum deltas in region indexing we can have based on the provided extents.
    minRegionKey = map.regionKey(queryMinExtents);
    maxRegionKey = map.regionKey(queryMaxExtents);

    tbb::parallel_for(tbb::blocked_range3d<size_t>(maxRegionKey.z, maxRegionKey.z + 1,
                                           maxRegionKey.y, maxRegionKey.y + 1,
                                           maxRegionKey.x, maxRegionKey.x + 1),
        [&currentNeighbours, &map, &query, &closest, &regionQueryFunc]
        (const tbb::blocked_range3d<int> &range)
        {
          glm::i16vec3 regionKey;
          for (int z = range.pages().begin(); z != range.pages().end(); ++z)
          {
            regionKey.z = z;
            for (int y = range.rows().begin(); y != range.rows().end(); ++y)
            {
              regionKey.y = y;
              for (int x = range.cols().begin(); x != range.cols().end(); ++x)
              {
                regionKey.x = x;
                currentNeighbours += regionQueryFunc(map, query, regionKey, closest);
              }
            }
          }
        }
      );

    return currentNeighbours;
#else  // OHM_THREADS
    return occupancyQueryRegions(map, query, closest, queryMinExtents, queryMaxExtents, regionQueryFunc);
#endif // OHM_THREADS
  }

  template <typename QUERY> inline
    unsigned occupancyQueryRegionsParallel(OccupancyMap &map, QUERY &query, ClosestResult &closest,
      const glm::dvec3 &queryMinExtents, const glm::dvec3 &queryMaxExtents,
      unsigned(*regionQueryFunc)(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &))
  {
    const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> regionOp = regionQueryFunc;
    return occupancyQueryRegionsParallel(map, query, closest, queryMinExtents, queryMaxExtents, regionOp);
  }
}

#endif // OCCUPANCYQUERYALG_H
