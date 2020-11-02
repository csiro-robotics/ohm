// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYQUERYALG_H
#define OCCUPANCYQUERYALG_H

#include "OhmConfig.h"

#include "ohm/OccupancyMap.h"

#include "ohm/private/OccupancyMapDetail.h"
#include "ohm/private/QueryDetail.h"

#ifdef OHM_THREADS
#include <tbb/blocked_range.h>
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>

#include <atomic>
#endif  // OHM_THREADS

#include <functional>

// This file contains various algorithms to help execute queries on GPU.
// Common code, structures and contracts are presented here.

namespace ohm
{
template <typename QUERY>
unsigned occupancyQueryRegions(
  OccupancyMap &map, QUERY &query, ClosestResult &closest, const glm::dvec3 &query_min_extents,
  const glm::dvec3 &query_max_extents,
  const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> &region_query_func)
{
  glm::i16vec3 min_region_key;
  glm::i16vec3 max_region_key;
  glm::i16vec3 region_key;
  unsigned current_neighbours = 0;

  // Determine the maximum deltas in region indexing we can have based on the provided extents.
  min_region_key = map.regionKey(query_min_extents);
  max_region_key = map.regionKey(query_max_extents);

  // Iterate the regions, invoking region_query_func for each.
  for (int16_t z = min_region_key.z; z <= max_region_key.z; ++z)
  {
    region_key.z = z;
    for (int16_t y = min_region_key.y; y <= max_region_key.y; ++y)
    {
      region_key.y = y;
      for (int16_t x = min_region_key.x; x <= max_region_key.x; ++x)
      {
        region_key.x = x;
        current_neighbours += region_query_func(map, query, region_key, closest);
      }
    }
  }

  return current_neighbours;
}

template <typename QUERY>
inline unsigned occupancyQueryRegions(OccupancyMap &map, QUERY &query, ClosestResult &closest,
                                      const glm::dvec3 &query_min_extents, const glm::dvec3 &query_max_extents,
                                      unsigned (*region_query_func)(OccupancyMap &, QUERY &, const glm::i16vec3 &,
                                                                    ClosestResult &))
{
  const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> region_op =
    region_query_func;
  return occupancyQueryRegions(map, query, closest, query_min_extents, query_max_extents, region_op);
}

template <typename QUERY>
unsigned occupancyQueryRegionsParallel(
  OccupancyMap &map, QUERY &query, ClosestResult &closest, const glm::dvec3 &query_min_extents,
  const glm::dvec3 &query_max_extents,
  const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> &region_query_func)
{
#ifdef OHM_THREADS
  glm::i16vec3 min_region_key;
  glm::i16vec3 max_region_key;
  std::atomic_uint current_neighbours(0u);

  // Determine the maximum deltas in region indexing we can have based on the provided extents.
  min_region_key = map.regionKey(query_min_extents);
  max_region_key = map.regionKey(query_max_extents);

  tbb::parallel_for(
    tbb::blocked_range3d<size_t>(max_region_key.z, max_region_key.z + 1, max_region_key.y, max_region_key.y + 1,
                                 max_region_key.x, max_region_key.x + 1),
    [&current_neighbours, &map, &query, &closest, &region_query_func](const tbb::blocked_range3d<int> &range) {
      glm::i16vec3 region_key;
      for (int z = range.pages().begin(); z != range.pages().end(); ++z)
      {
        region_key.z = z;
        for (int y = range.rows().begin(); y != range.rows().end(); ++y)
        {
          region_key.y = y;
          for (int x = range.cols().begin(); x != range.cols().end(); ++x)
          {
            region_key.x = x;
            current_neighbours += region_query_func(map, query, region_key, closest);
          }
        }
      }
    });

  return current_neighbours;
#else   // OHM_THREADS
  return occupancyQueryRegions(map, query, closest, query_min_extents, query_max_extents, region_query_func);
#endif  // OHM_THREADS
}

template <typename QUERY>
inline unsigned occupancyQueryRegionsParallel(OccupancyMap &map, QUERY &query, ClosestResult &closest,
                                              const glm::dvec3 &query_min_extents, const glm::dvec3 &query_max_extents,
                                              unsigned (*region_query_func)(OccupancyMap &, QUERY &,
                                                                            const glm::i16vec3 &, ClosestResult &))
{
  const std::function<unsigned(OccupancyMap &, QUERY &, const glm::i16vec3 &, ClosestResult &)> region_op =
    region_query_func;
  return occupancyQueryRegionsParallel(map, query, closest, query_min_extents, query_max_extents, region_op);
}
}  // namespace ohm

#endif  // OCCUPANCYQUERYALG_H
