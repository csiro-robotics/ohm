//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPER_H
#define RAYMAPPER_H

#include "OhmConfig.h"

#include "KeyList.h"
#include "MapCache.h"
#include "RayFilter.h"
#include "RayFlag.h"
#include "RayMapperBase.h"
#include "RayMapperInterface.h"
#include "Voxel.h"

#include <glm/vec3.hpp>

namespace ohm
{
  template <typename MAP, typename MAPINTERFACE = RayMapperInterface<MAP>>
  class RayMapper : public RayMapperBase
  {
  public:
    using MapInterface = MAPINTERFACE;

    RayMapper(MAP *map) : map_(map) {}

    size_t integrateRays(const glm::dvec3 *rays, size_t element_count,
                           unsigned ray_update_flags = kRfDefault) override;

  private:
    MAP *map_;
  };


  template <typename MAP, typename MAPINTERFACE>
  size_t RayMapper<MAP, MAPINTERFACE>::integrateRays(const glm::dvec3 *rays, size_t element_count,
                                                     unsigned ray_update_flags)
  {
    KeyList keys;
    MapCache cache;
    bool clipped_sample_voxel;

    const OccupancyMap &occupancy_map = MapInterface::occupancyMap(*map_);
    const float occupancy_threshold_value = occupancy_map.occupancyThresholdValue();
    const RayFilterFunction ray_filter = MapInterface::rayFilter(*map_);
    const bool use_filter = bool(ray_filter);

    glm::dvec3 start, end;
    unsigned filter_flags;
    for (size_t i = 0; i < element_count; i += 2)
    {
      filter_flags = 0;
      start = rays[i];
      end = rays[i + 1];

      if (use_filter)
      {
        if (!ray_filter(&start, &end, &filter_flags))
        {
          // Bad ray.
          continue;
        }
      }

      clipped_sample_voxel = (filter_flags & kRffClippedEnd);

      // Calculate line key for the last voxel if the end point has been clipped
      calculateSegmentKeys(occupancy_map, keys, start, end, clipped_sample_voxel);

      for (auto &&key : keys)
      {
        Voxel voxel = MapInterface::voxel(*map_, key, &cache);
        const float voxel_value = voxel.value();

        bool stop_traversal = false;
        if ((ray_update_flags & kRfStopOnFirstOccupied) && voxel_value >= occupancy_threshold_value &&
            voxel_value != voxel::invalidMarkerValue())
        {
          // Found first occupied voxel and request is to stop on the first occupied voxel. Abort traversal after update.
          stop_traversal = true;
        }

        // kRfClearOnly flag set => only affect occupied 7s.
        if (!(ray_update_flags & kRfClearOnly) || voxel_value >= occupancy_threshold_value)
        {
          MapInterface::integrateMiss(*map_, voxel, start, end);
        }

        if (stop_traversal)
        {
          // Found first occupied voxel and request is to stop on the first occupied voxel. Abort traversal.
          // Make sure we do not update the en voxel.
          clipped_sample_voxel = true;
          break;
        }
      }

      if (!clipped_sample_voxel)
      {
        Voxel voxel = MapInterface::voxel(*map_, MapInterface::voxelKey(*map_, rays[i + 1]), &cache);
        if (!(ray_update_flags & kRfEndPointAsFree))
        {
          MapInterface::integrateHit(*map_, voxel, start, end);
        }
        else
        {
          MapInterface::integrateMiss(*map_, voxel, start, end);
        }
      }
    }

    return element_count / 2;
  }
}  // namespace ohm


#endif  // RAYMAPPER_H
