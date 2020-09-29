// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelAlgorithms.h"

#include "Key.h"
#include "OccupancyMap.h"
#include "VoxelData.h"

#include <limits>

namespace ohm
{
  glm::ivec3 calculateVoxelSearchHalfExtents(const OccupancyMap &map, float search_radius)
  {
    return glm::ivec3(int(std::ceil(search_radius / map.resolution())));
  }


  float calculateNearestNeighbour(const Key &voxel_key, const OccupancyMap &map,
                                  const glm::ivec3 &voxel_search_half_extents, bool unobserved_as_occupied,
                                  bool ignore_self, float search_range, const glm::vec3 &axis_scaling,
                                  bool report_unscaled_distance)
  {
    Key search_key;
    Voxel<const float> test_voxel(&map, map.layout().occupancyLayer());
    glm::vec3 voxel_centre, separation;

    float scaled_range_sqr, scaled_closest_range_sqr;
    float range_sqr, closest_range_sqr;

    scaled_closest_range_sqr = closest_range_sqr = std::numeric_limits<float>::infinity();

    voxel_centre = map.voxelCentreLocal(voxel_key);

    test_voxel.setKey(voxel_key);
    // First try early out if the target voxel is occupied.
    if (!ignore_self &&
        (test_voxel.isValid() && isOccupied(test_voxel) || unobserved_as_occupied && isUnobservedOrNull(test_voxel)))
    {
      return 0.0f;
    }

    for (int z = -voxel_search_half_extents.z; z <= voxel_search_half_extents.z; ++z)
    {
      for (int y = -voxel_search_half_extents.y; y <= voxel_search_half_extents.y; ++y)
      {
        for (int x = -voxel_search_half_extents.x; x <= voxel_search_half_extents.x; ++x)
        {
          search_key = voxel_key;
          map.moveKey(search_key, x, y, z);
          test_voxel.setKey(search_key);

          if (ignore_self && x == 0 && y == 0 && z == 0)
          {
            continue;
          }

          if (test_voxel.isValid() && isOccupied(test_voxel) ||
              unobserved_as_occupied && isUnobservedOrNull(test_voxel))
          {
            separation = glm::vec3(map.voxelCentreLocal(search_key)) - voxel_centre;
            range_sqr = glm::dot(separation, separation);
            separation.x *= axis_scaling.x;
            separation.y *= axis_scaling.y;
            separation.z *= axis_scaling.z;
            scaled_range_sqr = glm::dot(separation, separation);
            if (!report_unscaled_distance)
            {
              range_sqr = scaled_range_sqr;
            }

            // Should this be scaledRangeSqr <= searchRange * searchRange?
            if (search_range == 0 || range_sqr <= search_range * search_range)
            {
              if (scaled_range_sqr < scaled_closest_range_sqr)
              {
                closest_range_sqr = range_sqr;
                scaled_closest_range_sqr = scaled_range_sqr;
              }
            }
          }
        }
      }
    }

    if (closest_range_sqr < std::numeric_limits<float>::infinity())
    {
      return std::sqrt(float(closest_range_sqr));
    }

    return -1.0f;
  }
}  // namespace ohm
