// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancynodealgorithms.h"

#include "mapcache.h"
#include "mapchunk.h"
#include "occupancykey.h"
#include "occupancymap.h"
#include "occupancynode.h"

#include <algorithm>
#include <limits>

namespace ohm
{
  glm::ivec3 calculateVoxelSearchHalfExtents(const OccupancyMap &map, float searchRadius)
  {
    return glm::ivec3(int(std::ceil(searchRadius / map.resolution())));
  }


  float calculateNearestNeighbour(const OccupancyKey &nodeKey, const OccupancyMap &map,
                                  const glm::ivec3 &voxelSearchHalfExtents,
                                  bool unknownAsOccupied, bool ignoreSelf, float searchRange,
                                  const glm::vec3 &axisScaling, bool reportUnscaledDistance)
  {
    OccupancyKey searchKey;
    OccupancyNodeConst testNode;
    MapCache cache;
    glm::vec3 nodeCentre, separation;

    float scaledRangeSqr, scaledClosestRangeSqr;
    float rangeSqr, closestRangeSqr;

    scaledClosestRangeSqr = closestRangeSqr = std::numeric_limits<float>::infinity();

    nodeCentre = map.voxelCentreLocal(nodeKey);

    testNode = map.node(nodeKey, &cache);
    // First try early out if the target node is occupied.
    if (!ignoreSelf && (testNode.isOccupied() || unknownAsOccupied && testNode.isUncertainOrNull()))
    {
      return 0.0f;
    }

    for (int z = -voxelSearchHalfExtents.z; z <= voxelSearchHalfExtents.z; ++z)
    {
      for (int y = -voxelSearchHalfExtents.y; y <= voxelSearchHalfExtents.y; ++y)
      {
        for (int x = -voxelSearchHalfExtents.x; x <= voxelSearchHalfExtents.x; ++x)
        {
          searchKey = nodeKey;
          map.moveKey(searchKey, x, y, z);
          testNode = map.node(searchKey, &cache);

          if (ignoreSelf && x == 0 && y == 0 && z == 0)
          {
            continue;
          }

          if (testNode.isOccupied() || unknownAsOccupied && testNode.isUncertainOrNull())
          {
            separation = glm::vec3(map.voxelCentreLocal(searchKey)) - nodeCentre;
            rangeSqr = glm::dot(separation, separation);
            separation.x *= axisScaling.x;
            separation.y *= axisScaling.y;
            separation.z *= axisScaling.z;
            scaledRangeSqr = glm::dot(separation, separation);
            if (!reportUnscaledDistance)
            {
              rangeSqr = scaledRangeSqr;
            }

            // Should this be scaledRangeSqr <= searchRange * searchRange?
            if (searchRange == 0 || rangeSqr <= searchRange * searchRange)
            {
              if (scaledRangeSqr < scaledClosestRangeSqr)
              {
                closestRangeSqr = rangeSqr;
                scaledClosestRangeSqr = scaledRangeSqr;
              }
            }
          }
        }
      }
    }

    if (closestRangeSqr < std::numeric_limits<float>::infinity())
    {
      return std::sqrt(float(closestRangeSqr));
    }

    return -1.0f;
  }
}
