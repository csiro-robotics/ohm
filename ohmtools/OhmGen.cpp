// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGen.h"

#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/MapCache.h>

using namespace ohm;

namespace ohmgen
{
  void fillMapWithEmptySpace(OccupancyMap &map, int x1, int y1, int z1, int x2, int y2, int z2, bool expect_empty_map)
  {
    OccupancyNode node;
    OccupancyKey key;
    MapCache cache;
    float initial_value;
    float expect_value;

    for (int z = z1; z < z2; ++z)
    {
      for (int y = y1; y < y2; ++y)
      {
        for (int x = x1; x < x2; ++x)
        {
          key = OccupancyKey(0, 0, 0, 0, 0, 0);
          map.moveKey(key, x, y, z);
          node = map.node(key, true, &cache);
          initial_value = (!node.isNull()) ? node.value() : OccupancyNode::invalidMarkerValue();
          expect_value = (initial_value == OccupancyNode::invalidMarkerValue()) ? map.missValue() : initial_value + map.missValue();
          expect_value = (map.saturateAtMinValue() && expect_value < map.minNodeValue()) ? map.minNodeValue() : expect_value;
          if (expect_empty_map && initial_value != OccupancyNode::invalidMarkerValue())
          {
            throw std::logic_error("Voxel should start uncertain.");
          }
          node.setValue(map.missValue());
        }
      }
    }
  }


  void cubicRoom(OccupancyMap &map, float boundary_range, int voxel_step)
  {
    int extents = int(boundary_range / map.resolution());

    ohm::MapCache cache;
    const auto build_walls = [&map, extents, voxel_step, &cache] (int a0, int a1, int a2)
    {
      const double map_res = map.resolution();
      OccupancyKeyList ray;
      glm::dvec3 point;
      for (int i = -extents; i < extents + 1; i += voxel_step)
      {
        for (int j = -extents; j < extents + 1; j += voxel_step)
        {
          for (int k = 0; k < 2; ++k)
          {
            point = map.origin();
            point[a0] = i * map_res;
            point[a1] = j * map_res;
            point[a2] = (k == 0 ? (-extents) * map.resolution() : (extents - 1) * map.resolution());
            map.node(map.voxelKey(point), true, &cache).setValue(map.occupancyThresholdValue() + map.hitValue());
          }
        }
      }
    };

    fillMapWithEmptySpace(map, -extents + 1, -extents + 1, -extents + 1, extents, extents, extents);
    build_walls(0, 1, 2);
    build_walls(1, 2, 0);
    build_walls(0, 2, 1);
  }
}
