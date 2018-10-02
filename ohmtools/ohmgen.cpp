// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmgen.h"

#include <ohm/occupancykey.h>
#include <ohm/occupancykeylist.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancyutil.h>
#include <ohm/occupancytype.h>
#include <ohm/mapcache.h>

#include <exception>

using namespace ohm;

namespace ohmgen
{
  void fillMapWithEmptySpace(OccupancyMap &map, int x1, int y1, int z1, int x2, int y2, int z2, bool expectEmptyMap)
  {
    const double mapRes = map.resolution();
    OccupancyNode node;
    OccupancyKey key;
    MapCache cache;
    float initialValue;
    float expectValue;

    for (int z = z1; z < z2; ++z)
    {
      for (int y = y1; y < y2; ++y)
      {
        for (int x = x1; x < x2; ++x)
        {
          key = OccupancyKey(0, 0, 0, 0, 0, 0);
          map.moveKey(key, x, y, z);
          node = map.node(key, true, &cache);
          initialValue = (!node.isNull()) ? node.value() : OccupancyNode::invalidMarkerValue();
          expectValue = (initialValue == OccupancyNode::invalidMarkerValue()) ? map.missValue() : initialValue + map.missValue();
          expectValue = (map.saturateAtMinValue() && expectValue < map.minNodeValue()) ? map.minNodeValue() : expectValue;
          if (expectEmptyMap && initialValue != OccupancyNode::invalidMarkerValue())
          {
            throw std::logic_error("Voxel should start uncertain.");
          }
          node.setValue(map.missValue());
        }
      }
    }
  }


  void cubicRoom(OccupancyMap &map, float boundaryRange, int voxelStep)
  {
    int extents = int(boundaryRange / map.resolution());

    ohm::MapCache cache;
    auto buildWalls = [&map, extents, voxelStep, &cache] (int a0, int a1, int a2)
    {
      const double mapRes = map.resolution();
      OccupancyNodeConst node;
      OccupancyKey key;
      OccupancyKeyList ray;
      glm::dvec3 point;
      glm::dvec3 origin = map.origin();
      for (int i = -extents; i < extents + 1; i += voxelStep)
      {
        for (int j = -extents; j < extents + 1; j += voxelStep)
        {
          for (int k = 0; k < 2; ++k)
          {
            point = map.origin();
            point[a0] = i * mapRes;
            point[a1] = j * mapRes;
            point[a2] = (k == 0 ? (-extents) * map.resolution() : (extents - 1) * map.resolution());
            map.node(map.voxelKey(point), true, &cache).setValue(map.occupancyThresholdValue() + map.hitValue());
          }
        }
      }
    };

    fillMapWithEmptySpace(map, -extents + 1, -extents + 1, -extents + 1, extents, extents, extents);
    buildWalls(0, 1, 2);
    buildWalls(1, 2, 0);
    buildWalls(0, 2, 1);
  }
}
