// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGen.h"

#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/MapCache.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>

using namespace ohm;

namespace ohmgen
{
  void fillWithValue(OccupancyMap &map,  // NOLINT(google-runtime-references)
                     const Key &min_key, const Key &max_key, float fill_value, const float *expect_value, int step)
  {
    Voxel voxel;
    MapCache cache;
    float initial_value;

    Key key = min_key;
    for (; key.isBoundedZ(min_key, max_key); map.moveKeyAlongAxis(key, 2, step))
    {
      key.setRegionAxis(1, min_key.regionKey()[1]);
      key.setLocalAxis(1, min_key.localKey()[1]);

      for (; key.isBoundedY(min_key, max_key); map.moveKeyAlongAxis(key, 1, step))
      {
        key.setRegionAxis(0, min_key.regionKey()[0]);
        key.setLocalAxis(0, min_key.localKey()[0]);

        for (; key.isBoundedX(min_key, max_key); map.moveKeyAlongAxis(key, 0, step))
        {
          voxel = map.voxel(key, true, &cache);
          initial_value = voxel::invalidMarkerValue();
          if (expect_value && initial_value != *expect_value)
          {
            throw std::logic_error("Voxel should start uncertain.");
          }
          voxel.setValue(fill_value);
        }
      }
    }
  }


  void fillMapWithEmptySpace(ohm::OccupancyMap &map, int x1, int y1, int z1, int x2, int y2, int z2,
                             bool expect_empty_map)
  {
    const float expect_initial_value = voxel::invalidMarkerValue();
    const float *expect_value_ptr = (expect_empty_map) ? &expect_initial_value : nullptr;

    Key min_key, max_key;
    min_key = max_key = Key(0, 0, 0, 0, 0, 0);

    map.moveKey(min_key, x1, y1, z1);
    map.moveKey(max_key, x2 - 1, y2 - 1, z2 - 1);

    fillWithValue(map, min_key, max_key, map.missValue(), expect_value_ptr, 1);
  }


  void buildWall(OccupancyMap &map,  // NOLINT(google-runtime-references)
                 int a0, int a1, int a2, int a0min, int a1min, int a0max, int a1max, int a2val)
  {
    Voxel voxel;
    Key key;
    MapCache cache;

    for (int val0 = a0min; val0 < a0max; ++val0)
    {
      for (int val1 = a1min; val1 < a1max; ++val1)
      {
        key = Key(0, 0, 0, 0, 0, 0);
        map.moveKeyAlongAxis(key, a0, val0);
        map.moveKeyAlongAxis(key, a1, val1);
        map.moveKeyAlongAxis(key, a2, a2val);
        voxel = map.voxel(key, true, &cache);
        voxel.setValue(map.occupancyThresholdValue());
      }
    }
  }

  void boxRoom(ohm::OccupancyMap &map, const glm::dvec3 &min_ext, const glm::dvec3 &max_ext, int voxel_step)
  {
    const Key min_key = map.voxelKey(min_ext);
    const Key max_key = map.voxelKey(max_ext);
    Key wall_key;

    fillWithValue(map, min_key, max_key, map.missValue(), nullptr, 1);

    // Left wall.
    wall_key = max_key;
    wall_key.setLocalAxis(0, min_key.localKey()[0]);
    wall_key.setRegionAxis(0, min_key.regionKey()[0]);
    fillWithValue(map, min_key, wall_key, map.occupancyThresholdValue(), nullptr, voxel_step);
    // Front wall
    wall_key = max_key;
    wall_key.setLocalAxis(1, min_key.localKey()[1]);
    wall_key.setRegionAxis(1, min_key.regionKey()[1]);
    fillWithValue(map, min_key, wall_key, map.occupancyThresholdValue(), nullptr, voxel_step);
    // Bottom wall
    wall_key = max_key;
    wall_key.setLocalAxis(2, min_key.localKey()[2]);
    wall_key.setRegionAxis(2, min_key.regionKey()[2]);
    fillWithValue(map, min_key, wall_key, map.occupancyThresholdValue(), nullptr, voxel_step);


    // Left wall.
    wall_key = min_key;
    wall_key.setLocalAxis(0, max_key.localKey()[0]);
    wall_key.setRegionAxis(0, max_key.regionKey()[0]);
    fillWithValue(map, wall_key, max_key, map.occupancyThresholdValue(), nullptr, voxel_step);
    // Front wall
    wall_key = min_key;
    wall_key.setLocalAxis(1, max_key.localKey()[1]);
    wall_key.setRegionAxis(1, max_key.regionKey()[1]);
    fillWithValue(map, wall_key, max_key, map.occupancyThresholdValue(), nullptr, voxel_step);
    // Bottom wall
    wall_key = min_key;
    wall_key.setLocalAxis(2, max_key.localKey()[2]);
    wall_key.setRegionAxis(2, max_key.regionKey()[2]);
    fillWithValue(map, wall_key, max_key, map.occupancyThresholdValue(), nullptr, voxel_step);
  }


  void slope(ohm::OccupancyMap &map, double angle_deg, const glm::dvec3 &min_ext, const glm::dvec3 &max_ext,
             int voxel_step)
  {
    ohm::Key o_key = map.voxelKey(min_ext);
    int range_x = int(std::ceil((max_ext.x - min_ext.x) / map.resolution()));
    int range_y = int(std::ceil((max_ext.y - min_ext.y) / map.resolution()));

    const double tan_theta = std::tan(angle_deg * M_PI / 180.0);
    glm::dvec3 coord;
    for (int y = 0; y < range_y; y += voxel_step)
    {
      for (int x = 0; x < range_x; x += voxel_step)
      {
        ohm::Key key = o_key;
        map.moveKey(key, x, y, 0);
        coord = map.voxelCentreGlobal(key);
        coord.z = min_ext.z + double(coord.y) * tan_theta;
        Voxel voxel = map.voxel(map.voxelKey(coord), true);
        voxel.setValue(map.occupancyThresholdValue());
      }
    }
  }
}  // namespace ohmgen
