// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/MapCache.h>
#include <ohm/OhmGpu.h>
#include <ohm/GpuMap.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/LineQuery.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/ClearanceProcess.h>
#include <ohm/Mapper.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include "OhmTestUtil.h"
#include <gtest/gtest.h>

using namespace ohm;
using namespace ohmutil;

namespace maptests
{
  TEST(Map, Hit)
  {
    OccupancyMap map(0.25);
    Key key(0, 0, 0, 0, 0, 0);
    map.integrateHit(key);

    VoxelConst voxel = map.voxel(key);
    ASSERT_TRUE(voxel.isValid());
    EXPECT_TRUE(voxel.isOccupied());

    const float voxel_value = voxel.value();
    EXPECT_EQ(voxel_value, map.hitValue());
  }


  TEST(Map, Miss)
  {
    OccupancyMap map(0.25);
    Key key(0, 0, 0, 0, 0, 0);
    map.integrateMiss(key);

    VoxelConst voxel = map.voxel(key);
    ASSERT_TRUE(voxel.isValid());
    EXPECT_TRUE(voxel.isFree());

    const float voxel_value = voxel.value();
    EXPECT_EQ(voxel_value, map.missValue());
  }


  TEST(Map, Clone)
  {
    OccupancyMap map(0.25);

    // Generate occupancy.
    ohmgen::cubicRoom(map, 5.0f);

    // Generate clearance values.
    ClearanceProcess ref_clearance(2.0f, kQfGpuEvaluate);
    ref_clearance.update(map, 0.0);

    // Clone the map
    const std::unique_ptr<OccupancyMap> map_copy(map.clone());

    // Compare results.
    MapCache cache;

    // Compare maps.
    ohmtestutil::compareMaps(*map_copy, map, ohmtestutil::kCfCompareAll | ohmtestutil::kCfExpectClearance);
  }


  TEST(Map, CloneSubmap)
  {
    const glm::dvec3 clone_min(0);
    const glm::dvec3 clone_max(4.5);
    OccupancyMap map(0.2);

    // Generate occupancy.
    ohmgen::cubicRoom(map, 5.0f);

    // Generate clearance values.
    ClearanceProcess ref_clearance(2.0f, kQfGpuEvaluate);
    ref_clearance.update(map, 0.0);

    // Clone the map
    const std::unique_ptr<OccupancyMap> map_copy(map.clone(clone_min, clone_max));

    // Compare results.
    MapCache cache;

    // Compare maps.
    ohmtestutil::compareMaps(*map_copy, map, clone_min, clone_max,
                             ohmtestutil::kCfCompareAll | ohmtestutil::kCfExpectClearance);
  }
}
