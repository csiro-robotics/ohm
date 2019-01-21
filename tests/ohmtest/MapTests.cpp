// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/Aabb.h>
#include <ohm/ClearanceProcess.h>
#include <ohm/Key.h>
#include <ohm/LineQuery.h>
#include <ohm/MapCache.h>
#include <ohm/OccupancyMap.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include "OhmTestUtil.h"

using namespace ohm;

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
    const double box_size = 5.0;
    ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

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
    const double box_size = 5.0;
    ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

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


  TEST(Map, ClipBox)
  {
    // Test clipping of rays to an Aabb on insert.
    const double resolution = 0.2;
    const uint8_t region_size = 32u;
    OccupancyMap map(resolution, glm::u8vec3(region_size));

    Aabb clip_box(glm::dvec3(-1.0), glm::dvec3(2.0));
    std::vector<glm::dvec3> rays;

    // Start with rays which pass through the box.
    rays.push_back(glm::dvec3(-2, 0, 0));
    rays.push_back(glm::dvec3(3, 0, 0));

    rays.push_back(glm::dvec3(0, -2, 0));
    rays.push_back(glm::dvec3(0, 3, 0));

    rays.push_back(glm::dvec3(0, 0, 3));
    rays.push_back(glm::dvec3(0, 0, -2));

    map.setRayFilter([&clip_box](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) {
      return clipBounded(start, end, filter_flags, clip_box);
    });

    map.integrateRays(rays.data(), rays.size());

    // Validate the map contains no occupied points; only free and unknown.
    const glm::dvec3 voxel_half_extents(0.5 * map.resolution());
    bool touched = false;
    for (auto iter = map.begin(); iter != map.end(); ++iter)
    {
      Voxel &voxel = *iter;
      touched = true;

      if (voxel.isValid())
      {
        if (!voxel.isUncertain())
        {
          EXPECT_LT(voxel.value(), map.occupancyThresholdValue());
          EXPECT_FALSE(voxel.isOccupied());

          // Voxel should also be with in the bounds of the Aabb. Check this.
          const Aabb voxel_box(voxel.centreGlobal() - voxel_half_extents, voxel.centreGlobal() + voxel_half_extents);
          EXPECT_TRUE(clip_box.overlaps(voxel_box)) << "Voxel box does not overlap extents";
        }
      }
    }

    EXPECT_TRUE(touched);

    // Reset the map. This also tests that resetting a GPU map works.
    map.clear();

    rays.clear();

    // Now rays which enter the box, ending at the origin.
    // Start with rays which pass through the box.
    rays.push_back(glm::dvec3(-2, 0, 0));
    rays.push_back(glm::dvec3(0, 0, 0));

    rays.push_back(glm::dvec3(0, -2, 0));
    rays.push_back(glm::dvec3(0, 0, 0));

    rays.push_back(glm::dvec3(0, 0, 3));
    rays.push_back(glm::dvec3(0, 0, 0));

    map.integrateRays(rays.data(), rays.size());

    // Validate the map contains no occupied points; only free and unknown.
    const Key target_key = map.voxelKey(glm::dvec3(0));
    touched = false;
    for (auto iter = map.begin(); iter != map.end(); ++iter)
    {
      Voxel &voxel = *iter;
      touched = true;

      if (voxel.isValid())
      {
        if (voxel.key() != target_key)
        {
          if (!voxel.isUncertain())
          {
            EXPECT_LT(voxel.value(), map.occupancyThresholdValue());
          }
          EXPECT_FALSE(voxel.isOccupied());
        }
        else
        {
          EXPECT_GE(voxel.value(), map.occupancyThresholdValue());
          EXPECT_TRUE(voxel.isOccupied());
        }

        // Touched voxels should also be with in the bounds of the Aabb. Check this.
        if (!voxel.isUncertain())
        {
          const Aabb voxel_box(voxel.centreGlobal() - voxel_half_extents, voxel.centreGlobal() + voxel_half_extents);
          EXPECT_TRUE(clip_box.overlaps(voxel_box)) << "Voxel box does not overlap extents";
        }
      }
    }

    EXPECT_TRUE(touched);
  }
}  // namespace maptests
