// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <ohm/Aabb.h>
#include <ohm/Key.h>
#include <ohm/LineQuery.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/VoxelData.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include "ohmtestcommon/OhmTestUtil.h"

using namespace ohm;

namespace maptests
{
TEST(Map, Hit)
{
  OccupancyMap map(0.25);
  Key key(0, 0, 0, 0, 0, 0);

  {
    Voxel<float> voxel_write(&map, map.layout().occupancyLayer());
    voxel_write.setKey(key);
    ASSERT_TRUE(voxel_write.isValid());
    integrateHit(voxel_write);
  }

  {
    Voxel<const float> voxel_read(&map, map.layout().occupancyLayer(), key);
    ASSERT_TRUE(voxel_read.isValid());
    EXPECT_TRUE(isOccupied(voxel_read));

    float voxel_value;
    voxel_read.read(&voxel_value);
    EXPECT_EQ(voxel_value, map.hitValue());
  }
}


TEST(Map, Miss)
{
  OccupancyMap map(0.25);
  Key key(0, 0, 0, 0, 0, 0);

  {
    Voxel<float> voxel_write(&map, map.layout().occupancyLayer());
    voxel_write.setKey(key);
    ASSERT_TRUE(voxel_write.isValid());
    integrateMiss(voxel_write);
  }

  {
    Voxel<const float> voxel_read(&map, map.layout().occupancyLayer(), key);
    ASSERT_TRUE(voxel_read.isValid());
    EXPECT_TRUE(isFree(voxel_read));

    float voxel_value;
    voxel_read.read(&voxel_value);
    EXPECT_EQ(voxel_value, map.missValue());
  }
}


TEST(Map, Clone)
{
  OccupancyMap map(0.25);

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Clone the map
  const std::unique_ptr<OccupancyMap> map_copy(map.clone());

  // Compare maps.
  ohmtestutil::compareMaps(*map_copy, map, ohmtestutil::kCfCompareAll);
}


TEST(Map, CloneSubmap)
{
  const glm::dvec3 clone_min(0);
  const glm::dvec3 clone_max(4.5);
  OccupancyMap map(0.2);

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Clone the map
  const std::unique_ptr<OccupancyMap> map_copy(map.clone(clone_min, clone_max));

  // Compare maps.
  ohmtestutil::compareMaps(*map_copy, map, clone_min, clone_max, ohmtestutil::kCfCompareAll);
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

  RayMapperOccupancy(&map).integrateRays(rays.data(), rays.size());

  // Validate the map contains no occupied points; only free and unknown.
  const glm::dvec3 voxel_half_extents(0.5 * map.resolution());
  bool touched = false;
  {
    Voxel<const float> voxel(&map, map.layout().occupancyLayer());
    for (auto iter = map.begin(); iter != map.end(); ++iter)
    {
      voxel.setKey(iter);
      touched = true;

      if (voxel.isValid())
      {
        float voxel_occupancy;
        voxel.read(&voxel_occupancy);
        if (!isUnobserved(voxel_occupancy))
        {
          EXPECT_LT(voxel_occupancy, map.occupancyThresholdValue());
          EXPECT_FALSE(isOccupied(voxel));

          // Voxel should also be with in the bounds of the Aabb. Check this.
          const Aabb voxel_box(map.voxelCentreGlobal(*iter) - voxel_half_extents,
                               map.voxelCentreGlobal(*iter) + voxel_half_extents);
          EXPECT_TRUE(clip_box.overlaps(voxel_box)) << "Voxel box does not overlap extents";
        }
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

  RayMapperOccupancy(&map).integrateRays(rays.data(), rays.size());

  // Validate the map contains no occupied points; only free and unknown.
  const Key target_key = map.voxelKey(glm::dvec3(0));
  float voxel_occupancy;
  touched = false;
  {
    Voxel<const float> voxel(&map, map.layout().occupancyLayer());
    for (auto iter = map.begin(); iter != map.end(); ++iter)
    {
      voxel.setKey(iter);
      touched = true;

      if (voxel.isValid())
      {
        voxel.read(&voxel_occupancy);
        if (voxel.key() != target_key)
        {
          if (!isUnobserved(voxel))
          {
            EXPECT_LT(voxel_occupancy, map.occupancyThresholdValue());
          }
          EXPECT_FALSE(isOccupied(voxel));
        }
        else
        {
          EXPECT_GE(voxel_occupancy, map.occupancyThresholdValue());
          EXPECT_TRUE(isOccupied(voxel));
        }

        // Touched voxels should also be with in the bounds of the Aabb. Check this.
        if (!isUnobserved(voxel))
        {
          const Aabb voxel_box(map.voxelCentreGlobal(*iter) - voxel_half_extents,
                               map.voxelCentreGlobal(*iter) + voxel_half_extents);
          EXPECT_TRUE(clip_box.overlaps(voxel_box)) << "Voxel box does not overlap extents";
        }
      }
    }
  }

  EXPECT_TRUE(touched);
}
}  // namespace maptests
