// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <ohm/Aabb.h>
#include <ohm/CopyUtil.h>
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
TEST(Copy, Clone)
{
  OccupancyMap map(0.25);

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Clone the map
  const std::unique_ptr<OccupancyMap> map_copy(map.clone());

  // Compare maps.
  ohmtestutil::compareMaps(*map_copy, map, ohmtestutil::kCfCompareExtended);
}


TEST(Copy, CloneSubmap)
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
  ohmtestutil::compareMaps(*map_copy, map, clone_min, clone_max, ohmtestutil::kCfCompareExtended);
}


TEST(Copy, Copy)
{
  // Test copying utilities.
  OccupancyMap map(0.2);
  OccupancyMap dst_map(map.resolution());

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Validate we can't copy into a map of differing resolution...
  EXPECT_FALSE(ohm::canCopy(ohm::OccupancyMap(map.resolution() * 2), map));
  // or of different region sizes...
  EXPECT_FALSE(ohm::canCopy(ohm::OccupancyMap(map.resolution(), glm::u8vec3(1, 1, 1)), map));
  // or at different origins
  dst_map.setOrigin(glm::dvec3(1, 2, 3));
  EXPECT_FALSE(ohm::canCopy(dst_map, map));

  // Rest dst_map origin for copy test.
  dst_map.setOrigin(map.origin());
  EXPECT_TRUE(ohm::canCopy(dst_map, map));

  // Copy and compare with no filter.
  EXPECT_TRUE(ohm::copyMap(dst_map, map));

  // Compare maps.
  ohmtestutil::compareMaps(dst_map, map, ohmtestutil::kCfCompareExtended);
}


TEST(Copy, CopySubmap)
{
  // Test copying utilities.
  const glm::dvec3 copy_min(0);
  const glm::dvec3 copy_max(4.5);
  OccupancyMap map(0.2);
  OccupancyMap dst_map(map.resolution());

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Copy and compare with no filter.
  EXPECT_TRUE(ohm::copyMap(dst_map, map, ohm::copyFilterExtents(copy_min, copy_max)));
  // Compare maps.
  ohmtestutil::compareMaps(dst_map, map, copy_min, copy_max, ohmtestutil::kCfCompareExtended);
}

TEST(Copy, CopyUpdated)
{
  // Test copying only touched voxels.
  // Test copying utilities.
  const glm::dvec3 copy_min(0);
  const glm::dvec3 copy_max(4.5);
  OccupancyMap map(0.2);
  OccupancyMap dst_map(map.resolution());

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Get the map stamp now.
  auto stamp = map.stamp();

  // Invoke the copy. Expect nothing to be copied.
  EXPECT_TRUE(ohm::copyMap(dst_map, map, ohm::copyFilterStamp(stamp)));
  EXPECT_EQ(dst_map.regionCount(), 0);

  // Make some updates to a single region.
  Key key(0, 0, 0, 0, 0, 0);
  Voxel<float> occupancy(&map, map.layout().occupancyLayer());
  ASSERT_TRUE(occupancy.isLayerValid());

  for (uint8_t z = 0; z < map.regionVoxelDimensions().z; ++z)
  {
    for (uint8_t y = 0; y < map.regionVoxelDimensions().y; ++y)
    {
      for (uint8_t x = 0; x < map.regionVoxelDimensions().x; ++x)
      {
        key.setLocalKey(glm::u8vec3(x, y, z));
        occupancy.setKey(key);
        ASSERT_TRUE(occupancy.isValid());
        ohm::integrateHit(occupancy);
      }
    }
  }

  // To finalise the map stamping changes, we need to change regions or invalid the Voxel object.
  occupancy.commit();

  // Copy only updated regions.
  EXPECT_TRUE(ohm::copyMap(dst_map, map, ohm::copyFilterStamp(stamp)));

  // Validate the target map only has the region we've updated
  EXPECT_EQ(dst_map.regionCount(), 1);
  EXPECT_NE(dst_map.region(key.regionKey(), false), nullptr);

  // Validate the region matches.
  // Lock the occupancy layers.
  Voxel<const float> src_occupancy(&map, map.layout().occupancyLayer(), key);
  ASSERT_TRUE(src_occupancy.isValid());
  Voxel<const float> dst_occupancy(&dst_map, dst_map.layout().occupancyLayer(), key);
  ASSERT_TRUE(dst_occupancy.isValid());

  // Compare voxels in the region.
  for (uint8_t z = 0; z < map.regionVoxelDimensions().z; ++z)
  {
    for (uint8_t y = 0; y < map.regionVoxelDimensions().y; ++y)
    {
      for (uint8_t x = 0; x < map.regionVoxelDimensions().x; ++x)
      {
        key.setLocalKey(glm::u8vec3(x, y, z));
        src_occupancy.setKey(key);
        dst_occupancy.setKey(key);
        ASSERT_TRUE(src_occupancy.isValid());
        ASSERT_TRUE(dst_occupancy.isValid());
        EXPECT_EQ(dst_occupancy.data(), src_occupancy.data());
      }
    }
  }
}
}  // namespace maptests
