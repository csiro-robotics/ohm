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

#include <logutil/LogUtil.h>

#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "ohmtestcommon/OhmTestUtil.h"

namespace maptests
{
TEST(Copy, Clone)
{
  ohm::OccupancyMap map(0.25);

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Clone the map
  const std::unique_ptr<ohm::OccupancyMap> map_copy(map.clone());

  // Compare maps.
  ohmtestutil::compareMaps(*map_copy, map, ohmtestutil::kCfCompareExtended);
}


TEST(Copy, CloneSubmap)
{
  const glm::dvec3 clone_min(0);
  const glm::dvec3 clone_max(4.5);
  ohm::OccupancyMap map(0.2);

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Clone the map
  const std::unique_ptr<ohm::OccupancyMap> map_copy(map.clone(clone_min, clone_max));

  // Compare maps.
  ohmtestutil::compareMaps(*map_copy, map, clone_min, clone_max, ohmtestutil::kCfCompareExtended);
}


TEST(Copy, Copy)
{
  // Test copying utilities.
  ohm::OccupancyMap map(0.2);
  ohm::OccupancyMap dst_map(map.resolution());

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
  ohm::OccupancyMap map(0.2);
  ohm::OccupancyMap dst_map(map.resolution());

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
  ohm::OccupancyMap map(0.2);
  ohm::OccupancyMap dst_map(map.resolution());

  // Generate occupancy.
  const double box_size = 5.0;
  ohmgen::boxRoom(map, glm::dvec3(-box_size), glm::dvec3(box_size));

  // Get the map stamp now.
  auto stamp = map.stamp();

  // Invoke the copy. Expect nothing to be copied.
  EXPECT_TRUE(ohm::copyMap(dst_map, map, ohm::copyFilterStamp(stamp)));
  EXPECT_EQ(dst_map.regionCount(), 0);

  // Make some updates to a single region.
  ohm::Key key(0, 0, 0, 0, 0, 0);
  ohm::Voxel<float> occupancy(&map, map.layout().occupancyLayer());
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
  ohm::Voxel<const float> src_occupancy(&map, map.layout().occupancyLayer(), key);
  ASSERT_TRUE(src_occupancy.isValid());
  ohm::Voxel<const float> dst_occupancy(&dst_map, dst_map.layout().occupancyLayer(), key);
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

TEST(Copy, LayerFilter)
{
  // Test copying only selected layers.
  ohm::MapFlag map_flags = ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal | ohm::MapFlag::kTouchTime;
  ohm::OccupancyMap src_map(0.1, map_flags);

  ohm::Voxel<float> src_occupancy(&src_map, src_map.layout().occupancyLayer());
  ohm::Voxel<ohm::VoxelMean> src_mean(&src_map, src_map.layout().meanLayer());
  ohm::Voxel<float> src_traversal(&src_map, src_map.layout().traversalLayer());
  ohm::Voxel<float> src_touch_time(&src_map, src_map.layout().layerIndex(ohm::default_layer::touchTimeLayerName()));

  ASSERT_TRUE(src_occupancy.isLayerValid());
  ASSERT_TRUE(src_mean.isLayerValid());
  ASSERT_TRUE(src_traversal.isLayerValid());
  ASSERT_TRUE(src_touch_time.isLayerValid());

  const float occupancy_value = -1.0f;
  const ohm::VoxelMean mean_value = { 1, 2 };
  const float traversal_value = 1.0f;
  const uint32_t touch_time_value = 42u;

  // Populate a single chunk with known values.
  ohm::Key key(0, 0, 0, 0, 0, 0);
  for (int z = 0; z < src_map.regionVoxelDimensions().z; ++z)
  {
    key.setLocalAxis(2, z);
    for (int y = 0; y < src_map.regionVoxelDimensions().y; ++y)
    {
      key.setLocalAxis(1, y);
      for (int x = 0; x < src_map.regionVoxelDimensions().x; ++x)
      {
        key.setLocalAxis(0, x);
        ohm::setVoxelKey(key, src_occupancy, src_mean, src_traversal, src_touch_time);
        ASSERT_TRUE(src_occupancy.isValid());
        ASSERT_TRUE(src_mean.isValid());
        ASSERT_TRUE(src_traversal.isValid());
        ASSERT_TRUE(src_touch_time.isValid());

        src_occupancy.write(occupancy_value);
        src_mean.write(mean_value);
        src_traversal.write(traversal_value);
        src_touch_time.write(touch_time_value);
      }
    }
  }
  src_occupancy.commit();
  src_mean.commit();
  src_traversal.commit();
  src_touch_time.commit();

  // Create a map to copy into omitting the TouchTime layer.
  map_flags &= ~ohm::MapFlag::kTouchTime;
  ohm::OccupancyMap dst_map(src_map.resolution(), map_flags);

  // Copy all layers except occupancy.
  std::vector<std::string> copy_layers = { ohm::default_layer::meanLayerName(),
                                           ohm::default_layer::traversalLayerName(),
                                           ohm::default_layer::touchTimeLayerName() };

  ohm::copyMap(dst_map, src_map, {}, ohm::copyLayersFilter(copy_layers));

  // Validate all voxels match except occupancy values.
  ohm::Voxel<const float> dst_occupancy(&dst_map, dst_map.layout().occupancyLayer());
  ohm::Voxel<const ohm::VoxelMean> dst_mean(&dst_map, dst_map.layout().meanLayer());
  ohm::Voxel<const float> dst_traversal(&dst_map, dst_map.layout().traversalLayer());
  // Not using touch time in the dst_map.
  ASSERT_EQ(dst_map.layout().layerIndex(ohm::default_layer::touchTimeLayerName()), -1);

  ASSERT_TRUE(dst_occupancy.isLayerValid());
  ASSERT_TRUE(dst_mean.isLayerValid());
  ASSERT_TRUE(dst_traversal.isLayerValid());

  // Now compare values.
  key = ohm::Key(0, 0, 0, 0, 0, 0);
  for (int z = 0; z < dst_map.regionVoxelDimensions().z; ++z)
  {
    key.setLocalAxis(2, z);
    for (int y = 0; y < dst_map.regionVoxelDimensions().y; ++y)
    {
      key.setLocalAxis(1, y);
      for (int x = 0; x < dst_map.regionVoxelDimensions().x; ++x)
      {
        key.setLocalAxis(0, x);
        ohm::setVoxelKey(key, dst_occupancy, dst_mean, dst_traversal);
        ASSERT_TRUE(dst_occupancy.isValid());
        ASSERT_TRUE(dst_mean.isValid());
        ASSERT_TRUE(dst_traversal.isValid());

        ASSERT_NE(dst_occupancy.data(), occupancy_value);
        ASSERT_EQ(dst_occupancy.data(), ohm::unobservedOccupancyValue());
        ASSERT_EQ(dst_mean.data().coord, mean_value.coord);
        ASSERT_EQ(dst_mean.data().count, mean_value.count);
        ASSERT_EQ(dst_traversal.data(), traversal_value);
      }
    }
  }
  dst_occupancy.reset();
  dst_mean.reset();
  dst_traversal.reset();

  // Clear the map and copy again using the negated filter.
  dst_map.clear();
  ohm::copyMap(dst_map, src_map, {}, ohm::copyFilterNegate(ohm::copyLayersFilter(copy_layers)));

  dst_occupancy = ohm::Voxel<const float>(&dst_map, dst_map.layout().occupancyLayer());
  dst_mean = ohm::Voxel<const ohm::VoxelMean>(&dst_map, dst_map.layout().meanLayer());
  dst_traversal = ohm::Voxel<const float>(&dst_map, dst_map.layout().traversalLayer());
  ASSERT_TRUE(dst_occupancy.isLayerValid());
  ASSERT_TRUE(dst_mean.isLayerValid());
  ASSERT_TRUE(dst_traversal.isLayerValid());

  // Compare again.
  key = ohm::Key(0, 0, 0, 0, 0, 0);
  for (int z = 0; z < dst_map.regionVoxelDimensions().z; ++z)
  {
    key.setLocalAxis(2, z);
    for (int y = 0; y < dst_map.regionVoxelDimensions().y; ++y)
    {
      key.setLocalAxis(1, y);
      for (int x = 0; x < dst_map.regionVoxelDimensions().x; ++x)
      {
        key.setLocalAxis(0, x);
        ohm::setVoxelKey(key, dst_occupancy, dst_mean, dst_traversal);
        ASSERT_TRUE(dst_occupancy.isValid());
        ASSERT_TRUE(dst_mean.isValid());
        ASSERT_TRUE(dst_traversal.isValid());

        ASSERT_EQ(dst_occupancy.data(), occupancy_value);
        ASSERT_NE(dst_mean.data().coord, mean_value.coord);
        ASSERT_NE(dst_mean.data().count, mean_value.count);
        ASSERT_EQ(dst_mean.data().coord, 0);
        ASSERT_EQ(dst_mean.data().count, 0);
        ASSERT_NE(dst_traversal.data(), traversal_value);
        ASSERT_EQ(dst_traversal.data(), 0);
      }
    }
  }
}
}  // namespace maptests
