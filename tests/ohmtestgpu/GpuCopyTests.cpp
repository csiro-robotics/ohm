// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/Aabb.h>
#include <ohm/CopyUtil.h>
#include <ohm/Key.h>
#include <ohm/LineQuery.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/VoxelData.h>

#include <ohmgpu/GpuMap.h>

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

using namespace ohm;

namespace maptests
{
TEST(Copy, CopyFromGpu)
{
  // Test copying from GPU memory.
  const double map_extents = 10.0;
  const double noise = 1.0;
  const unsigned ray_count = 1024 * 128;
  OccupancyMap map(0.2);
  OccupancyMap dst_map(map.resolution());
  {
    GpuMap gpu_map(&map, true);

    // Make some rays.
    std::mt19937 rand_engine;
    std::uniform_real_distribution<double> unit_rand(-1, 1);
    std::uniform_real_distribution<double> length_rand(map_extents - noise, map_extents);
    std::vector<glm::dvec3> rays;

    // Build rays to create a generally spherical shell.
    while (rays.size() < ray_count * 2)
    {
      rays.push_back(glm::dvec3(0.05));
      glm::dvec3 ray_end(unit_rand(rand_engine), unit_rand(rand_engine), unit_rand(rand_engine));
      ray_end = glm::normalize(ray_end);
      ray_end *= length_rand(rand_engine);
      rays.push_back(ray_end);
    }

    gpu_map.integrateRays(rays.data(), rays.size());

    // Do not sync back to CPU. Copy directly from GPU.
    EXPECT_TRUE(copyMap(dst_map, map));
    // Compare one region ensuring it *does not* match.
    const MapChunk *src_region = map.region(map.voxelKey(rays[1]).regionKey(), false);
    ASSERT_NE(src_region, nullptr);

    // Find the corresponding chunk in the copied map.
    const MapChunk *dst_region = dst_map.region(src_region->region.coord, false);
    ASSERT_NE(dst_region, nullptr);

    // Lock the occupancy layers.
    Voxel<const float> src_occupancy(&map, map.layout().occupancyLayer(), Key(src_region->region.coord, 0, 0, 0));
    ASSERT_TRUE(src_occupancy.isValid());
    Voxel<const float> dst_occupancy(&dst_map, dst_map.layout().occupancyLayer(), src_occupancy.key());
    ASSERT_TRUE(dst_occupancy.isValid());

    // Compare voxels in the region.
    size_t observed_voxel_count = 0;
    for (uint8_t z = 0; z < map.regionVoxelDimensions().z; ++z)
    {
      for (uint8_t y = 0; y < map.regionVoxelDimensions().y; ++y)
      {
        for (uint8_t x = 0; x < map.regionVoxelDimensions().x; ++x)
        {
          src_occupancy.setKey(Key(src_region->region.coord, x, y, z));
          dst_occupancy.setKey(src_occupancy.key());
          ASSERT_TRUE(src_occupancy.isValid());
          ASSERT_TRUE(dst_occupancy.isValid());

          // Observed voxels in dst_map should not match those in map.
          if (dst_occupancy.data() != unobservedOccupancyValue())
          {
            EXPECT_NE(dst_occupancy.data(), src_occupancy.data());
            ++observed_voxel_count;
          }
        }
      }
    }

    // We expect to have observed something. Otherwise we'd pass a failure to copy at all.
    EXPECT_GT(observed_voxel_count, 0);
    std::cout << "Compared " << observed_voxel_count << " voxels" << std::endl;

    // Now sync the GPU map back to CPU and compare the whole map - they should match.
    gpu_map.syncVoxels();
    ohmtestutil::compareMaps(dst_map, map, ohmtestutil::kCfCompareExtended);
  }
}

TEST(Copy, CopySubmapFromGpu)
{
  // Test copying from GPU memory.
  const double map_extents = 10.0;
  const double noise = 1.0;
  const unsigned ray_count = 1024 * 128;
  const glm::dvec3 copy_min(0);
  const glm::dvec3 copy_max(4.5);
  OccupancyMap map(0.2);
  OccupancyMap dst_map(map.resolution());
  {
    GpuMap gpu_map(&map, true);

    // Make some rays.
    std::mt19937 rand_engine;
    std::uniform_real_distribution<double> unit_rand(-1, 1);
    std::uniform_real_distribution<double> length_rand(map_extents - noise, map_extents);
    std::vector<glm::dvec3> rays;

    // Build rays to create a generally spherical shell.
    while (rays.size() < ray_count * 2)
    {
      rays.push_back(glm::dvec3(0.05));
      glm::dvec3 ray_end(unit_rand(rand_engine), unit_rand(rand_engine), unit_rand(rand_engine));
      ray_end = glm::normalize(ray_end);
      ray_end *= length_rand(rand_engine);
      rays.push_back(ray_end);
    }

    gpu_map.integrateRays(rays.data(), rays.size());

    // Do not sync back to CPU. Copy directly from GPU.
    EXPECT_TRUE(copyMap(dst_map, map, copyFilterExtents(copy_min, copy_max)));

    // Now sync the GPU map back to CPU and compare the whole map - they should match.
    gpu_map.syncVoxels();
    ohmtestutil::compareMaps(dst_map, map, copy_min, copy_max, ohmtestutil::kCfCompareExtended);
  }
}

TEST(Copy, CopyUpdated)
{
  // Test copying from GPU memory.
  const double map_extents = 10.0;
  const double noise = 1.0;
  const unsigned ray_count = 1024 * 128;
  const glm::dvec3 copy_min(0);
  const glm::dvec3 copy_max(4.5);
  OccupancyMap map(0.2);
  OccupancyMap dst_map(map.resolution());
  {
    GpuMap gpu_map(&map, true);

    // Make some rays.
    std::mt19937 rand_engine;
    std::uniform_real_distribution<double> unit_rand(-1, 1);
    std::uniform_real_distribution<double> length_rand(map_extents - noise, map_extents);
    std::vector<glm::dvec3> rays;

    // Build rays to create a generally spherical shell.
    while (rays.size() < ray_count * 2)
    {
      rays.push_back(glm::dvec3(0.05));
      glm::dvec3 ray_end(unit_rand(rand_engine), unit_rand(rand_engine), unit_rand(rand_engine));
      ray_end = glm::normalize(ray_end);
      ray_end *= length_rand(rand_engine);
      rays.push_back(ray_end);
    }

    gpu_map.integrateRays(rays.data(), rays.size());

    // Get the map stamp now.
    auto stamp = map.stamp();

    // Invoke the copy. Expect nothing to be copied.
    EXPECT_TRUE(ohm::copyMap(dst_map, map, ohm::copyFilterStamp(stamp)));
    EXPECT_EQ(dst_map.regionCount(), 0);

    // Make some updates to a single region.
    Key key(0, 0, 0, 0, 0, 0);

    rays.clear();

    for (uint8_t z = 0; z < map.regionVoxelDimensions().z; ++z)
    {
      for (uint8_t y = 0; y < map.regionVoxelDimensions().y; ++y)
      {
        for (uint8_t x = 0; x < map.regionVoxelDimensions().x; ++x)
        {
          key.setLocalKey(glm::u8vec3(x, y, z));
          // Make a ray which goes nowhere, but update a single voxel.
          rays.emplace_back(map.voxelCentreGlobal(key));
          rays.emplace_back(map.voxelCentreGlobal(key));
        }
      }
    }

    // Integrate the blatting of the region.
    gpu_map.integrateRays(rays.data(), rays.size());

    // Copy only updated regions.
    EXPECT_TRUE(ohm::copyMap(dst_map, map, ohm::copyFilterStamp(stamp)));

    // Validate the target map only has the region we've updated
    EXPECT_EQ(dst_map.regionCount(), 1);
    EXPECT_NE(dst_map.region(key.regionKey(), false), nullptr);

    // Sync to CPU for comparison.
    gpu_map.syncVoxels();
    std::cout << "Stamp @ 3: " << map.stamp() << std::endl;

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
}
}  // namespace maptests
