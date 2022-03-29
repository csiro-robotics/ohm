// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/QueryFlag.h>
#include <ohm/RayMapperTsdf.h>
#include <ohm/VoxelData.h>

#include <ohmgpu/GpuTsdfMap.h>
#include <ohmgpu/LineKeysQueryGpu.h>

namespace tsdf
{
TEST(Tsdf, Basic)
{
  // This is a very rudimentary test.
  // See ohmtest Tsdf.Basic
  const double resolution = 0.1;
  const glm::u8vec3 region_size(32);

  // Originally was using 1.0 for each axis. However, this yielded a discrepancy in the key calculation between
  // the different OpenCL programs used, with voxel steps occuring in a different order - floating point error as far
  // as I can tell. By biasing, we ensure the next step select is not choosing between the same magnitudes on different
  // axes.
  const glm::dvec3 bias(1.0, 0.9, 0.8);
  const std::vector<glm::dvec3> rays = {
    glm::dvec3(0.0), glm::dvec3(bias.x, 0.0, 0.0),        glm::dvec3(0.0), glm::dvec3(-bias.x, 0.0, 0.0),
    glm::dvec3(0.0), glm::dvec3(bias.x, bias.y, 0.0),     glm::dvec3(0.0), glm::dvec3(-bias.x, bias.y, 0.0),
    glm::dvec3(0.0), glm::dvec3(0.0, bias.y, 0.0),        glm::dvec3(0.0), glm::dvec3(bias.x, -bias.y, 0.0),
    glm::dvec3(0.0), glm::dvec3(-bias.x, 0.0, bias.z),    glm::dvec3(0.0), glm::dvec3(bias.x, bias.y, bias.z),
    glm::dvec3(0.0), glm::dvec3(-bias.x, bias.y, bias.z), glm::dvec3(0.0), glm::dvec3(bias.x, 0.0, bias.z),
    glm::dvec3(0.0), glm::dvec3(bias.x, -bias.y, bias.z), glm::dvec3(0.0), glm::dvec3(-bias.x, 0.0, bias.z),
    glm::dvec3(0.0), glm::dvec3(bias.x, bias.y, -bias.z), glm::dvec3(0.0), glm::dvec3(-bias.x, bias.y, -bias.z),
    glm::dvec3(0.0), glm::dvec3(bias.x, 0.0, -bias.z),    glm::dvec3(0.0), glm::dvec3(bias.x, -bias.y, -bias.z),
  };

  // Test core voxel mean positioning
  ohm::OccupancyMap map(resolution, region_size, ohm::MapFlag::kTsdf);
  ohm::GpuTsdfMap tsdf_mapper(&map);

  // Offset the map origin to trace between voxel centres.
  map.setOrigin(glm::dvec3(-0.5 * map.resolution()));
  // Set large truncation distance so our values matche computeDistance() results.
  tsdf_mapper.setDefaultTruncationDistance(10.0);

  // Use the GPU ray calculator in order to touch the same voxels. We can get different results using
  // ohm::MapLineWalker due to floating point precision differences.
  ohm::LineKeysQueryGpu query(map, ohm::kQfGpuEvaluate);
  query.setRays(rays.data(), rays.size());
  query.execute();

  ASSERT_EQ(query.numberOfResults(), rays.size() / 2u);

  for (size_t i = 0; i < rays.size(); i += 2)
  {
    // Clear previous results.
    map.clear();
    // Must initialise the voxel reference after the map clear or it may become invalid.
    ohm::Voxel<const ohm::VoxelTsdf> tsdf(&map, map.layout().layerIndex(ohm::default_layer::tsdfLayerName()));
    ASSERT_TRUE(tsdf.isLayerValid()) << "ray index " << i / 2u;
    // Trace ray.
    tsdf_mapper.integrateRays(&rays[i], 2, nullptr, nullptr, 0);
    tsdf_mapper.syncVoxels();

    const size_t index_offset = query.resultIndices()[i / 2u];
    const size_t key_count = query.resultCounts()[i / 2u];

    for (size_t k = 0; k < key_count; ++k)
    {
      const ohm::Key key = query.intersectedVoxels()[index_offset + k];
      ohm::setVoxelKey(key, tsdf);
      ASSERT_TRUE(tsdf.isValid()) << "ray index " << i / 2u;
      const ohm::VoxelTsdf tsdf_data = tsdf.data();
      EXPECT_NEAR(tsdf_data.distance, ohm::computeDistance(rays[i], rays[i + 1], map.voxelCentreGlobal(key)), 1e-6)
        << "ray index " << i / 2u;
    }
  }
}

TEST(Tsdf, Truncation)
{
  // This is a very rudimentary test.
  // See ohmtest Tsdf.Basic
  const double resolution = 0.1;
  const glm::u8vec3 region_size(32);

  // Originally was using 1.0 for each axis. However, this yielded a discrepancy in the key calculation between
  // the different OpenCL programs used, with voxel steps occuring in a different order - floating point error as far
  // as I can tell. By biasing, we ensure the next step select is not choosing between the same magnitudes on different
  // axes.
  const glm::dvec3 bias(1.0, 0.9, 0.8);
  const std::vector<glm::dvec3> rays = {
    glm::dvec3(0.0), glm::dvec3(bias.x, 0.0, 0.0),        glm::dvec3(0.0), glm::dvec3(-bias.x, 0.0, 0.0),
    glm::dvec3(0.0), glm::dvec3(bias.x, bias.y, 0.0),     glm::dvec3(0.0), glm::dvec3(-bias.x, bias.y, 0.0),
    glm::dvec3(0.0), glm::dvec3(0.0, bias.y, 0.0),        glm::dvec3(0.0), glm::dvec3(bias.x, -bias.y, 0.0),
    glm::dvec3(0.0), glm::dvec3(-bias.x, 0.0, bias.z),    glm::dvec3(0.0), glm::dvec3(bias.x, bias.y, bias.z),
    glm::dvec3(0.0), glm::dvec3(-bias.x, bias.y, bias.z), glm::dvec3(0.0), glm::dvec3(bias.x, 0.0, bias.z),
    glm::dvec3(0.0), glm::dvec3(bias.x, -bias.y, bias.z), glm::dvec3(0.0), glm::dvec3(-bias.x, 0.0, bias.z),
    glm::dvec3(0.0), glm::dvec3(bias.x, bias.y, -bias.z), glm::dvec3(0.0), glm::dvec3(-bias.x, bias.y, -bias.z),
    glm::dvec3(0.0), glm::dvec3(bias.x, 0.0, -bias.z),    glm::dvec3(0.0), glm::dvec3(bias.x, -bias.y, -bias.z),
  };

  // Test core voxel mean positioning
  ohm::OccupancyMap map(resolution, region_size, ohm::MapFlag::kTsdf);
  ohm::GpuTsdfMap tsdf_mapper(&map);

  // Offset the map origin to trace between voxel centres.
  map.setOrigin(glm::dvec3(-0.5 * map.resolution()));
  // Set a very small truncation distance. All voxels except the same voxel should be truncated to this value.
  tsdf_mapper.setDefaultTruncationDistance(0.01f * float(map.resolution()));

  // Use the GPU ray calculator in order to touch the same voxels. We can get different results using
  // ohm::MapLineWalker due to floating point precision differences.
  ohm::LineKeysQueryGpu query(map, ohm::kQfGpuEvaluate);
  query.setRays(rays.data(), rays.size());
  query.execute();

  ASSERT_EQ(query.numberOfResults(), rays.size() / 2u);

  for (size_t i = 0; i < rays.size(); i += 2)
  {
    // Clear previous results.
    map.clear();

    // We integrate ray twice, ensuring the distance stays truncated.
    for (int j = 0; j < 2; ++j)
    {
      // Must initialise the voxel reference after the map clear or it may become invalid.
      ohm::Voxel<const ohm::VoxelTsdf> tsdf(&map, map.layout().layerIndex(ohm::default_layer::tsdfLayerName()));
      ASSERT_TRUE(tsdf.isLayerValid()) << "ray index " << i / 2u;
      // Trace ray.
      tsdf_mapper.integrateRays(&rays[i], 2, nullptr, nullptr, 0);
      tsdf_mapper.syncVoxels();

      const size_t index_offset = query.resultIndices()[i / 2u];
      const size_t key_count = query.resultCounts()[i / 2u];

      const ohm::Key end_voxel_key = map.voxelKey(rays[i + 1]);
      for (size_t k = 0; k < key_count; ++k)
      {
        const ohm::Key key = query.intersectedVoxels()[index_offset + k];
        if (key == end_voxel_key)
        {
          // Don't test the end voxel. Distance will be less.
          continue;
        }

        ohm::setVoxelKey(key, tsdf);
        ASSERT_TRUE(tsdf.isValid()) << "ray index " << i / 2u;
        const ohm::VoxelTsdf tsdf_data = tsdf.data();
        EXPECT_EQ(tsdf_data.distance, tsdf_mapper.defaultTruncationDistance()) << "ray index " << i / 2u;
      }
    }
  }
}
}  // namespace tsdf
