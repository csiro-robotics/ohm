// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Aabb.h>
#include <ohm/KeyList.h>
#include <ohm/MapProbability.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/VoxelData.h>
#include <ohmgpu/GpuCache.h>
#include <ohmgpu/GpuLayerCache.h>
#include <ohmgpu/GpuMap.h>

#include <logutil/LogUtil.h>

#include <ohmtools/OhmCloud.h>
#include <ohmutil/GlmStream.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

using namespace ohm;

namespace voxelmean
{
typedef std::chrono::high_resolution_clock TimingClock;

struct VoxelMeanResult
{
  glm::dvec3 expected_position;
  glm::dvec3 reported_position;
  glm::dvec3 voxel_centre;
};

void printVoxelPositionResults(const std::vector<VoxelMeanResult> &voxel_mean_results, bool common_voxel_centre,
                               double map_resolution)
{
  if (voxel_mean_results.empty())
  {
    return;
  }

  std::cout << std::setfill(' ');

  // Error checking first, then pretty reporting.
  for (const VoxelMeanResult &result : voxel_mean_results)
  {
    EXPECT_NEAR(result.expected_position.x, result.reported_position.x, map_resolution / 1e3);
    EXPECT_NEAR(result.expected_position.y, result.reported_position.y, map_resolution / 1e3);
    EXPECT_NEAR(result.expected_position.z, result.reported_position.z, map_resolution / 1e3);
  }

  if (common_voxel_centre)
  {
    std::cout << "Voxel centre: " << voxel_mean_results.front().voxel_centre << std::endl;
  }

  glm::dvec3 pos_error;
  const int col = 30;
  std::ostringstream s;
  std::cout << std::left;
  std::cout << std::setw(col) << "Input position" << std::setw(col) << "Voxel mean";
  if (!common_voxel_centre)
  {
    std::cout << std::setw(col) << "Centre";
  }
  std::cout << std::setw(col) << "error" << '\n';

  for (const VoxelMeanResult &result : voxel_mean_results)
  {
    pos_error = result.expected_position - result.reported_position;

    s << result.expected_position;
    std::cout << std::setw(col) << s.str();
    s.str(std::string());
    s << result.reported_position;
    std::cout << std::setw(col) << s.str();
    s.str(std::string());
    if (!common_voxel_centre)
    {
      s << result.voxel_centre;
      std::cout << std::setw(col) << s.str();
      s.str(std::string());
    }
    s << pos_error;
    std::cout << std::setw(col) << s.str();
    s.str(std::string());
    std::cout << std::endl;
  }
}

TEST(VoxelMean, Basic)
{
  const double resolution = 0.5;
  const glm::u8vec3 region_size(32);

  // Test core voxel mean positioning
  OccupancyMap map(resolution, region_size, MapFlag::kVoxelMean);

  static const glm::dvec3 positions[] =  //
    {
      //
      glm::dvec3(0.0),   //
      glm::dvec3(0.05),  //
      glm::dvec3(0.15),  //
      glm::dvec3(0.20),  //
      glm::dvec3(0.25),  //
      glm::dvec3(0.30),  //
      glm::dvec3(0.35),  //
      glm::dvec3(0.40),  //
      glm::dvec3(0.45),  //
      glm::dvec3(0.50),  //
    };

  std::vector<VoxelMeanResult> results;

  Voxel<ohm::VoxelMean> voxel(&map, map.layout().meanLayer(), map.voxelKey(glm::dvec3(0.5 * resolution)));
  ASSERT_TRUE(voxel.isValid());
  for (unsigned i = 0; i < sizeof(positions) / sizeof(positions[0]); ++i)
  {
    VoxelMeanResult sub_vox;

    setPositionSafe(voxel, positions[i]);

    sub_vox.expected_position = positions[i];
    sub_vox.reported_position = positionSafe(voxel);
    sub_vox.voxel_centre = map.voxelCentreGlobal(voxel.key());

    results.push_back(sub_vox);
  }
  voxel.reset();

  printVoxelPositionResults(results, true, map.resolution());
}

TEST(VoxelMean, LayoutToggle)
{
  // Test enabling and disabling voxel mean layout.
  const double resolution = 0.5;
  const glm::u8vec3 region_size(32);

  // Test core voxel mean positioning
  OccupancyMap map(resolution, region_size);
  // Setup a GPU cache to validate the change in cache size.
  GpuMap gpu_wrap(&map, true, 2048);  // Borrow pointer.

  ASSERT_TRUE(gpu_wrap.gpuOk());

  const GpuLayerCache *gpu_occupancy_cache = gpu_wrap.gpuCache()->layerCache(kGcIdOccupancy);
  const size_t without_voxel_means_chunk_size = gpu_occupancy_cache->chunkSize();

  // First integrate without voxel mean positioning
  glm::dvec3 sample_pos = glm::dvec3(1.1);
  ohm::integrateHit(map, sample_pos);

  glm::dvec3 voxel_centre = map.voxelCentreGlobal(map.voxelKey(sample_pos));

  {
    Voxel<const ohm::VoxelMean> voxel_mean(&map, map.layout().meanLayer(), map.voxelKey(sample_pos));
    // Note: voxel_mean has no valid layer, but the key is valid.
    glm::dvec3 voxel_pos = positionSafe(voxel_mean);
    EXPECT_EQ(voxel_centre.x, voxel_pos.x);
    EXPECT_EQ(voxel_centre.y, voxel_pos.y);
    EXPECT_EQ(voxel_centre.z, voxel_pos.z);
  }

  // Now enable voxel mean positioning.
  // voxel becomes invalid.
  // Cache the voxel layout so we can add and remove the voxel mean layer.
  MapLayout cached_layout = map.layout();

  gpu_occupancy_cache = nullptr;
  map.addVoxelMeanLayer();
  gpu_occupancy_cache = gpu_wrap.gpuCache()->layerCache(kGcIdOccupancy);

  EXPECT_NE(gpu_wrap.gpuCache()->layerCache(kGcIdVoxelMean), nullptr);

  const size_t with_voxel_means_chunk_size = gpu_occupancy_cache->chunkSize();
  Voxel<ohm::VoxelMean> voxel_mean(&map, map.layout().meanLayer(), map.voxelKey(sample_pos));
  Voxel<const float> voxel_occupancy(voxel_mean, map.layout().occupancyLayer());
  ASSERT_TRUE(voxel_mean.isValid());
  ASSERT_TRUE(voxel_occupancy.isValid());

  EXPECT_EQ(with_voxel_means_chunk_size, without_voxel_means_chunk_size);
  unsigned active_gpu_layers = 0;
  for (unsigned i = 0; i < gpu_wrap.gpuCache()->layerCount(); ++i)
  {
    if (gpu_wrap.gpuCache()->layerCache(i) != nullptr)
    {
      ++active_gpu_layers;
    }
  }
  EXPECT_EQ(active_gpu_layers, 2);

  EXPECT_TRUE(ohm::isOccupied(voxel_occupancy));

  // Set the voxel position.
  setPositionSafe(voxel_mean, sample_pos, 1);

  // Position should no longer match the voxel centre.
  glm::dvec3 voxel_pos = positionSafe(voxel_mean);
  EXPECT_NE(voxel_centre.x, voxel_pos.x);
  EXPECT_NE(voxel_centre.y, voxel_pos.y);
  EXPECT_NE(voxel_centre.z, voxel_pos.z);

  EXPECT_NEAR(voxel_pos.x, sample_pos.x, resolution / 1000.0);
  EXPECT_NEAR(voxel_pos.y, sample_pos.y, resolution / 1000.0);
  EXPECT_NEAR(voxel_pos.z, sample_pos.z, resolution / 1000.0);

  voxel_mean.reset();
  voxel_occupancy.reset();

  // Now remove voxel mean positioning.
  gpu_occupancy_cache = nullptr;
  map.updateLayout(cached_layout);
  EXPECT_EQ(gpu_wrap.gpuCache()->layerCount(), kGcIdOccupancy + 1);
  gpu_occupancy_cache = gpu_wrap.gpuCache()->layerCache(kGcIdOccupancy);
  const size_t without_voxel_means_chunk_size2 = gpu_occupancy_cache->chunkSize();

  voxel_mean = Voxel<ohm::VoxelMean>(&map, map.layout().meanLayer(), map.voxelKey(sample_pos));
  voxel_occupancy = Voxel<const float>(voxel_mean, map.layout().occupancyLayer());

  EXPECT_EQ(without_voxel_means_chunk_size2, without_voxel_means_chunk_size);

  // Expect occupancy to be unchanged.
  ASSERT_FALSE(voxel_mean.isValid());
  ASSERT_TRUE(voxel_occupancy.isValid());
  EXPECT_TRUE(ohm::isOccupied(voxel_occupancy));

  // Expect the position to match the voxel centre.
  voxel_pos = positionSafe(voxel_mean);
  EXPECT_EQ(voxel_centre.x, voxel_pos.x);
  EXPECT_EQ(voxel_centre.y, voxel_pos.y);
  EXPECT_EQ(voxel_centre.z, voxel_pos.z);
  voxel_mean.reset();
  voxel_occupancy.reset();
}

TEST(VoxelMean, Gpu)
{
  const double resolution = 0.5;
  const unsigned batch_size = 1;
  const glm::u8vec3 region_size(32);

  // Make a ray.
  std::vector<glm::dvec3> rays;
  rays.emplace_back(glm::dvec3(0));
  rays.emplace_back(glm::dvec3(1.1));
  rays.emplace_back(glm::dvec3(0));
  rays.emplace_back(glm::dvec3(-2.4));
  rays.emplace_back(glm::dvec3(0));
  rays.emplace_back(glm::dvec3(1, -2.2, -3.3));

  // Test basic map populate using GPU and ensure it matches CPU (close enough).
  OccupancyMap map(resolution, region_size, MapFlag::kVoxelMean);
  GpuMap gpu_wrap(&map, true, unsigned(batch_size * 2));  // Borrow pointer.

  ASSERT_TRUE(gpu_wrap.gpuOk());

  gpu_wrap.integrateRays(rays.data(), unsigned(rays.size()));
  gpu_wrap.syncVoxels();

  std::vector<VoxelMeanResult> results;
  Voxel<const ohm::VoxelMean> voxel(&map, map.layout().meanLayer());
  ASSERT_TRUE(voxel.isLayerValid());
  for (size_t i = 1; i < rays.size(); i += 2)
  {
    voxel.setKey(map.voxelKey(rays[i]));
    ASSERT_TRUE(voxel.isValid());
    VoxelMeanResult sub_vox;

    sub_vox.expected_position = rays[i];
    sub_vox.reported_position = positionSafe(voxel);
    sub_vox.voxel_centre = map.voxelCentreGlobal(voxel.key());

    results.push_back(sub_vox);
  }
  voxel.reset();

  printVoxelPositionResults(results, false, map.resolution());
}

TEST(VoxelMean, Compare)
{
  const double resolution = 0.5;
  const unsigned batch_size = 1;
  const glm::u8vec3 region_size(32);

  // Make a ray.
  std::vector<glm::dvec3> rays;
  rays.emplace_back(glm::dvec3(0));
  rays.emplace_back(glm::dvec3(1.1));
  rays.emplace_back(glm::dvec3(0));
  rays.emplace_back(glm::dvec3(-2.4));
  rays.emplace_back(glm::dvec3(0));
  rays.emplace_back(glm::dvec3(1, -2.2, -3.3));

  // Test basic map populate using GPU and ensure it matches CPU (close enough).
  OccupancyMap cpu_map(resolution, region_size, MapFlag::kVoxelMean);
  OccupancyMap gpu_map(resolution, region_size, MapFlag::kVoxelMean);
  GpuMap gpu_wrap(&gpu_map, true, unsigned(batch_size * 2));  // Borrow pointer.

  // In this test we don't adjust the voxel mean weighting. We just validate we get the same results in GPU and CPU.

  ASSERT_TRUE(gpu_wrap.gpuOk());

  cpu_map.integrateRays(rays.data(), unsigned(rays.size()));
  gpu_wrap.integrateRays(rays.data(), unsigned(rays.size()));
  gpu_wrap.syncVoxels();

  std::vector<VoxelMeanResult> results;
  Voxel<const ohm::VoxelMean> cpu_voxel(&cpu_map, cpu_map.layout().meanLayer());
  Voxel<const ohm::VoxelMean> gpu_voxel(&gpu_map, gpu_map.layout().meanLayer());
  ASSERT_TRUE(cpu_voxel.isLayerValid());
  ASSERT_TRUE(gpu_voxel.isLayerValid());
  for (size_t i = 1; i < rays.size(); i += 2)
  {
    cpu_voxel.setKey(cpu_map.voxelKey(rays[i]));
    gpu_voxel.setKey(gpu_map.voxelKey(rays[i]));
    EXPECT_TRUE(cpu_voxel.isValid());
    EXPECT_EQ(cpu_voxel.isValid(), gpu_voxel.isValid());
    if (cpu_voxel.isValid() && gpu_voxel.isValid())
    {
      VoxelMeanResult sub_vox;

      sub_vox.expected_position = positionSafe(cpu_voxel);
      sub_vox.reported_position = positionSafe(gpu_voxel);
      sub_vox.voxel_centre = cpu_map.voxelCentreGlobal(cpu_voxel.key());

      results.push_back(sub_vox);
    }
  }
  cpu_voxel.reset();
  gpu_voxel.reset();

  printVoxelPositionResults(results, false, cpu_map.resolution());
}
}  // namespace voxelmean
