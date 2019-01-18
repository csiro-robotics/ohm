// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Aabb.h>
#include <ohm/GpuMap.h>
#include <ohm/KeyList.h>
#include <ohm/GpuCache.h>
#include <ohm/GpuLayerCache.h>
#include <ohm/MapCache.h>
#include <ohm/MapChunk.h>
#include <ohm/MapProbability.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>

#include <ohmtools/OhmCloud.h>
#include <ohmutil/OhmUtil.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

using namespace ohm;

namespace subvoxel
{
  typedef std::chrono::high_resolution_clock TimingClock;

  struct SubVoxelResult
  {
    glm::dvec3 expected_position;
    glm::dvec3 reported_position;
    glm::dvec3 voxel_centre;
  };

  void printVoxelPositionResults(const std::vector<SubVoxelResult> &sub_voxel_results, bool common_voxel_centre,
                                 double map_resolution)
  {
    if (sub_voxel_results.empty())
    {
      return;
    }

    std::cout << std::setfill(' ');

    // Error checking first, then pretty reporting.
    for (const SubVoxelResult &result : sub_voxel_results)
    {
      EXPECT_NEAR(result.expected_position.x, result.reported_position.x, map_resolution / 1e3);
      EXPECT_NEAR(result.expected_position.y, result.reported_position.y, map_resolution / 1e3);
      EXPECT_NEAR(result.expected_position.z, result.reported_position.z, map_resolution / 1e3);
    }

    if (common_voxel_centre)
    {
      std::cout << "Voxel centre: " << sub_voxel_results.front().voxel_centre << std::endl;
    }

    glm::dvec3 pos_error;
    const int col = 30;
    std::ostringstream s;
    std::cout << std::left;
    std::cout << std::setw(col) << "Input position" << std::setw(col) << "Sub-voxel";
    if (!common_voxel_centre)
    {
      std::cout << std::setw(col) << "Centre";
    }
    std::cout << std::setw(col) << "error" << '\n';

    for (const SubVoxelResult &result : sub_voxel_results)
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

  TEST(SubVoxel, Basic)
  {
    const double resolution = 0.5;
    const glm::u8vec3 region_size(32);

    // Test core sub-voxel positioning
    OccupancyMap map(resolution, region_size, true);

    Voxel voxel = map.voxel(map.voxelKey(glm::dvec3(0.5 * resolution)), true);

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

    std::vector<SubVoxelResult> results;

    for (unsigned i = 0; i < sizeof(positions) / sizeof(positions[0]); ++i)
    {
      SubVoxelResult sub_vox;

      voxel.setPosition(positions[i]);

      sub_vox.expected_position = positions[i];
      sub_vox.reported_position = voxel.position();
      sub_vox.voxel_centre = voxel.centreGlobal();

      results.push_back(sub_vox);
    }

    printVoxelPositionResults(results, true, map.resolution());
  }

  TEST(SubVoxel, LayoutToggle)
  {
    // Test enabling and disabling sub-voxel layout.
    const double resolution = 0.5;
    const glm::u8vec3 region_size(32);

    // Test core sub-voxel positioning
    OccupancyMap map(resolution, region_size, false);
    // Setup a GPU cache to validate the change in cache size.
    GpuMap gpu_wrap(&map, true, 2048);  // Borrow pointer.

    ASSERT_TRUE(gpu_wrap.gpuOk());

    const GpuLayerCache *gpu_occupancy_cache = gpu_wrap.gpuCache()->layerCache(map.layout().occupancyLayer());
    const size_t without_sub_voxels_chunk_size = gpu_occupancy_cache->chunkSize();

    // First integrate without sub-voxel positioning
    glm::dvec3 sample_pos = glm::dvec3(1.1);
    map.integrateHit(sample_pos);
    Voxel voxel = map.voxel(map.voxelKey(sample_pos), true);

    glm::dvec3 voxel_centre = voxel.centreGlobal();
    glm::dvec3 voxel_pos = voxel.position();

    EXPECT_EQ(voxel_centre.x, voxel_pos.x);
    EXPECT_EQ(voxel_centre.y, voxel_pos.y);
    EXPECT_EQ(voxel_centre.z, voxel_pos.z);

    // Now enable sub-voxel positioning.
    // voxel becomes invalid.

    map.setSubVoxelsEnabled(true);
    const size_t with_sub_voxels_chunk_size = gpu_occupancy_cache->chunkSize();
    voxel = map.voxel(map.voxelKey(sample_pos), true);

    EXPECT_GE(with_sub_voxels_chunk_size, without_sub_voxels_chunk_size);

    ASSERT_TRUE(voxel.isValid());
    EXPECT_TRUE(voxel.isOccupied());

    // Set the voxel position.
    voxel.setPosition(sample_pos);

    // Position should no longer match the voxel centre.
    voxel_pos = voxel.position();
    EXPECT_NE(voxel_centre.x, voxel_pos.x);
    EXPECT_NE(voxel_centre.y, voxel_pos.y);
    EXPECT_NE(voxel_centre.z, voxel_pos.z);

    EXPECT_NEAR(voxel_pos.x, sample_pos.x, resolution / 1000.0);
    EXPECT_NEAR(voxel_pos.y, sample_pos.y, resolution / 1000.0);
    EXPECT_NEAR(voxel_pos.z, sample_pos.z, resolution / 1000.0);

    // Now remove sub-voxel positioning.
    map.setSubVoxelsEnabled(false);
    const size_t without_sub_voxels_chunk_size2 = gpu_occupancy_cache->chunkSize();
    voxel = map.voxel(map.voxelKey(sample_pos), true);

    EXPECT_EQ(without_sub_voxels_chunk_size2, without_sub_voxels_chunk_size);

    // Expect occupancy to be unchanged.
    ASSERT_TRUE(voxel.isValid());
    EXPECT_TRUE(voxel.isOccupied());

    // Expect the position to match the voxel centre.
    voxel_pos = voxel.position();
    EXPECT_EQ(voxel_centre.x, voxel_pos.x);
    EXPECT_EQ(voxel_centre.y, voxel_pos.y);
    EXPECT_EQ(voxel_centre.z, voxel_pos.z);
  }

  TEST(SubVoxel, Cpu)
  {
    const double resolution = 0.5;
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
    OccupancyMap map(resolution, region_size, true);

    // Set the sub-voxel weighting to 1.0 to ensure we get a result close to the input value.
    map.setSubVoxelWeighting(1.0);

    map.integrateRays(rays.data(), unsigned(rays.size()));

    std::vector<SubVoxelResult> results;
    for (size_t i = 1; i < rays.size(); i += 2)
    {
      const VoxelConst voxel = map.voxel(map.voxelKey(rays[i]));
      if (voxel.isValid())
      {
        SubVoxelResult sub_vox;

        sub_vox.expected_position = rays[i];
        sub_vox.reported_position = voxel.position();
        sub_vox.voxel_centre = voxel.centreGlobal();

        results.push_back(sub_vox);
      }
    }

    printVoxelPositionResults(results, false, map.resolution());
  }

  TEST(SubVoxel, Gpu)
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
    OccupancyMap map(resolution, region_size, true);
    GpuMap gpu_wrap(&map, true, unsigned(batch_size * 2));  // Borrow pointer.

    // Set the sub-voxel weighting to 1.0 to ensure we get a result close to the input value.
    map.setSubVoxelWeighting(1.0);

    ASSERT_TRUE(gpu_wrap.gpuOk());

    gpu_wrap.integrateRays(rays.data(), unsigned(rays.size()));
    gpu_wrap.syncOccupancy();

    std::vector<SubVoxelResult> results;
    for (size_t i = 1; i < rays.size(); i += 2)
    {
      const VoxelConst voxel = map.voxel(map.voxelKey(rays[i]));
      if (voxel.isValid())
      {
        SubVoxelResult sub_vox;

        sub_vox.expected_position = rays[i];
        sub_vox.reported_position = voxel.position();
        sub_vox.voxel_centre = voxel.centreGlobal();

        results.push_back(sub_vox);
      }
    }

    printVoxelPositionResults(results, false, map.resolution());
  }

  TEST(SubVoxel, Compare)
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
    OccupancyMap cpu_map(resolution, region_size, true);
    OccupancyMap gpu_map(resolution, region_size, true);
    GpuMap gpu_wrap(&gpu_map, true, unsigned(batch_size * 2));  // Borrow pointer.

    // In this test we don't adjust the sub-voxel weighting. We just validate we get the same results in GPU and CPU.

    ASSERT_TRUE(gpu_wrap.gpuOk());

    cpu_map.integrateRays(rays.data(), unsigned(rays.size()));
    gpu_wrap.integrateRays(rays.data(), unsigned(rays.size()));
    gpu_wrap.syncOccupancy();

    std::vector<SubVoxelResult> results;
    for (size_t i = 1; i < rays.size(); i += 2)
    {
      const VoxelConst cpu_voxel = cpu_map.voxel(cpu_map.voxelKey(rays[i]));
      const VoxelConst gpu_voxel = gpu_map.voxel(gpu_map.voxelKey(rays[i]));
      EXPECT_EQ(cpu_voxel.isValid(), gpu_voxel.isValid());
      if (cpu_voxel.isValid() && gpu_voxel.isValid())
      {
        SubVoxelResult sub_vox;

        sub_vox.expected_position = cpu_voxel.position();
        sub_vox.reported_position = gpu_voxel.position();
        sub_vox.voxel_centre = cpu_voxel.centreGlobal();

        results.push_back(sub_vox);
      }
    }

    printVoxelPositionResults(results, false, cpu_map.resolution());
  }
}  // namespace subvoxel
