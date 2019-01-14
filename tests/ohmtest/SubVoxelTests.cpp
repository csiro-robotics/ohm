// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Aabb.h>
#include <ohm/GpuMap.h>
#include <ohm/KeyList.h>
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
    glm::dvec3 original_position;
    glm::dvec3 reported_position;
    glm::dvec3 voxel_centre;
  };

  void printVoxelPositionResults(const std::vector<SubVoxelResult> &sub_voxel_results, bool common_voxel_centre)
  {
    if (sub_voxel_results.empty())
    {
      return;
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
      pos_error = result.original_position - result.reported_position;

      s << result.original_position;
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

      sub_vox.original_position = positions[i];
      sub_vox.reported_position = voxel.position();
      sub_vox.voxel_centre = voxel.centreGlobal();

      results.push_back(sub_vox);
    }

    printVoxelPositionResults(results, true);
  }

  TEST(SubVoxel, Cpu)
  {
    const double resolution = 0.5;
    const unsigned batch_size = 1;
    const glm::u8vec3 region_size(32);

    // Make a ray.
    std::vector<glm::dvec3> rays;
    rays.emplace_back(glm::dvec3(0.3));
    rays.emplace_back(glm::dvec3(1.1));

    // rays.emplace_back(glm::dvec3(-5.0));
    // rays.emplace_back(glm::dvec3(0.3));

    // Test basic map populate using GPU and ensure it matches CPU (close enough).
    OccupancyMap map(resolution, region_size, true);

    // map.integrateRays(rays.data(), unsigned(rays.size()));
    map.integrateHit(rays[1]);

    std::vector<SubVoxelResult> results;
    for (size_t i = 1; i < rays.size(); i += 2)
    {
      const VoxelConst voxel = map.voxel(map.voxelKey(rays[i]));
      if (voxel.isValid())
      {
        SubVoxelResult sub_vox;

        sub_vox.original_position = rays[i];
        sub_vox.reported_position = voxel.position();
        sub_vox.voxel_centre = voxel.centreGlobal();

        results.push_back(sub_vox);
      }
    }

    printVoxelPositionResults(results, false);
  }

  TEST(SubVoxel, Gpu)
  {
    const double resolution = 0.5;
    const unsigned batch_size = 1;
    const glm::u8vec3 region_size(32);

    // Make a ray.
    std::vector<glm::dvec3> rays;
    rays.emplace_back(glm::dvec3(0.3));
    rays.emplace_back(glm::dvec3(1.1));

    // Test basic map populate using GPU and ensure it matches CPU (close enough).
    OccupancyMap map(resolution, region_size, true);
    GpuMap gpu_wrap(&map, true, unsigned(batch_size * 2));  // Borrow pointer.

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

        sub_vox.original_position = rays[i];
        sub_vox.reported_position = voxel.position();
        sub_vox.voxel_centre = voxel.centreGlobal();

        results.push_back(sub_vox);
      }
    }

    printVoxelPositionResults(results, false);
  }
}  // namespace subvoxel
