// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/Aabb.h>
#include <ohm/KeyList.h>
#include <ohm/MapChunk.h>
#include <ohm/MapProbability.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/VoxelData.h>

#include <ohmtools/OhmCloud.h>

#include <ohmutil/GlmStream.h>
#include <ohmutil/OhmUtil.h>

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

void printVoxelPositionResults(const std::vector<VoxelMeanResult> &mean_results, bool common_voxel_centre,
                               double map_resolution)
{
  if (mean_results.empty())
  {
    return;
  }

  std::cout << std::setfill(' ');

  // Error checking first, then pretty reporting.
  for (const VoxelMeanResult &result : mean_results)
  {
    EXPECT_NEAR(result.expected_position.x, result.reported_position.x, map_resolution / 1e3);
    EXPECT_NEAR(result.expected_position.y, result.reported_position.y, map_resolution / 1e3);
    EXPECT_NEAR(result.expected_position.z, result.reported_position.z, map_resolution / 1e3);
  }

  if (common_voxel_centre)
  {
    std::cout << "Voxel centre: " << mean_results.front().voxel_centre << std::endl;
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

  for (const VoxelMeanResult &result : mean_results)
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

  Voxel<ohm::VoxelMean> mean(&map, map.layout().meanLayer(), map.voxelKey(glm::dvec3(0.5 * resolution)));
  ASSERT_TRUE(mean.isLayerValid());
  ASSERT_TRUE(mean.isValid());

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

  for (unsigned i = 0; i < sizeof(positions) / sizeof(positions[0]); ++i)
  {
    VoxelMeanResult sub_vox;

    setPositionSafe(mean, positions[i], 1);

    sub_vox.expected_position = positions[i];
    sub_vox.reported_position = positionSafe(mean);
    sub_vox.voxel_centre = map.voxelCentreGlobal(mean.key());

    results.push_back(sub_vox);
  }
  mean.reset();

  printVoxelPositionResults(results, true, map.resolution());
}

TEST(VoxelMean, LayoutToggle)
{
  // Test enabling and disabling voxel mean layout.
  const double resolution = 0.5;
  const glm::u8vec3 region_size(32);

  // Test core voxel mean positioning
  OccupancyMap map(resolution, region_size);

  // First integrate without voxel mean positioning
  glm::dvec3 sample_pos = glm::dvec3(1.1);
  const Key key(map.voxelKey(sample_pos));
  integrateHit(map, key);

  Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  Voxel<ohm::VoxelMean> mean(&map, map.layout().meanLayer());
  ASSERT_TRUE(occupancy.isLayerValid());
  ASSERT_FALSE(mean.isLayerValid());

  setVoxelKey(map.voxelKey(sample_pos), mean, occupancy);
  ASSERT_TRUE(occupancy.isValid());
  ASSERT_FALSE(mean.isValid());

  glm::dvec3 voxel_centre = map.voxelCentreGlobal(occupancy.key());
  glm::dvec3 voxel_pos = positionSafe(mean);

  EXPECT_EQ(voxel_centre.x, voxel_pos.x);
  EXPECT_EQ(voxel_centre.y, voxel_pos.y);
  EXPECT_EQ(voxel_centre.z, voxel_pos.z);
  occupancy.reset();
  mean.reset();

  // Now enable voxel mean positioning.
  // voxel becomes invalid.
  // Cache the current layout before voxel mean. We'll restore this layout later.
  MapLayout cached_layout(map.layout());

  map.addVoxelMeanLayer();

  occupancy = Voxel<const float>(&map, map.layout().occupancyLayer(), key);
  mean = Voxel<ohm::VoxelMean>(&map, map.layout().meanLayer(), key);
  ASSERT_TRUE(occupancy.isValid());
  ASSERT_TRUE(mean.isValid());

  // Set the voxel position.
  setPositionSafe(mean, sample_pos);

  // Position should no longer match the voxel centre.
  voxel_pos = positionSafe(mean);
  EXPECT_NE(voxel_centre.x, voxel_pos.x);
  EXPECT_NE(voxel_centre.y, voxel_pos.y);
  EXPECT_NE(voxel_centre.z, voxel_pos.z);

  EXPECT_NEAR(voxel_pos.x, sample_pos.x, resolution / 1000.0);
  EXPECT_NEAR(voxel_pos.y, sample_pos.y, resolution / 1000.0);
  EXPECT_NEAR(voxel_pos.z, sample_pos.z, resolution / 1000.0);
  occupancy.reset();
  mean.reset();

  // Now remove voxel mean positioning.
  map.updateLayout(cached_layout);
  occupancy = Voxel<const float>(&map, map.layout().occupancyLayer(), key);
  mean = Voxel<ohm::VoxelMean>(&map, map.layout().meanLayer(), key);
  ASSERT_TRUE(occupancy.isValid());
  ASSERT_FALSE(mean.isValid());

  // Expect occupancy to be unchanged.
  EXPECT_TRUE(isOccupied(occupancy));

  // Expect the position to match the voxel centre.
  voxel_pos = positionSafe(mean);
  EXPECT_EQ(voxel_centre.x, voxel_pos.x);
  EXPECT_EQ(voxel_centre.y, voxel_pos.y);
  EXPECT_EQ(voxel_centre.z, voxel_pos.z);
  occupancy.reset();
  mean.reset();
}

TEST(VoxelMean, Cpu)
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
  rays.emplace_back(glm::dvec3(0));
  rays.emplace_back(glm::dvec3(2.5, -1.6, -3.38));

  // Test basic map populate using GPU and ensure it matches CPU (close enough).
  OccupancyMap map(resolution, region_size, MapFlag::kVoxelMean);
  RayMapperOccupancy(&map).integrateRays(rays.data(), unsigned(rays.size()));

  std::vector<VoxelMeanResult> results;
  Voxel<ohm::VoxelMean> voxel(&map, map.layout().meanLayer());
  for (size_t i = 1; i < rays.size(); i += 2)
  {
    voxel.setKey(map.voxelKey(rays[i]));
    if (voxel.isValid())
    {
      VoxelMeanResult sub_vox;

      sub_vox.expected_position = rays[i];
      sub_vox.reported_position = positionSafe(voxel);
      sub_vox.voxel_centre = map.voxelCentreGlobal(voxel.key());

      results.push_back(sub_vox);
    }
  }
  voxel.reset();

  printVoxelPositionResults(results, false, map.resolution());
}
}  // namespace voxelmean
