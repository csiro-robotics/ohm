// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/LineWalk.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/RayMapperTsdf.h>
#include <ohm/VoxelData.h>

namespace tsdf
{
TEST(Tsdf, Basic)
{
  // Really not sure what sort of test to do here, so we'll keep it really simple.
  // For each ray we'll start with a clear map and populate the ray. They we'll check each voxel has the same
  // value as ohm::computeDistance() reports.
  const double resolution = 0.1;
  const glm::u8vec3 region_size(32);

  const std::vector<glm::dvec3> rays = {
    glm::dvec3(0.0), glm::dvec3(1.0, 0.0, 0.0),  glm::dvec3(0.0), glm::dvec3(-1.0, 0.0, 0.0),
    glm::dvec3(0.0), glm::dvec3(1.0, 1.0, 0.0),  glm::dvec3(0.0), glm::dvec3(-1.0, 1.0, 0.0),
    glm::dvec3(0.0), glm::dvec3(1.0, 0.0, 0.0),  glm::dvec3(0.0), glm::dvec3(1.0, -1.0, 0.0),
    glm::dvec3(0.0), glm::dvec3(-1.0, 0.0, 1.0), glm::dvec3(0.0), glm::dvec3(1.0, 1.0, 1.0),
    glm::dvec3(0.0), glm::dvec3(-1.0, 1.0, 1.0), glm::dvec3(0.0), glm::dvec3(1.0, 0.0, 1.0),
    glm::dvec3(0.0), glm::dvec3(1.0, -1.0, 1.0), glm::dvec3(0.0), glm::dvec3(-1.0, 0.0, 1.0),
    glm::dvec3(0.0), glm::dvec3(1.0, 1.0, -1.0), glm::dvec3(0.0), glm::dvec3(-1.0, 1.0, -1.0),
    glm::dvec3(0.0), glm::dvec3(1.0, 0.0, -1.0), glm::dvec3(0.0), glm::dvec3(1.0, -1.0, -1.0),
  };

  // Test core voxel mean positioning
  ohm::OccupancyMap map(resolution, region_size, ohm::MapFlag::kTsdf);
  ohm::RayMapperTsdf tsdf_mapper(&map);

  // Offset the map origin to trace between voxel centres.
  map.setOrigin(glm::dvec3(-0.5 * map.resolution()));
  // Set large truncation distance so our values matche computeDistance() results.
  tsdf_mapper.setDefaultTruncationDistance(10.0);

  for (size_t i = 0; i < rays.size(); i += 2)
  {
    map.clear();
    // Must initialise the voxel reference after the map clear or it may become invalid.
    ohm::Voxel<const ohm::VoxelTsdf> tsdf(&map, map.layout().layerIndex(ohm::default_layer::tsdfLayerName()));
    ASSERT_TRUE(tsdf.isLayerValid()) << "ray index " << i / 2u;
    // Trace ray.
    tsdf_mapper.integrateRays(&rays[i], 2, nullptr, nullptr, 0);

    const auto visit_func = [&](const ohm::Key &key, double enter_range, double exit_range,
                                const glm::ivec3 &steps_remaining) -> bool  //
    {
      ohm::setVoxelKey(key, tsdf);
      EXPECT_TRUE(tsdf.isValid()) << "ray index " << i / 2u;
      if (!tsdf.isValid())
      {
        return false;
      }
      const ohm::VoxelTsdf tsdf_data = tsdf.data();
      EXPECT_NEAR(tsdf_data.distance, ohm::computeDistance(rays[i], rays[i + 1], map.voxelCentreGlobal(key)), 1e-6)
        << "ray index " << i / 2u;
      return true;
    };

    ohm::walkSegmentKeys(ohm::LineWalkContext(map, visit_func), rays[i], rays[i + 1]);
  }
}
}  // namespace tsdf
