// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/Key.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperNdt.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>

#include <glm/vec3.hpp>

namespace decayratetests
{
void testThrough(ohm::OccupancyMap &map, ohm::RayMapper &mapper)
{
  ASSERT_GT(map.layout().decayRateLayer(), 0);

  // Offset the map so that we can cast rays through the origin without incurring floating point ambituity on the target
  // voxel.
  map.setOrigin(glm::dvec3(-0.5f * map.resolution()));

  // Create the rays we are to cast.
  const std::vector<glm::dvec3> rays = { glm::dvec3(-1, 0, 0), glm::dvec3(1, 0, 0),  glm::dvec3(0, -1, 0),
                                         glm::dvec3(0, 1, 0),  glm::dvec3(0, 0, -1), glm::dvec3(0, 0, 1) };

  ohm::Voxel<const float> decay_rate_voxel(&map, map.layout().decayRateLayer());
  ASSERT_TRUE(decay_rate_voxel.isLayerValid());

  // Integrate and test one ray at a time.
  float expected_decay_rate = 0;
  for (size_t i = 0; i < rays.size(); i += 2)
  {
    mapper.integrateRays(&rays[i], 2);
    expected_decay_rate += float(map.resolution());
    const auto key = map.voxelKey(glm::dvec3(0));
    decay_rate_voxel.setKey(key);
    ASSERT_TRUE(decay_rate_voxel.isValid());
    EXPECT_NEAR(decay_rate_voxel.data(), expected_decay_rate, 1e-3f);
  }
}

TEST(DecayRate, Through)
{
  // For this test, we want to ensure that the decay rate accumulates correctly. To do that we select a set of rays
  // which pass through a voxel perpendicular to various faces of that voxel. In this way we know that the decay
  // rate should accumulate at approximately the voxel resolution per ray.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kDecayRate);
  ohm::RayMapperOccupancy mapper(&map);
  testThrough(map, mapper);
}

TEST(DecayRate, ThroughNdt)
{
  // As Through, but using the NDT mapper.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kDecayRate);
  ohm::NdtMap ndt(&map, true);
  ohm::RayMapperNdt mapper(&ndt);
  testThrough(map, mapper);
}

void testInto(ohm::OccupancyMap &map, ohm::RayMapper &mapper)
{
  ASSERT_GT(map.layout().decayRateLayer(), 0);

  // Offset the map so that we can cast rays through the origin without incurring floating point ambituity on the target
  // voxel.
  map.setOrigin(glm::dvec3(-0.5f * map.resolution()));

  // Create the rays we are to cast.
  const std::vector<glm::dvec3> rays = {
    glm::dvec3(-1, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(0, -1, 0), glm::dvec3(0, 0, 0),
    glm::dvec3(0, 0, -1), glm::dvec3(0, 0, 0), glm::dvec3(1, 0, 0),  glm::dvec3(0, 0, 0),
    glm::dvec3(0, 1, 0),  glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 1),  glm::dvec3(0, 0, 0),
  };

  ohm::Voxel<const float> decay_rate_voxel(&map, map.layout().decayRateLayer());
  ASSERT_TRUE(decay_rate_voxel.isLayerValid());

  // Integrate and test one ray at a time.
  float expected_decay_rate = 0;
  for (size_t i = 0; i < rays.size(); i += 2)
  {
    mapper.integrateRays(&rays[i], 2);
    expected_decay_rate += float(0.5 * map.resolution());
    const auto key = map.voxelKey(glm::dvec3(0));
    decay_rate_voxel.setKey(key);
    ASSERT_TRUE(decay_rate_voxel.isValid());
    EXPECT_NEAR(decay_rate_voxel.data(), expected_decay_rate, 1e-3f);
  }
}

TEST(DecayRate, Into)
{
  // For this test, we want to ensure that the decay rate accumulates correctly in the final voxel. To do that we select
  // a set of rays which end in a voxel a voxel entering perpendicular to various faces of that voxel. In this way we
  // know that the decay rate should accumulate at approximately half the voxel resolution per ray.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kDecayRate);
  ohm::RayMapperOccupancy mapper(&map);
  testInto(map, mapper);
}
}  // namespace decayratetests
