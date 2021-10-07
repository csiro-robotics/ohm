// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/DefaultLayer.h>
#include <ohm/Key.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperNdt.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>
#include <ohm/VoxelTouchTime.h>

#include <glm/gtx/norm.hpp>
#include <glm/vec3.hpp>

#include <random>

namespace touchtime
{
void testTouchTime(ohm::OccupancyMap &map, ohm::RayMapper &mapper)
{
  const unsigned iterations = 10;
  const unsigned ray_count = 1000u;
  const double time_base = 1000.0;
  const double time_step = 0.5;
  std::vector<glm::dvec3> rays;
  std::vector<double> timestamps;
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);

  // Set the map origin to avoid (0, 0, 0) being on a voxel boundary.
  map.setOrigin(glm::dvec3(-0.5 * map.resolution()));

  ASSERT_TRUE(map.touchTimeEnabled());

  rays.reserve(2 * ray_count);
  for (unsigned i = 0; i < iterations; ++i)
  {
    rays.clear();
    timestamps.clear();
    for (unsigned r = 0; r < ray_count; ++r)
    {
      // Sample is at the origin. We'll build random rays around that.
      glm::dvec3 origin;
      do
      {
        origin = glm::dvec3(uniform(rng), uniform(rng), uniform(rng));
      } while (glm::length2(origin) < 1e-6);  // Make sure it's not degenerate.
      // Normalise the origin ray, then expand it out to be larger than a single voxel.
      origin = glm::normalize(origin);
      origin *= map.resolution() * 3;
      rays.emplace_back(origin);
      rays.emplace_back(glm::dvec3(0));

      timestamps.emplace_back(time_base + r * time_step);
    }
    // Now use the ray mapper
    mapper.integrateRays(rays.data(), rays.size(), nullptr, timestamps.data(), ohm::kRfDefault);

    // Check the result.
    ohm::Voxel<uint32_t> time_voxel(&map, map.layout().layerIndex(ohm::default_layer::touchTimeLayerName()));
    ASSERT_TRUE(time_voxel.isLayerValid());
    time_voxel.setKey(map.voxelKey(glm::dvec3(0)));
    ASSERT_TRUE(time_voxel.isValid());

    const double extracted_time = ohm::decodeVoxelTouchTime(map.firstRayTime(), time_voxel.data());
    EXPECT_EQ(extracted_time, timestamps.back());

    time_voxel.write(0u);
    EXPECT_EQ(time_voxel.data(), 0);
  }
}

TEST(TouchTime, WithOccupancy)
{
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kTouchTime);
  ohm::RayMapperOccupancy mapper(&map);
  testTouchTime(map, mapper);
}

TEST(TouchTime, WithNdt)
{
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kTouchTime);
  ohm::NdtMap ndt_map(&map, true);
  ohm::RayMapperNdt mapper(&ndt_map);
  testTouchTime(map, mapper);
}
}  // namespace touchtime
