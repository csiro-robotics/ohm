// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/DefaultLayer.h>
#include <ohm/Key.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperNdt.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>
#include <ohm/VoxelIncident.h>
#include <ohm/VoxelMean.h>

#include <ohmgpu/GpuMap.h>
#include <ohmgpu/GpuNdtMap.h>
#include <ohmgpu/RayItem.h>

#include <glm/gtx/norm.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <random>

namespace incidents
{
void testIncidentNormals(ohm::OccupancyMap &map, ohm::GpuMap &mapper, bool single_ray_batches)
{
  const unsigned iterations = 10;
  const unsigned ray_count = 1000u;
  std::vector<ohm::RayItem> ray_items;
  std::vector<glm::dvec3> rays;
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);

  // Set the map origin to avoid (0, 0, 0) being on a voxel boundary.
  map.setOrigin(glm::dvec3(-0.5 * map.resolution()));

  ASSERT_TRUE(map.incidentNormalEnabled());
  ASSERT_TRUE(map.voxelMeanEnabled());

  ray_items.reserve(ray_count);
  rays.reserve(2 * ray_count);
  for (unsigned i = 0; i < iterations; ++i)
  {
    unsigned expected_packed = 0;
    ray_items.clear();
    rays.clear();

    // We build into RayItem entries to ensure we sort into the same order as the GPU will and can process in that
    // order.
    for (unsigned r = 0; r < ray_count; ++r)
    {
      // Sample is at the origin. We'll build random rays around that.
      ohm::RayItem item{};
      item.sample = glm::dvec3(0.0);
      do
      {
        item.origin = glm::dvec3(uniform(rng), uniform(rng), uniform(rng));
      } while (glm::length2(item.origin) < 1e-6);  // Make sure it's not degenerate.
      // Normalise the origin ray, then expand it out to be larger than a single voxel.
      item.origin = glm::normalize(item.origin);
      item.origin *= map.resolution() * 3;
      item.origin_key = map.voxelKey(item.origin);
      item.origin_key = map.voxelKey(item.origin);
      item.sample_key = map.voxelKey(item.sample);
      ray_items.push_back(item);
    }

    // Now sort the ray items.
    std::sort(ray_items.begin(), ray_items.end());

    // Migrate into rays to use mapper API and make the expected calculations.
    for (unsigned r = 0; r < unsigned(ray_items.size()); ++r)
    {
      // Do the same calculation made by the mapper, but without encoding.
      const auto &item = ray_items[r];
      rays.emplace_back(item.origin);
      rays.emplace_back(item.sample);
      expected_packed = ohm::updateIncidentNormal(expected_packed, glm::vec3(item.origin) - glm::vec3(item.sample), r);
    }
    // Now use the ray mapper
    if (single_ray_batches)
    {
      // Need to add the rays one at a time because it's heavily affected by order (non deterministic)
      for (unsigned r = 0; r < ray_count; ++r)
      {
        mapper.integrateRays(rays.data() + r * 2, 2);
      }
    }
    else
    {
      mapper.integrateRays(rays.data(), ray_count * 2);
    }
    mapper.syncVoxels();

    // Check the result.
    ohm::Voxel<uint32_t> incident_voxel(&map, map.layout().layerIndex(ohm::default_layer::incidentNormalLayerName()));
    ASSERT_TRUE(incident_voxel.isLayerValid());
    incident_voxel.setKey(map.voxelKey(glm::dvec3(0)));
    ASSERT_TRUE(incident_voxel.isValid());

    // Convert the vectors for comparison - CPU works in double while GPU is float so we expect some error.
    const glm::vec3 mapped_incident = ohm::decodeNormal(incident_voxel.data());
    const glm::vec3 expected_incident = ohm::decodeNormal(expected_packed);
    const glm::vec3 diff_incident = expected_incident - mapped_incident;
    EXPECT_NEAR(glm::length(diff_incident), 0, 1e-2f);

    // Clear the incident normal
    incident_voxel.write(0);
    EXPECT_EQ(incident_voxel.data(), 0);

    // Also clear the voxel mean as we use the sample count from there.
    ohm::Voxel<ohm::VoxelMean> mean_voxel(&map, map.layout().meanLayer());
    ASSERT_TRUE(mean_voxel.isLayerValid());
    mean_voxel.setKey(map.voxelKey(glm::dvec3(0)));
    ASSERT_TRUE(mean_voxel.isValid());

    mean_voxel.write(ohm::VoxelMean{});
    EXPECT_EQ(mean_voxel.data().count, 0);
  }
}

TEST(Incident, WithOccupancy)
{
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kIncidentNormal);
  ohm::GpuMap mapper(&map, true);
  // Must do one ray at a time because of non-determinism.
  testIncidentNormals(map, mapper, true);
}

TEST(Incident, WithNdt)
{
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kIncidentNormal);
  ohm::GpuNdtMap mapper(&map, true);
  // NDT can batch rays because sample update is sequential.
  testIncidentNormals(map, mapper, false);
  // testIncidentNormals(map, mapper, true);
}
}  // namespace incidents
