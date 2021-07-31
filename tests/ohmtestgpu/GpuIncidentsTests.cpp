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

#include <glm/gtx/norm.hpp>
#include <glm/vec3.hpp>

#include <random>

namespace incidents
{
void testIncidentNormals(ohm::OccupancyMap &map, ohm::GpuMap &mapper)
{
  const unsigned iterations = 10;
  const unsigned ray_count = 1000u;
  const float epsilon = 1e-2f;
  std::vector<glm::dvec3> rays;
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);

  // Set the map origin to avoid (0, 0, 0) being on a voxel boundary.
  map.setOrigin(glm::dvec3(-0.5 * map.resolution()));

  ASSERT_TRUE(map.incidentNormalEnabled());
  ASSERT_TRUE(map.voxelMeanEnabled());

  rays.reserve(2 * ray_count);
  for (unsigned i = 0; i < iterations; ++i)
  {
    unsigned expected_packed = 0;
    rays.clear();
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

      // Do the same calculation made by the mapper, but without encoding.
      expected_packed = ohm::updateIncidentNormal(expected_packed, origin, r);
    }
    // Now use the ray mapper
    // Need to add the rays one at a time because it's heavily affected by order (non deterministic)
    for (unsigned r = 0; r < ray_count; ++r)
    {
      mapper.integrateRays(rays.data() + r * 2, 2);
    }
    mapper.syncVoxels();

    // Check the result.
    ohm::Voxel<uint32_t> incident_voxel(&map, map.layout().layerIndex(ohm::default_layer::incidentNormalLayerName()));
    ASSERT_TRUE(incident_voxel.isLayerValid());
    incident_voxel.setKey(map.voxelKey(glm::dvec3(0)));
    ASSERT_TRUE(incident_voxel.isValid());

    EXPECT_EQ(incident_voxel.data(), expected_packed);

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
  testIncidentNormals(map, mapper);
}

TEST(Incident, WithNdt)
{
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kIncidentNormal);
  ohm::GpuNdtMap mapper(&map, true);
  testIncidentNormals(map, mapper);
}
}  // namespace incidents
