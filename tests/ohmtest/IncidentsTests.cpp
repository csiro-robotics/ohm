// Copyright (c) 2020
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
#include <ohm/VoxelIncident.h>

#include <glm/gtx/norm.hpp>
#include <glm/vec3.hpp>

#include <random>

namespace incidents
{
void testIncidentNormals(ohm::OccupancyMap &map, ohm::RayMapper &mapper)
{
  const unsigned iterations = 10;
  const unsigned ray_count = 1000u;
  std::vector<glm::dvec3> rays;
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);

  ASSERT_TRUE(map.incidentNormalEnabled());

  rays.reserve(2 * ray_count);
  for (unsigned i = 0; i < iterations; ++i)
  {
    glm::vec3 expected_incident(0);
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

      glm::vec3 incident(origin);
      incident = glm::normalize(incident);

      // Do the same calculation made by the mapper, but without encoding.
      expected_incident = ohm::updateIncidentNormalV3(incident, expected_incident, r);
    }
    // Now use the ray mapper
    mapper.integrateRays(rays.data(), rays.size());

    // Check the result.
    ohm::Voxel<uint32_t> incident_voxel(&map, map.layout().layerIndex(ohm::default_layer::incidentNormalLayerName()));
    ASSERT_TRUE(incident_voxel.isLayerValid());
    incident_voxel.setKey(map.voxelKey(glm::dvec3(0)));
    ASSERT_TRUE(incident_voxel.isValid());
    const glm::vec3 extracted_incident = ohm::decodeNormal(incident_voxel.data());
    // Subtract the two incident vectors and ensure we the result is near zero.
    const glm::vec3 diff = expected_incident - extracted_incident;
    const float e = 1e-3f;
    EXPECT_NEAR(diff.x, 0.0f, e);
    EXPECT_NEAR(diff.y, 0.0f, e);
    EXPECT_NEAR(diff.z, 0.0f, e);
  }
}

TEST(Incident, WithOccupancy)
{
  // As Through, but using the NDT mapper.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kIncidentNormal);
  ohm::RayMapperOccupancy mapper(&map);
  testIncidentNormals(map, mapper);
}

TEST(Incident, WithNdt)
{
  // As Through, but using the NDT mapper.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kIncidentNormal);
  ohm::NdtMap ndt_map(&map, true);
  ohm::RayMapperNdt mapper(&ndt_map);
  testIncidentNormals(map, mapper);
}
}  // namespace incidents
