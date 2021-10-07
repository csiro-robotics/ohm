// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/RaysQuery.h>
#include <ohm/VoxelOccupancy.h>

#include <gtest/gtest.h>

namespace raysquerytests
{
TEST(RaysQuery, Cpu)
{
  const double base_scale = 10.0;
  const std::array<double, 3> query_scale = { 1.2, 1.2, 0.6 };
  const double resolution = 0.1;
  const std::vector<glm::dvec3> rays =  //
    {
      glm::dvec3(0.0), glm::dvec3(base_scale, 0, 0),                      //
      glm::dvec3(0.0), glm::dvec3(0, base_scale, 0),                      //
      glm::dvec3(0.0), glm::dvec3(0, 0, base_scale),                      //
      glm::dvec3(0.0), glm::dvec3(-base_scale, 0, 0),                     //
      glm::dvec3(0.0), glm::dvec3(0, -base_scale, 0),                     //
      glm::dvec3(0.0), glm::dvec3(0, 0, -base_scale),                     //
      glm::dvec3(0.0), glm::dvec3(base_scale, base_scale, 0),             //
      glm::dvec3(0.0), glm::dvec3(0, base_scale, base_scale),             //
      glm::dvec3(0.0), glm::dvec3(base_scale, 0, base_scale),             //
      glm::dvec3(0.0), glm::dvec3(-base_scale, -base_scale, 0),           //
      glm::dvec3(0.0), glm::dvec3(0, -base_scale, -base_scale),           //
      glm::dvec3(0.0), glm::dvec3(-base_scale, 0, -base_scale),           //
      glm::dvec3(0.0), glm::dvec3(base_scale, base_scale, base_scale),    //
      glm::dvec3(0.0), glm::dvec3(-base_scale, -base_scale, -base_scale)  //
    };

  // Build the map.
  ohm::OccupancyMap map(resolution);
  ohm::RayMapperOccupancy mapper(&map);

  // First add just the samples into the map, keeping the rest unobserved.
  for (size_t i = 1; i < rays.size(); i += 2)
  {
    ohm::integrateHit(map, map.voxelKey(rays[i]));
  }

  // Construct the the new RaysQuery object now so we can test the reset as well.
  ohm::RaysQuery query;
  query.setMap(&map);
  query.setVolumeCoefficient(1.0f);

  const std::array<ohm::OccupancyType, 3> expected_terminal_type = { ohm::OccupancyType::kOccupied,
                                                                     ohm::OccupancyType::kOccupied,
                                                                     ohm::OccupancyType::kFree };

  for (size_t iteration = 0; iteration < query_scale.size(); ++iteration)
  {
    // Scale the rays for lookup.
    for (size_t i = 0; i < rays.size(); i += 2)
    {
      query.addRay(rays[i], rays[i + 1] * query_scale[iteration]);
    }

    // Make the query.
    query.execute();

    // Compare results.
    ASSERT_EQ(query.numberOfResults(), rays.size() / 2);
    const double *ranges = query.ranges();
    const double *unobserved_volumes = query.unobservedVolumes();
    const ohm::OccupancyType *terminal_types = query.terminalOccupancyTypes();
    for (size_t i = 0; i < query.numberOfResults(); ++i)
    {
      // First iteration has the whole ray length unobserved. Second has zero.
      if (iteration > 0)
      {
        EXPECT_EQ(unobserved_volumes[i], 0.0);
      }
      else
      {
        // TODO: contrive a reasonable comparative calculation for the unobserved_volume results.
        EXPECT_GT(unobserved_volumes[i], 0.0);
      }

      double ray_length = 0;
      if (iteration < 2)
      {
        // We should reach the last voxel of each input ray on each query.
        const glm::dvec3 ray_delta = rays[i * 2 + 1] - rays[i * 2 + 0];
        ray_length = glm::length(ray_delta);
      }
      else
      {
        // We should reach the last voxel of each query ray on each query.
        const glm::dvec3 ray_delta = query.rays()[i * 2 + 1] - query.rays()[i * 2 + 0];
        ray_length = glm::length(ray_delta);
      }
      EXPECT_NEAR(ranges[i], ray_length, 1e-5);
      EXPECT_EQ(terminal_types[i], expected_terminal_type[iteration]);
    }


    // Now do a full ray integration and redo the comparison.
    if (iteration == 0)
    {
      mapper.integrateRays(rays.data(), rays.size());
    }

    // Hard reset to clear the rays
    query.reset(true);
  }
}
}  // namespace raysquerytests
