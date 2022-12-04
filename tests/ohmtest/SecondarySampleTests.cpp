// Copyright (c) 2022
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <ohm/Key.h>
#include <ohm/LineWalk.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/RayMapperSecondarySample.h>
#include <ohm/VoxelData.h>

#include <sstream>
#include <vector>

#include <gtest/gtest.h>

#include "ohmtestcommon/OhmTestUtil.h"

namespace sampletests
{
TEST(SecondarySample, ExcludeOrigin)
{
  std::ostringstream errors;

  // Test the behaviour of the ray flag kRfExcludeOrigin, fundamental to treating dual returns correctly.
  const std::vector<glm::dvec3> rays = { glm::dvec3(0, 0, 0), glm::dvec3(1, 0, 0) };
  ohm::OccupancyMap map(0.1);

  // Offset the map so that (0, 0, 0) is a voxel centre.
  map.setOrigin(glm::dvec3(0.5 * map.resolution()));

  ohm::RayMapperOccupancy occupancy_mapper(&map);
  occupancy_mapper.integrateRays(rays.data(), rays.size(), nullptr, nullptr, ohm::kRfExcludeOrigin);

  ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  ASSERT_TRUE(occupancy.isLayerValid());
  const auto visit_function = [&errors, &map, &occupancy, &rays](const ohm::Key &voxel_key, double enter_range,
                                                                 double exit_range) -> bool {
    (void)enter_range;
    (void)exit_range;

    occupancy.setKey(voxel_key);
    if (!occupancy.isValid())
    {
      errors << "occupancy voxel not valid\n";
      return false;
    }

    // Ensure voxel at (0, 0, 0) is still unobserved.
    float expected_occupancy = {};
    if (voxel_key == map.voxelKey(rays[0]))
    {
      // First voxel in the ray.
      expected_occupancy = ohm::unobservedOccupancyValue();
    }
    else if (voxel_key == map.voxelKey(rays[1]))
    {
      // Last voxel in ray.
      expected_occupancy = map.hitValue();
    }
    else
    {
      // Intermediary voxel.
      expected_occupancy = map.missValue();
    }

    if (occupancy.data() != expected_occupancy)
    {
      errors << "occupancy mismatch @ " << voxel_key << '\n';
      errors << "  expected: " << expected_occupancy << '\n';
      errors << "  actual: " << occupancy.data() << '\n';
    }

    return true;
  };

  ohm::walkSegmentKeys(ohm::LineWalkContext(map, visit_function), rays[0], rays[1]);

  errors.flush();
  if (!errors.str().empty())
  {
    FAIL() << errors.str();
  }
}

TEST(SecondarySample, DualReturnRay)
{
  // Simulate a lidar dual return scenario.
  std::ostringstream errors;
  // Test the behaviour of the ray flag kRfExcludeOrigin, fundamental to treating dual returns correctly.
  ohm::OccupancyMap map(0.1);

  // Offset the map so that (0, 0, 0) is a voxel centre.
  map.setOrigin(glm::dvec3(0.5 * map.resolution()));

  // Build rays. First ray is the "first return". The second ray is a "dual return" which starts from the sample point
  // of the first ray.
  const std::vector<glm::dvec3> rays =           //
    {                                            //
      glm::dvec3(0, 0, 0), glm::dvec3(1, 0, 0),  //
      glm::dvec3(1, 0, 0), glm::dvec3(2, 0, 0)
    };

  // Add the samples to map, excluding the origin. We lose something here for the first ray as the origin voxel is not
  // cleared, but we also do not try clear the origin of the second return.
  ohm::RayMapperOccupancy occupancy_mapper(&map);
  occupancy_mapper.integrateRays(rays.data(), rays.size(), nullptr, nullptr, ohm::kRfExcludeOrigin);
  ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  ASSERT_TRUE(occupancy.isLayerValid());
  const auto visit_function = [&errors, &map, &occupancy, &rays](const ohm::Key &voxel_key, double enter_range,
                                                                 double exit_range) -> bool {
    (void)enter_range;
    (void)exit_range;

    occupancy.setKey(voxel_key);
    if (!occupancy.isValid())
    {
      errors << "occupancy voxel not valid\n";
      return false;
    }

    // Ensure voxel at (0, 0, 0) is still unobserved.
    float expected_occupancy = {};
    if (voxel_key == map.voxelKey(rays[0]))
    {
      // First voxel in the ray.
      expected_occupancy = ohm::unobservedOccupancyValue();
    }
    else if (voxel_key == map.voxelKey(rays[1]) || voxel_key == map.voxelKey(rays[3]))
    {
      // One of the return samples.
      expected_occupancy = map.hitValue();
    }
    else
    {
      // Intermediary voxel.
      expected_occupancy = map.missValue();
    }

    if (occupancy.data() != expected_occupancy)
    {
      errors << "occupancy mismatch @ " << voxel_key << '\n';
      errors << "  expected: " << expected_occupancy << '\n';
      errors << "  actual: " << occupancy.data() << '\n';
    }

    return true;
  };

  // Walk the entire ray from "sensor" to "second return".
  ohm::walkSegmentKeys(ohm::LineWalkContext(map, visit_function), rays[0], rays[3]);

  errors.flush();
  if (!errors.str().empty())
  {
    FAIL() << errors.str();
  }
}

TEST(SecondarySample, Stats)
{
  // Validate the stats collected for secondary samples such as for lidar dual returns.
  ohm::OccupancyMap map(0.1, ohm::MapFlag::kSecondarySample);

  // Offset the map so that (0, 0, 0) is a voxel centre.
  map.setOrigin(glm::dvec3(0.5 * map.resolution()));

  // Build a set of "secondary returns". We treat the origin as the first return, and progressively shift the first
  // sample along voxel centres for voxels along the line (0, 0, 0) -> (1, 0, 0). The secondary sample is always at
  // (1, 0, 0).
  std::vector<glm::dvec3> rays;

  ohm::Key key(0, 0, 0, 0, 0, 0);
  const int sample_count = 10;
  for (int i = 0; i < sample_count; ++i)
  {
    rays.emplace_back(glm::dvec3(i * map.resolution(), 0, 0));
    rays.emplace_back(glm::dvec3(1, 0, 0));
  }

  // Work out our expected mean distance from first to second sample and expected standard deviation.
  double expected_mean = 0;
  double expected_std_dev = 0;
  for (size_t i = 0; i < rays.size(); i += 2)
  {
    const double distance = glm::length(rays[i + 1] - rays[i]);
    // Calculate total distance
    expected_mean += distance;
  }
  // Finalise mean distance.
  expected_mean /= sample_count;

  // Calculate variance.
  for (size_t i = 0; i < rays.size(); i += 2)
  {
    const double distance = glm::length(rays[i + 1] - rays[i]);
    const double deviation = distance - expected_mean;
    expected_std_dev += deviation * deviation;
  }

  // Finalise the standard deviation
  expected_std_dev = std::sqrt(expected_std_dev / sample_count);

  // Use the secondary sample mapper.
  ohm::RayMapperSecondarySample sample_mapper(&map);
  ASSERT_TRUE(sample_mapper.valid());
  sample_mapper.integrateRays(rays.data(), rays.size());

  // Check the sample voxel stats.
  ohm::Voxel<const ohm::VoxelSecondarySample> secondary_sample_voxel(
    &map, map.layout().layerIndex(ohm::default_layer::secondarySamplesLayerName()), map.voxelKey(rays[1]));
  ASSERT_TRUE(secondary_sample_voxel.isValid());
  const ohm::VoxelSecondarySample secondary_sample = secondary_sample_voxel.data();

  const double actual_mean = secondarySampleRangeMean(secondary_sample);
  const double actual_std_dev = secondarySampleRangeStdDev(secondary_sample);

  EXPECT_EQ(secondary_sample.count, sample_count);
  EXPECT_NEAR(actual_mean, expected_mean, 1e-2f);
  EXPECT_NEAR(actual_std_dev, expected_std_dev, 1e-2f);
}
}  // namespace sampletests
