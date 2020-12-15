// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include <ohm/OccupancyMap.h>
#include <ohm/RayPatternConical.h>
#include <ohm/VoxelData.h>

#include <ohm/ClearingPattern.h>

#include <array>
#include <cstddef>
#include <functional>

#include <gtest/gtest.h>

namespace ohmtestutil
{
/// Template function for testing the RayPattern integration for use with either a CPU or GPU map.
///
/// This test creates a series of four voxels of varying types then applys a single ray ClearingPattern using various
/// exclusion flags, and checks the results.
///
/// @param map The @c OccupancyMap being updated.
/// @param ray_map The map object to apply the @c ClearingPattern to. This is the @c OccupancyMap when updating using
/// CPU and the @c GpuMap when using GPU.
/// @param pre_validation_function Function to call before validation. For GPU, this should sync to CPU.
/// @param test_start_index Debug parameter used to choose the first test index to run.
/// @param test_end_index Debug parameter used to choose the last test index to run - should be set to 1 more than the
///   last test index to be run.
template <typename RayMap>
void RayPatternExcludeTest(ohm::OccupancyMap &map, RayMap &ray_map,
                           std::function<void()> pre_validation_func = std::function<void()>(),
                           size_t test_start_index = 0, size_t test_end_index = ~0u)
{
  // Setup the map. Must work with a 1.0m resolution for convenience.
  ASSERT_EQ(map.resolution(), 1.0);
  // Set the miss value to be greater magnitude than the hit value. This ensures a single clear ray will erase a hit.
  // Note the negative sign is required for a probability decrease.
  map.setMissValue(-1.1f * map.hitValue());
  // Offset by half a voxel so that integer coordinates approximately index voxel centres.
  map.setOrigin(glm::dvec3(0.5 * map.resolution()));
  // Disable voxel saturation
  map.setSaturateAtMinValue(false);
  map.setSaturateAtMaxValue(false);
  // Remove voxel value limits
  map.setMinVoxelValue(std::numeric_limits<float>::lowest());
  map.setMaxVoxelValue(std::numeric_limits<float>::max());

  // Configure a function object to reset the voxel values of interest.
  ohm::Voxel<float> occupancy(&map, map.layout().occupancyLayer());
  ASSERT_TRUE(occupancy.isLayerValid());

  // function object to call to reset the map state to:
  // - unobserved @ (0, 0, 0)
  // - free @ (1, 0, 0)
  // - occupied @ (2, 0, 0)
  const auto reset_map_func = [&map, &occupancy]  //
  {
    occupancy.setKey(map.voxelKey(glm::dvec3(0, 0, 0)));
    ASSERT_TRUE(occupancy.isValid());
    occupancy.write(ohm::unobservedOccupancyValue());

    occupancy.setKey(map.voxelKey(glm::dvec3(1, 0, 0)));
    ASSERT_TRUE(occupancy.isValid());
    occupancy.write(map.missValue());

    occupancy.setKey(map.voxelKey(glm::dvec3(2, 0, 0)));
    ASSERT_TRUE(occupancy.isValid());
    occupancy.write(map.hitValue());

    occupancy.setKey(map.voxelKey(glm::dvec3(3, 0, 0)));
    ASSERT_TRUE(occupancy.isValid());
    occupancy.write(map.hitValue());

    // Ensure the last write operation is committed. Particularly important when this test is used on a GpuMap as it
    // ensures the GpuCache recognises the CPU based change.
    occupancy.commit();
  };

  // Build a single ray pattern.
  ohm::RayPattern rays;
  rays.addRay(glm::dvec3(0, 0, 0), glm::dvec3(10, 0, 0));
  ohm::ClearingPattern pattern(&rays, false);

  // Setup the various test cases.
  struct TestData
  {
    /// Context used for error display.
    const char *context;
    /// @c RayFlag values to use when applying the pattern.
    unsigned ray_flags;
    /// Expected occupancy values along the ray.
    std::array<float, 4> expected_values;
  };

  const std::array<TestData, 5> tests =  //
    {                                    //
      TestData{
        "default flags",
        ohm::ClearingPattern::kDefaultRayFlags,
        { ohm::unobservedOccupancyValue(), map.missValue(), map.hitValue() + map.missValue(), map.hitValue() } },
      TestData{ "continue past occupied",
                ohm::ClearingPattern::kDefaultRayFlags & ~ohm::kRfStopOnFirstOccupied,
                { ohm::unobservedOccupancyValue(), map.missValue(), map.hitValue() + map.missValue(),
                  map.hitValue() + map.missValue() } },
      TestData{ "exclude unobserved",
                ohm::kRfEndPointAsFree | ohm::kRfExcludeUnobserved,
                { ohm::unobservedOccupancyValue(), 2.0f * map.missValue(), map.hitValue() + map.missValue(),
                  map.hitValue() + map.missValue() } },
      TestData{
        "exclude free",
        ohm::kRfEndPointAsFree | ohm::kRfExcludeFree,
        { map.missValue(), map.missValue(), map.hitValue() + map.missValue(), map.hitValue() + map.missValue() } },
      TestData{ "exclude occupied",
                ohm::kRfEndPointAsFree | ohm::kRfExcludeOccupied,
                { map.missValue(), 2.0f * map.missValue(), map.hitValue(), map.hitValue() } }
    };

  // Run the test data sets
  for (size_t i = std::max<size_t>(0u, test_start_index); i < std::min<size_t>(tests.size(), test_end_index); ++i)
  {
    const TestData &test = tests[i];
    // Reset the map.
    reset_map_func();
    // Set the ray flags.
    pattern.setRayFlags(test.ray_flags);
    // Apply the pattern.
    pattern.apply(&ray_map, glm::dmat4(1.0));
    if (pre_validation_func)
    {
      pre_validation_func();
    }
    // Check the results.
    for (size_t i = 0; i < test.expected_values.size(); ++i)
    {
      // Retrieve the corresponding voxel.
      occupancy.setKey(map.voxelKey(glm::dvec3(double(i), 0, 0)));
      ASSERT_TRUE(occupancy.isValid()) << test.context << " voxel " << i;
      if (occupancy.data() != ohm::unobservedOccupancyValue() ||
          test.expected_values[i] != ohm::unobservedOccupancyValue())
      {
        EXPECT_NEAR(occupancy.data(), test.expected_values[i], 1e-3f) << test.context << " voxel " << i;
      }
      else
      {
        // Handle unobserved == infinity
        EXPECT_EQ(occupancy.data(), test.expected_values[i]);
      }
    }
  }
}
}  // namespace ohmtestutil