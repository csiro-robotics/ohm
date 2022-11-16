// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohmgpu/RaysQueryGpu.h>

#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/QueryFlag.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/VoxelOccupancy.h>

#include <logutil/LogUtil.h>
#include <ohmutil/GlmStream.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

namespace raysquerytests
{
TEST(RaysQuery, Gpu)
{
  // Note this is a comparative test to ensure GPU results match CPU. The algorithm validation occurs in
  // RaysQueryTests.cpp (the CPU equivalent test).
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
  // Scoped to ensure the query_gpu releases GPU resources before teh occupancy map - specifically the map's GpuCache.
  {
    ohm::RaysQuery query_cpu;
    ohm::RaysQueryGpu query_gpu;
    query_cpu.setMap(&map);
    query_gpu.setMap(&map);

    // Iterations:
    // 0 : samples only in the map, no free space.
    // 1 : samples and free space in the map.
    // 2 : as 2, query_gpu running CPU code.
    const size_t max_iterations = 3;
    for (size_t iteration = 0; iteration < max_iterations; ++iteration)
    {
      if (iteration == 2)
      {
        // For the 3rd iteration, run the GPU query in CPU mode instead.
        query_gpu.setQueryFlags((~ohm::kQfGpu) & query_gpu.queryFlags());
      }
      else
      {
        EXPECT_EQ(query_gpu.queryFlags() & ohm::kQfGpu, ohm::kQfGpu);
      }

      // Scale the rays for lookup.
      for (size_t i = 0; i < rays.size(); i += 2)
      {
        query_cpu.addRay(rays[i], rays[i + 1] * query_scale[iteration]);
        query_gpu.addRay(rays[i], rays[i + 1] * query_scale[iteration]);
      }

      // Make the query.
      query_cpu.execute();
      query_gpu.execute();

      // Compare results.
      ASSERT_EQ(query_cpu.numberOfResults(), rays.size() / 2);
      ASSERT_EQ(query_gpu.numberOfResults(), rays.size() / 2);
      const double *ranges_cpu = query_cpu.ranges();
      const double *unobserved_volumes_cpu = query_cpu.unobservedVolumes();
      const ohm::OccupancyType *terminal_types_cpu = query_cpu.terminalOccupancyTypes();
      const double *ranges_gpu = query_gpu.ranges();
      const double *unobserved_volumes_gpu = query_gpu.unobservedVolumes();
      const ohm::OccupancyType *terminal_types_gpu = query_gpu.terminalOccupancyTypes();
      for (size_t i = 0; i < query_cpu.numberOfResults(); ++i)
      {
        EXPECT_NEAR(ranges_cpu[i], ranges_gpu[i], 1e-4f) << "[" << i << "]";
        // We can get a fair amount of deviation in the volume due to floating point error on the GPU - it's single
        // precision.
        EXPECT_NEAR(unobserved_volumes_cpu[i], unobserved_volumes_gpu[i], 2.5 * resolution) << "[" << i << "]";
        EXPECT_EQ(terminal_types_cpu[i], terminal_types_gpu[i]) << "[" << i << "]";
      }

      // Now do a full ray integration and redo the comparison.
      if (iteration == 0)
      {
        mapper.integrateRays(rays.data(), rays.size());
      }

      // Hard reset to clear the rays
      query_cpu.reset(true);
      query_gpu.reset(true);
    }
  }
}

TEST(RaysQuery, CpuVsGpu)
{
  // Rays query timing test
  // Note this is a comparative test to ensure GPU results match CPU. The algorithm validation occurs in
  // RaysQueryTests.cpp (the CPU equivalent test).
  const double base_scale = 10.0;
  const double query_scale = 1.2;
  const double resolution = 0.1;
  const std ::vector<size_t> ray_counts = { 10, 100, 1000, 2000, 5000, 10000 };
  using Clock = std::chrono::high_resolution_clock;

  // Make some rays.
  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> rand(-base_scale, base_scale);
  std::vector<glm::dvec3> rays;
  // Pure occupancy map.
  ohm::OccupancyMap map(resolution, ohm::MapFlag::kNone);

  {
    // Scoped to ensure the query_gpu releases GPU resources before the occupancy map - specifically the map's
    // GpuCache.
    ohm::RaysQuery query_cpu;
    ohm::RaysQueryGpu query_gpu;
    query_cpu.setMap(&map);
    query_gpu.setMap(&map);
    bool first_iteration = true;

    // ohm::GpuMap gpu_map(&map);

    for (const auto ray_count : ray_counts)
    {
      rays.clear();
      // Add the additional rays (random).
      while (rays.size() < ray_count * 2)
      {
        rays.emplace_back(glm::dvec3(0.0));
        rays.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
      }

      // (Re)build the map with the new points.
      map.clear();

// FIXME(KS): There is something going wrong with synching the map back to CPU in the 10000 ray test.
// This is a sporadic failure and the reliably presents as 850 ray trace discrepancies between CPU and GPU. It is
// in fact the CPU has the wrong data for these rays. All other rays are fine. The pass rate is maybe 50/50.
#if 0
      // gpu_map.integrateRays(rays.data(), rays.size());
      // gpu_map.syncVoxels();
#else   // #
      ohm::RayMapperOccupancy mapper(&map);
      mapper.integrateRays(rays.data(), rays.size());
#endif  // #

      EXPECT_EQ(query_gpu.queryFlags() & ohm::kQfGpu, ohm::kQfGpu);

      // Add scaled the rays for lookup.
      for (size_t i = 0; i < rays.size(); i += 2)
      {
        query_cpu.addRay(rays[i], rays[i + 1] * query_scale);
        query_gpu.addRay(rays[i], rays[i + 1] * query_scale);
      }

      if (first_iteration)
      {
        // Prime the GpuRays query to ensure the GPU program is loaded.
        query_gpu.execute();
        first_iteration = false;
      }

      // Make the query.
      const auto cpu_time_start = Clock::now();
      query_cpu.execute();
      const auto cpu_time_end = Clock::now();
      const auto gpu_time_start = Clock::now();
      query_gpu.execute();
      const auto gpu_time_end = Clock::now();

      // Compare results.
      ASSERT_EQ(query_cpu.numberOfResults(), rays.size() / 2);
      ASSERT_EQ(query_gpu.numberOfResults(), rays.size() / 2);
      const double *ranges_cpu = query_cpu.ranges();
      const double *unobserved_volumes_cpu = query_cpu.unobservedVolumes();
      const ohm::OccupancyType *terminal_types_cpu = query_cpu.terminalOccupancyTypes();
      const double *ranges_gpu = query_gpu.ranges();
      const double *unobserved_volumes_gpu = query_gpu.unobservedVolumes();
      const ohm::OccupancyType *terminal_types_gpu = query_gpu.terminalOccupancyTypes();
      // Maximum failures is the less of 5 or a fifth of the ray count.
      // const unsigned allowed_mismatches = 0;
      const unsigned allowed_mismatches = std::min(5u, unsigned(ray_count) / 5u);
      unsigned mismatches = 0;
      for (size_t i = 0; i < query_cpu.numberOfResults(); ++i)
      {
        // Check the range delta.
        float range_epsilon = 1e-4f;
        if (std::abs(ranges_cpu[i] - ranges_gpu[i]) >= range_epsilon)
        {
          if (mismatches < allowed_mismatches)
          {
            range_epsilon = float(std::sqrt(3.0 * map.resolution() * map.resolution()));
          }
          else
          {
            const auto precision = std::cout.precision();
            std::cout << std::setprecision(17);
            std::cout << rays[i * 2 + 0] << " -> " << rays[i * 2 + 1] << " [" << mismatches << "]" << std::endl;
            std::cout << std::setprecision(precision);
          }
          ++mismatches;
        }

        EXPECT_NEAR(ranges_cpu[i], ranges_gpu[i], range_epsilon) << "[" << i << "]";
        // We can get a fair amount of deviation in the volume due to floating point error on the GPU - it's single
        // precision.
        EXPECT_NEAR(unobserved_volumes_cpu[i], unobserved_volumes_gpu[i], 2.5 * resolution) << "[" << i << "]";
        EXPECT_EQ(terminal_types_cpu[i], terminal_types_gpu[i]) << "[" << i << "]";
      }

      EXPECT_LE(mismatches, allowed_mismatches);

      std::cout << "Rays: " << query_cpu.numberOfRays() << " cpu: " << (cpu_time_end - cpu_time_start)
                << " gpu: " << (gpu_time_end - gpu_time_start) << std::endl;

      // Hard reset to clear the rays
      query_cpu.reset(true);
      query_gpu.reset(true);
    }
  }
}
}  // namespace raysquerytests
