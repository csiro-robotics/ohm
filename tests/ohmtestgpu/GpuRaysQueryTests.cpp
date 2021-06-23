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

#include <ohmutil/OhmUtil.h>

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
    // Scoped to ensure the query_gpu releases GPU resources before teh occupancy map - specifically the map's
    // GpuCache.
    ohm::RaysQuery query_cpu;
    ohm::RaysQueryGpu query_gpu;
    query_cpu.setMap(&map);
    query_gpu.setMap(&map);
    bool first_iteration = true;

    for (const auto ray_count : ray_counts)
    {
      // Add the additional rays (random).
      while (rays.size() < ray_count * 2)
      {
        rays.emplace_back(glm::dvec3(0.0));
        rays.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
      }

      // (Re)build the map with the new points.
      map.clear();
      ohm::GpuMap gpu_map(&map);

      gpu_map.integrateRays(rays.data(), rays.size());
      gpu_map.syncVoxels();

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
      for (size_t i = 0; i < query_cpu.numberOfResults(); ++i)
      {
        EXPECT_NEAR(ranges_cpu[i], ranges_gpu[i], 1e-4f) << "[" << i << "]";
        // We can get a fair amount of deviation in the volume due to floating point error on the GPU - it's single
        // precision.
        EXPECT_NEAR(unobserved_volumes_cpu[i], unobserved_volumes_gpu[i], 2.5 * resolution) << "[" << i << "]";
        EXPECT_EQ(terminal_types_cpu[i], terminal_types_gpu[i]) << "[" << i << "]";
      }

      std::cout << "Rays: " << query_cpu.numberOfRays() << " cpu: " << (cpu_time_end - cpu_time_start)
                << " gpu: " << (gpu_time_end - gpu_time_start) << std::endl;

      // Hard reset to clear the rays
      query_cpu.reset(true);
      query_gpu.reset(true);
    }
  }
}
}  // namespace raysquerytests
