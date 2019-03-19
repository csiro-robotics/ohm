// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohmgpu/ClearanceProcess.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohmgpu/LineQueryGpu.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohmgpu/OhmGpu.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>
#include <ohmutil/GlmStream.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

using namespace ohm;

namespace linequerytests
{
  typedef std::chrono::high_resolution_clock TimingClock;

  void sparseMap(OccupancyMap &map, bool add_hit = true)
  {
    ohmgen::fillMapWithEmptySpace(map, -64, -64, -64, 63, 63, 63);
    Key key = map.voxelKey(glm::dvec3(0.5 * map.resolution()));
    VoxelConst voxel = map.voxel(key);
    if (add_hit)
    {
      for (int i = 0; i < 1; ++i)
      {
        voxel = map.integrateHit(glm::dvec3(0));
      }
      EXPECT_TRUE(voxel.value() >= 0);
    }
  }

  void lineQueryTest(OccupancyMap &map, bool gpu)
  {
    const double map_res = map.resolution();
    LineQueryGpu query(map, glm::dvec3(0) - glm::dvec3(map_res), glm::dvec3(0), 2.0f,
                    LineQuery::kDefaultFlags | kQfNearestResult);
    query.setStartPoint(glm::dvec3(-2, 0, 0));
    query.setEndPoint(glm::dvec3(2, 0, 0));
    if (gpu)
    {
      query.setQueryFlags(kQfNearestResult | kQfGpuEvaluate);
      query.setQueryFlags(kQfNearestResult | kQfGpuEvaluate);
    }

    const bool exec_ok = query.execute();
    ASSERT_TRUE(exec_ok);

    ASSERT_GT(query.numberOfResults(), 0);
    EXPECT_EQ(query.numberOfResults(), 1);

    std::cout << "Nearest result: " << query.intersectedVoxels()[0] << " @ " << query.ranges()[0] << std::endl;

    EXPECT_EQ(query.intersectedVoxels()[0], map.voxelKey(glm::dvec3(0)))
      << query.intersectedVoxels()[0] << " != " << map.voxelKey(glm::dvec3(0));
    EXPECT_EQ(query.ranges()[0], 0);

    std::string file_name("line-query-");
    file_name += (gpu) ? "gpu" : "cpu";
    file_name += ".ohm";
    ohm::save(file_name.c_str(), map);
  }

  TEST(LineQuery, Gpu)
  {
    OccupancyMap map(0.1);
    sparseMap(map);
    lineQueryTest(map, true);
  }

  TEST(LineQuery, CpuVsGpuSimple)
  {
    OccupancyMap map(0.1);
    sparseMap(map);

    const double map_res = map.resolution();
    LineQuery cpu_query(map, glm::dvec3(0) - glm::dvec3(map_res), glm::dvec3(0), 2.0f);
    cpu_query.setStartPoint(glm::dvec3(-5, 0, 0));
    cpu_query.setEndPoint(glm::dvec3(5, 0, 0));

    LineQueryGpu gpu_query(map, glm::dvec3(0) - glm::dvec3(map_res), glm::dvec3(0), 2.0f, kQfGpuEvaluate);
    gpu_query.setStartPoint(cpu_query.startPoint());
    gpu_query.setEndPoint(cpu_query.endPoint());

    cpu_query.execute();
    gpu_query.execute();

    EXPECT_EQ(cpu_query.numberOfResults(), gpu_query.numberOfResults());
    const size_t result_count = std::min(cpu_query.numberOfResults(), gpu_query.numberOfResults());
    for (size_t i = 0; i < result_count; ++i)
    {
      EXPECT_NEAR(cpu_query.ranges()[i], gpu_query.ranges()[i], 1e-6f);
    }
  }

  TEST(LineQuery, CpuVsGpu)
  {
    Profile profile;
    const float boundary_distance = 5.0f;
    OccupancyMap map(0.1);

    // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
    ohmgen::boxRoom(map, glm::dvec3(-boundary_distance), glm::dvec3(boundary_distance), 3);

    // Run the query and compare CPU/GPU results.
    LineQuery cpu_query(map, glm::dvec3(0), glm::dvec3(0), 2.0f);
    LineQueryGpu gpu_query(map, glm::dvec3(0), glm::dvec3(0), 2.0f, kQfGpuEvaluate);
    std::mt19937 rand_engine;
    std::uniform_real_distribution<double> rand(-1.1 * boundary_distance, 1.1 * boundary_distance);

    // Generate line set to test.
    std::vector<glm::dvec3> line_points;
    // Seed the query with a known line(s).
    line_points.push_back(glm::dvec3(-0.5f * boundary_distance, -0.25f * boundary_distance, 0.25f * boundary_distance));
    line_points.push_back(glm::dvec3(1.2f * boundary_distance));
    // linePoints.push_back(glm::dvec3(-1.2f * boundaryDistance, 0, 0));
    // linePoints.push_back(glm::dvec3(1.2f * boundaryDistance, 0, 0));

    const int query_iterations = 50;
    // const int queryIterations = 1;
    // Create new random line query points to meet the hard coded queryIterations.
    while (line_points.size() < query_iterations * 2)
    {
      line_points.push_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
    }

    // Save data for debugging (optional)
    bool save_clouds = true;
    if (save_clouds)
    {
      ohmtools::saveCloud("boundary.ply", map);
    }

#if 0
    // Precalculate clearance values for the GPU map (using GPU).
    ClearanceProcess clearances(gpuQuery.searchRadius(), kQfGpuEvaluate);
    // Calculate for the whole map.
    std::cout << "Clearance Map: " << std::flush;
    auto clearanceStartTime = timing_clock::now();
    ProfileMarker gpuClearance("GPU Clearance", &profile);
    clearances.update(map, std::numeric_limits<float>::infinity());
    gpuClearance.end();
    auto clearanceElapsed = timing_clock::now() - clearanceStartTime;
    std::cout << clearanceElapsed << std::endl;
#endif  // #

    std::cout << "Compare results from " << query_iterations << " line queries." << std::endl;
    for (int i = 0; i < query_iterations; ++i)
    {
      // Setup query.
      cpu_query.setStartPoint(line_points[i * 2 + 0]);
      cpu_query.setEndPoint(line_points[i * 2 + 1]);
      gpu_query.setStartPoint(cpu_query.startPoint());
      gpu_query.setEndPoint(cpu_query.endPoint());

      std::cout << std::setprecision(4) << "Line: (" << cpu_query.startPoint().x << ", " << cpu_query.startPoint().y
                << ", " << cpu_query.startPoint().z << ")->(" << cpu_query.endPoint().x << ", "
                << cpu_query.endPoint().y << ", " << cpu_query.endPoint().z << ")" << std::endl;

      // Run CPU query
      std::cout << "CPU: " << std::flush;
      auto start_time = TimingClock::now();
      ProfileMarker mark_cpu("CPU", &profile);
      cpu_query.execute();
      mark_cpu.end();
      auto exec_time = TimingClock::now() - start_time;
      std::cout << exec_time << std::endl;

      if (save_clouds)
      {
        ohmtools::saveQueryCloud("boundary-cpu-line.ply", map, cpu_query, cpu_query.searchRadius());
      }

      // Run GPU query.
      std::cout << "GPU: " << std::flush;

      start_time = TimingClock::now();
      ProfileMarker mark_gpu("GPU", &profile);
      gpu_query.execute();
      mark_gpu.end();
      exec_time = TimingClock::now() - start_time;
      std::cout << exec_time << std::endl;

      if (save_clouds)
      {
        ohmtools::saveQueryCloud("boundary-gpu-line.ply", map, gpu_query, gpu_query.searchRadius());
        glm::dvec3 min_ext = gpu_query.startPoint();
        glm::dvec3 max_ext = min_ext;
        min_ext.x = std::min(min_ext.x, gpu_query.endPoint().x);
        min_ext.y = std::min(min_ext.y, gpu_query.endPoint().y);
        min_ext.z = std::min(min_ext.z, gpu_query.endPoint().z);
        max_ext.x = std::max(max_ext.x, gpu_query.endPoint().x);
        max_ext.y = std::max(max_ext.y, gpu_query.endPoint().y);
        max_ext.z = std::max(max_ext.z, gpu_query.endPoint().z);
        ohmtools::saveClearanceCloud("boundary-gpu-region.ply", map, min_ext, max_ext, gpu_query.searchRadius());
      }

      // Compare results.
      start_time = TimingClock::now();
      EXPECT_EQ(cpu_query.numberOfResults(), gpu_query.numberOfResults());
      const size_t result_count = std::min(cpu_query.numberOfResults(), gpu_query.numberOfResults());
      for (size_t i = 0; i < result_count; ++i)
      {
        EXPECT_EQ(cpu_query.intersectedVoxels()[i], gpu_query.intersectedVoxels()[i]);
        EXPECT_NEAR(cpu_query.ranges()[i], gpu_query.ranges()[i], float(0.5 * map.resolution()));
        if (std::abs(cpu_query.ranges()[i] - gpu_query.ranges()[i]) >= float(0.5 * map.resolution()))
        {
          auto voxel = gpu_query.intersectedVoxels()[i];
          std::cout << "@ voxel : " << voxel << ' ' << map.voxelCentreGlobal(voxel) << std::endl;
        }
      }
      exec_time = TimingClock::now() - start_time;
      std::cout << "Compared results in " << exec_time << std::endl;
    }
  }

  TEST(LineQuery, CpuVsGpuPerf)
  {
    Profile profile;
    OccupancyMap map(0.1);
    sparseMap(map);

    // For this test we make repeated iterations to ensure that GPU ends up being faster when it can precalculate
    // and cache results.
    // In this test, we also compare the results to ensure they are the same.
    const int iterations = 50;
    TimingClock::time_point start_time_cpu, end_time_cpu, start_time_gpu, end_time_gpu;
    LineQueryGpu query(map, glm::dvec3(0), glm::dvec3(0), 2.0f);
    std::vector<Key> cpu_keys;
    std::vector<float> cpu_ranges;

    query.setStartPoint(glm::dvec3(-5, 0, 0));
    query.setEndPoint(glm::dvec3(5, 0, 0));
    query.setQueryFlags(LineQuery::kDefaultFlags | kQfNearestResult);

    std::cout << "Execute " << iterations << " query iterations." << std::endl;

    // CPU execution.
    std::cout << "CPU " << std::flush;
    start_time_cpu = TimingClock::now();
    for (int i = 0; i < iterations; ++i)
    {
      ProfileMarker mark_cpu("CPU", &profile);
      query.execute();
    }
    end_time_cpu = TimingClock::now();

    auto cpu_total_duration = end_time_cpu - start_time_cpu;
    auto cpu_average_duration = cpu_total_duration / iterations;

    std::cout << cpu_total_duration << " average completion " << cpu_average_duration << std::endl;

    // Collect CPU results.
    cpu_keys.resize(query.numberOfResults());
    cpu_ranges.resize(query.numberOfResults());
    for (size_t i = 0; i < query.numberOfResults(); ++i)
    {
      cpu_keys[i] = query.intersectedVoxels()[i];
      cpu_ranges[i] = query.ranges()[i];
    }

    // Touch the map so that the GPU query won't use values from the last iteration.
    map.touch();

    // GPU execution.
    query.reset();  // Hard reset.
    query.setQueryFlags(kQfNearestResult | ohm::kQfGpuEvaluate);

    std::cout << "GPU " << std::flush;
    start_time_gpu = TimingClock::now();
    for (int i = 0; i < iterations; ++i)
    {
      ProfileMarker mark_gpu("GPU", &profile);
      query.execute();
    }
    end_time_gpu = TimingClock::now();

    auto gpu_total_duration = end_time_gpu - start_time_gpu;
    auto gpu_average_duration = gpu_total_duration / iterations;
    std::cout << gpu_total_duration << " average completion " << gpu_average_duration << std::endl;

    // Compare results.
    EXPECT_EQ(cpu_keys.size(), query.numberOfResults());
    const size_t result_count = std::min(cpu_keys.size(), query.numberOfResults());
    // GPU has algorithmic differences which result in slightly different ranges at time.
    // Allow a tolerance for range comparison.
    const float ranges_tolerance = 0.5f * float(map.resolution());
    for (size_t i = 0; i < result_count; ++i)
    {
      EXPECT_EQ(cpu_keys[i], query.intersectedVoxels()[i]);
      if (std::signbit(cpu_ranges[i]) != std::signbit(query.ranges()[i]))
      {
        std::cerr << "Range sign mismatch for " << query.intersectedVoxels()[i] << std::endl;
        std::cerr << "CPU: " << cpu_ranges[i] << " GPU: " << query.ranges()[i] << std::endl;
      }
      EXPECT_NEAR(cpu_ranges[i], query.ranges()[i], ranges_tolerance);
    }

    // Validate that the GPU queries are faster (only with repeated iterations though).

    // Average out the timing to per iteration time.

    EXPECT_LT(gpu_total_duration.count(), cpu_total_duration.count());
    EXPECT_LT(gpu_average_duration.count(), cpu_average_duration.count());
  }
}  // namespace linequerytests
