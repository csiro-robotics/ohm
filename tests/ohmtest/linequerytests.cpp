// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancygpu.h"
#include "occupancykey.h"
#include "occupancykeylist.h"
#include "occupancylinequery.h"
#include "occupancymap.h"
#include "occupancytype.h"
#include "occupancyutil.h"
#include "ohmclearanceprocess.h"
#include "occupancymapserialise.h"

#include "ohmcloud.h"
#include "ohmgen.h"

#include "ohmutil.h"
#include "profile.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

using namespace ohm;
using namespace ohmutil;

namespace linequerytests
{
  typedef std::chrono::high_resolution_clock timing_clock;

  void sparseMap(OccupancyMap &map, bool addHit = true)
  {
    ohmgen::fillMapWithEmptySpace(map, -64, -64, -64, 63, 63, 63);
    OccupancyKey key = map.voxelKey(glm::dvec3(0.5 * map.resolution()));
    OccupancyNodeConst node = map.node(key);
    if (addHit)
    {
      for (int i = 0; i < 1; ++i)
      {
        node = map.integrateHit(glm::dvec3(0));
      }
      EXPECT_TRUE(node.value() >= 0);
    }
  }

  void lineQueryTest(OccupancyMap &map, bool gpu)
  {
    const double mapRes = map.resolution();
    LineQuery query(map, glm::dvec3(0) - glm::dvec3(mapRes), glm::dvec3(0), 2.0f, LineQuery::DefaultFlags | QF_NearestResult);
    query.setStartPoint(glm::dvec3(-2, 0, 0));
    query.setEndPoint(glm::dvec3(2, 0, 0));
    if (gpu)
    {
      query.setQueryFlags(QF_NearestResult | QF_GpuEvaluate);
      query.setQueryFlags(QF_NearestResult | QF_GpuEvaluate);
    }

    bool execOk = query.execute();
    ASSERT_TRUE(execOk);

    ASSERT_GT(query.numberOfResults(), 0);
    EXPECT_EQ(query.numberOfResults(), 1);

    std::cout << "Nearest result: " << query.intersectedVoxels()[0] << " @ " << query.ranges()[0] << std::endl;

    EXPECT_EQ(query.intersectedVoxels()[0], map.voxelKey(glm::dvec3(0)))
      << query.intersectedVoxels()[0] << " != " << map.voxelKey(glm::dvec3(0));
    EXPECT_EQ(query.ranges()[0], 0);

    std::string fileName("line-query-");
    fileName += (gpu) ? "gpu" : "cpu";
    fileName += ".ohm";
    ohm::save(fileName.c_str(), map);
  }

  TEST(LineQuery, Gpu)
  {
    OccupancyMap map(0.1);
    sparseMap(map);
    lineQueryTest(map, true);
  }

  TEST(LineQuery, Cpu)
  {
    OccupancyMap map(0.1);
    sparseMap(map);
    lineQueryTest(map, false);
  }

  TEST(LineQuery, CpuVsGpuSimple)
  {
    OccupancyMap map(0.1);
    sparseMap(map);

    const double mapRes = map.resolution();
    LineQuery cpuQuery(map, glm::dvec3(0) - glm::dvec3(mapRes), glm::dvec3(0), 2.0f);
    cpuQuery.setStartPoint(glm::dvec3(-5, 0, 0));
    cpuQuery.setEndPoint(glm::dvec3(5, 0, 0));

    LineQuery gpuQuery(map, glm::dvec3(0) - glm::dvec3(mapRes), glm::dvec3(0), 2.0f, QF_GpuEvaluate);
    gpuQuery.setStartPoint(cpuQuery.startPoint());
    gpuQuery.setEndPoint(cpuQuery.endPoint());

    cpuQuery.execute();
    gpuQuery.execute();

    EXPECT_EQ(cpuQuery.numberOfResults(), gpuQuery.numberOfResults());
    size_t resultCount = std::min(cpuQuery.numberOfResults(), gpuQuery.numberOfResults());
    for (size_t i = 0; i < resultCount; ++i)
    {
      EXPECT_NEAR(cpuQuery.ranges()[i], gpuQuery.ranges()[i], 1e-6f);
    }
  }

  TEST(LineQuery, CpuVsGpu)
  {
    Profile profile;
    const float boundaryDistance = 5.0f;
    OccupancyMap map(0.1);

    // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
    ohmgen::cubicRoom(map, boundaryDistance, 3);

    // Run the query and compare CPU/GPU results.
    LineQuery cpuQuery(map, glm::dvec3(0), glm::dvec3(0), 2.0f);
    LineQuery gpuQuery(map, glm::dvec3(0), glm::dvec3(0), 2.0f, QF_GpuEvaluate);
    std::mt19937 randEngine;
    std::uniform_real_distribution<double> rand(-1.1 * boundaryDistance, 1.1 * boundaryDistance);

    // Generate line set to test.
    std::vector<glm::dvec3> linePoints;
    // Seed the query with a known line(s).
    linePoints.push_back(glm::dvec3(-0.5f * boundaryDistance, -0.25f * boundaryDistance, 0.25f * boundaryDistance));
    linePoints.push_back(glm::dvec3(1.2f * boundaryDistance));
    //linePoints.push_back(glm::dvec3(-1.2f * boundaryDistance, 0, 0));
    //linePoints.push_back(glm::dvec3(1.2f * boundaryDistance, 0, 0));

    const int queryIterations = 50;
    // const int queryIterations = 1;
    // Create new random line query points to meet the hard coded queryIterations.
    while (linePoints.size() < queryIterations * 2)
    {
      linePoints.push_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
    }

    // Save data for debugging (optional)
    bool saveClouds = true;
    if (saveClouds)
    {
      ohmtools::saveCloud("boundary.ply", map);
    }

#if 0
    // Precalculate clearance values for the GPU map (using GPU).
    ClearanceProcess clearances(gpuQuery.searchRadius(), QF_GpuEvaluate);
    // Calculate for the whole map.
    std::cout << "Clearance Map: " << std::flush;
    auto clearanceStartTime = timing_clock::now();
    ProfileMarker gpuClearance("GPU Clearance", &profile);
    clearances.update(map, std::numeric_limits<float>::infinity());
    gpuClearance.end();
    auto clearanceElapsed = timing_clock::now() - clearanceStartTime;
    std::cout << clearanceElapsed << std::endl;
#endif // #

    std::cout << "Compare results from " << queryIterations << " line queries." << std::endl;
    for (int i = 0; i < queryIterations; ++i)
    {
      // Setup query.
      cpuQuery.setStartPoint(linePoints[i * 2 + 0]);
      cpuQuery.setEndPoint(linePoints[i * 2 + 1]);
      gpuQuery.setStartPoint(cpuQuery.startPoint());
      gpuQuery.setEndPoint(cpuQuery.endPoint());

      std::cout << std::setprecision(4) << "Line: (" << cpuQuery.startPoint().x << ", " << cpuQuery.startPoint().y
                << ", " << cpuQuery.startPoint().z << ")->(" << cpuQuery.endPoint().x << ", " << cpuQuery.endPoint().y
                << ", " << cpuQuery.endPoint().z << ")" << std::endl;

      // Run CPU query
      std::cout << "CPU: " << std::flush;
      auto startTime = timing_clock::now();
      ProfileMarker markCpu("CPU", &profile);
      cpuQuery.execute();
      markCpu.end();
      auto execTime = timing_clock::now() - startTime;
      std::cout << execTime << std::endl;

      if (saveClouds)
      {
        ohmtools::saveQueryCloud("boundary-cpu-line.ply", map, cpuQuery, cpuQuery.searchRadius());
      }

      // Run GPU query.
      std::cout << "GPU: " << std::flush;

      startTime = timing_clock::now();
      ProfileMarker markGpu("GPU", &profile);
      gpuQuery.execute();
      markGpu.end();
      execTime = timing_clock::now() - startTime;
      std::cout << execTime << std::endl;

      if (saveClouds)
      {
        ohmtools::saveQueryCloud("boundary-gpu-line.ply", map, gpuQuery, gpuQuery.searchRadius());
        glm::dvec3 minExt = gpuQuery.startPoint();
        glm::dvec3 maxExt = minExt;
        minExt.x = std::min(minExt.x, gpuQuery.endPoint().x);
        minExt.y = std::min(minExt.y, gpuQuery.endPoint().y);
        minExt.z = std::min(minExt.z, gpuQuery.endPoint().z);
        maxExt.x = std::max(maxExt.x, gpuQuery.endPoint().x);
        maxExt.y = std::max(maxExt.y, gpuQuery.endPoint().y);
        maxExt.z = std::max(maxExt.z, gpuQuery.endPoint().z);
        ohmtools::saveClearanceCloud("boundary-gpu-region.ply", map, minExt, maxExt, gpuQuery.searchRadius());
      }

      // Compare results.
      startTime = timing_clock::now();
      EXPECT_EQ(cpuQuery.numberOfResults(), gpuQuery.numberOfResults());
      size_t resultCount = std::min(cpuQuery.numberOfResults(), gpuQuery.numberOfResults());
      for (size_t i = 0; i < resultCount; ++i)
      {
        EXPECT_EQ(cpuQuery.intersectedVoxels()[i], gpuQuery.intersectedVoxels()[i]);
        EXPECT_NEAR(cpuQuery.ranges()[i], gpuQuery.ranges()[i], float(0.5 * map.resolution()));
        if (std::abs(cpuQuery.ranges()[i] - gpuQuery.ranges()[i]) >= float(0.5 * map.resolution()))
        {
          auto voxel = gpuQuery.intersectedVoxels()[i];
          std::cout << "@ voxel : " << voxel << ' ' << map.voxelCentreGlobal(voxel) << std::endl;
        }
      }
      execTime = timing_clock::now() - startTime;
      std::cout << "Compared results in " << timing_clock::now() - startTime << std::endl;
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
    timing_clock::time_point startTimeCpu, endTimeCpu, startTimeGpu, endTimeGpu;
    LineQuery query(map, glm::dvec3(0), glm::dvec3(0), 2.0f);
    std::vector<OccupancyKey> cpuKeys;
    std::vector<float> cpuRanges;

    query.setStartPoint(glm::dvec3(-5, 0, 0));
    query.setEndPoint(glm::dvec3(5, 0, 0));
    query.setQueryFlags(LineQuery::DefaultFlags | QF_NearestResult);

    std::cout << "Execute " << iterations << " query iterations." << std::endl;

    // CPU execution.
    std::cout << "CPU " << std::flush;
    startTimeCpu = timing_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
      ProfileMarker markCpu("CPU", &profile);
      query.execute();
    }
    endTimeCpu = timing_clock::now();

    auto cpuTotalDuration = endTimeCpu - startTimeCpu;
    auto cpuAverageDuration = cpuTotalDuration / iterations;

    std::cout << cpuTotalDuration << " average completion " << cpuAverageDuration << std::endl;

    // Collect CPU results.
    cpuKeys.resize(query.numberOfResults());
    cpuRanges.resize(query.numberOfResults());
    for (size_t i = 0; i < query.numberOfResults(); ++i)
    {
      cpuKeys[i] = query.intersectedVoxels()[i];
      cpuRanges[i] = query.ranges()[i];
    }

    // Touch the map so that the GPU query won't use values from the last iteration.
    map.touch();

    // GPU execution.
    query.reset();  // Hard reset.
    query.setQueryFlags(QF_NearestResult | ohm::QF_GpuEvaluate);

    std::cout << "GPU " << std::flush;
    startTimeGpu = timing_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
      ProfileMarker markGpu("GPU", &profile);
      query.execute();
    }
    endTimeGpu = timing_clock::now();

    auto gpuTotalDuration = endTimeGpu - startTimeGpu;
    auto gpuAverageDuration = gpuTotalDuration / iterations;
    std::cout << gpuTotalDuration << " average completion " << gpuAverageDuration << std::endl;

    // Compare results.
    EXPECT_EQ(cpuKeys.size(), query.numberOfResults());
    size_t resultCount = std::min(cpuKeys.size(), query.numberOfResults());
    // GPU has algorithmic differences which result in slightly different ranges at time.
    // Allow a tolerance for range comparison.
    const float rangesTolerance = 0.5f * float(map.resolution());
    for (size_t i = 0; i < resultCount; ++i)
    {
      EXPECT_EQ(cpuKeys[i], query.intersectedVoxels()[i]);
      if (std::signbit(cpuRanges[i]) != std::signbit(query.ranges()[i]))
      {
        std::cerr << "Range sign mismatch for " << query.intersectedVoxels()[i] << std::endl;
        std::cerr << "CPU: " << cpuRanges[i] << " GPU: " << query.ranges()[i] << std::endl;
      }
      EXPECT_NEAR(cpuRanges[i], query.ranges()[i], rangesTolerance);
    }

    // Validate that the GPU queries are faster (only with repeated iterations though).

    // Average out the timing to per iteration time.

    EXPECT_LT(gpuTotalDuration.count(), cpuTotalDuration.count());
    EXPECT_LT(gpuAverageDuration.count(), cpuAverageDuration.count());
  }
}
