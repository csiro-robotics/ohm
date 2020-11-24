// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohmgpu/LineKeysQueryGpu.h>
// #include <ohm/OccupancyType.h>

#include <ohmtools/OhmGen.h>
#include <ohmutil/GlmStream.h>
#include <ohmutil/OhmUtil.h>

#include <chrono>
#include <random>

#include <gtest/gtest.h>

namespace linekeys
{
typedef std::chrono::high_resolution_clock TimingClock;

void compareResults(const ohm::LineKeysQuery &query, const ohm::LineKeysQuery &reference)
{
  EXPECT_EQ(query.numberOfResults(), reference.numberOfResults());
  const size_t result_count = std::min(query.numberOfResults(), reference.numberOfResults());

  const ohm::OccupancyMap &map = *query.map();
  ohm::Key key, key_ref;
  size_t index_offset, index_offset_ref;
  size_t key_count, key_count_ref;
  size_t compare_key_count;
  bool line_ok;
  const int allow_voxel_deviation = 2;
  for (size_t r = 0; r < result_count; ++r)
  {
    line_ok = true;
    index_offset = query.resultIndices()[r];
    key_count = query.resultCounts()[r];
    index_offset_ref = reference.resultIndices()[r];
    key_count_ref = reference.resultCounts()[r];

    // Result count may differ by one for floating point error.
    if (key_count >= key_count_ref)
    {
      EXPECT_LE(key_count - key_count_ref, 1);
      line_ok = key_count - key_count_ref <= 1;
    }
    else
    {
      EXPECT_LE(key_count_ref - key_count, 1);
      line_ok = key_count_ref - key_count <= 1;
    }

    compare_key_count = std::min(key_count, key_count_ref);

    // First and last keys must match exactly (start and end voxels).
    if (compare_key_count)
    {
      EXPECT_EQ(query.intersectedVoxels()[index_offset], reference.intersectedVoxels()[index_offset_ref]);
      EXPECT_EQ(query.intersectedVoxels()[index_offset + key_count - 1],
                reference.intersectedVoxels()[index_offset_ref + key_count_ref - 1]);
    }

    // Start at index 1. Already checked the start voxel.
    for (size_t k = 1; k < compare_key_count; ++k)
    {
      bool keys_ok = true;
      // The comparison of the voxels must account for differences in precision on CPU (double) and GPU (single).
      // We allow line keys to diverge by one voxel.
      key = query.intersectedVoxels()[index_offset + k];
      key_ref = reference.intersectedVoxels()[index_offset_ref + k];

      if (key != key_ref)
      {
        // Keys are not equal. Ensure we are only one voxel off.
        if (key.regionKey() == key_ref.regionKey())
        {
          // In the same region. Ensure we are only one voxel off.
          for (int a = 0; a < 3; ++a)
          {
            if (key.localKey()[a] != key_ref.localKey()[a])
            {
              // Divergent axis. We expect only a single voxel difference.
              const bool allowed_voxel_shift =
                std::abs(key.localKey()[a] - key_ref.localKey()[a]) <= allow_voxel_deviation;
              keys_ok = keys_ok && allowed_voxel_shift;
              EXPECT_LE(std::abs(key.localKey()[a] - key_ref.localKey()[a]), allow_voxel_deviation);
            }
          }
        }
        else
        {
          // Region change.
          for (int a = 0; a < 3; ++a)
          {
            if (key.regionKey()[a] != key_ref.regionKey()[a])
            {
              // Divergent axis. We expect only a single voxel difference.
              const bool single_region_shift = std::abs(key.regionKey()[a] - key_ref.regionKey()[a]) <= 1;
              keys_ok = keys_ok && single_region_shift;
              EXPECT_LE(std::abs(key.regionKey()[a] - key_ref.regionKey()[a]), 1);
            }
          }

          if (keys_ok)
          {
            // Check the local key change.
            for (int a = 0; a < 3; ++a)
            {
              if (key.localKey()[a] != key_ref.localKey()[a])
              {
                // Divergent axis. We expect one voxel to be zero and the other to be the maximum of
                // the next region.
                bool allowed_voxel_shift =
                  key.localKey()[a] == 0 &&
                    key_ref.localKey()[a] == map.regionVoxelDimensions()[a] - allow_voxel_deviation ||
                  key_ref.localKey()[a] == 0 &&
                    key.localKey()[a] == map.regionVoxelDimensions()[a] - allow_voxel_deviation ||
                  std::abs(key.regionKey()[a] - key_ref.regionKey()[a]) <= allow_voxel_deviation;
                EXPECT_TRUE(allowed_voxel_shift);
                if (!allowed_voxel_shift)
                {
                  std::cout << "a: " << a << std::endl;
                  // EXPECT_EQ((int)key.localKey()[a], 0);
                  // EXPECT_EQ((int)keyB.localKey()[a], 0);
                  // EXPECT_EQ((int)keyB.localKey()[a], map.regionVoxelDimensions()[a] - 1);
                  // EXPECT_EQ((int)key.localKey()[a], map.regionVoxelDimensions()[a] - 1);
                }
                keys_ok = keys_ok && allowed_voxel_shift;
              }
            }
          }
        }

        if (!keys_ok)
        {
          std::cout << "More than one voxel difference : " << key << " vs. " << key_ref << std::endl;
          line_ok = false;
        }
      }
    }

    if (!line_ok)
    {
      std::cout << std::setprecision(20);
      std::cout << "Failed with line [" << r << "]: " << (query.rays()[r * 2 + 0]) << " to "
                << (query.rays()[r * 2 + 1]) << std::endl;
      EXPECT_TRUE(line_ok);
    }
  }
}


void dumpLines(const ohm::LineKeysQuery &query)
{
  const size_t result_count = query.numberOfResults();
  size_t index_offset;
  size_t key_count;
  ohm::Key key;

  for (size_t r = 0; r < result_count; ++r)
  {
    index_offset = query.resultIndices()[r];
    key_count = query.resultCounts()[r];

    std::cout << r << " line length " << key_count << '\n';
    for (size_t k = 0; k < key_count; ++k)
    {
      key = query.intersectedVoxels()[index_offset + k];
      std::cout << "  " << k << ": " << key << '\n';
    }
  }
}


TEST(LineKeys, QueryGpu)
{
  const double query_half_extents = 10.0;
  const int query_count = 10000;
  const int iteration_count = 1;
  ohm::OccupancyMap map(0.25);

  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> rand(-query_half_extents, query_half_extents);
  std::vector<glm::dvec3> line_points;

  // No need to populate the map. We are just calculating ray keys.

  // Full GPU evaluation query.
  ohm::LineKeysQueryGpu gpu_query(ohm::kQfGpuEvaluate | ohm::kQfNoCache);
  // Allow using cached GPU results query.
  ohm::LineKeysQueryGpu gpu_query2(ohm::kQfGpuEvaluate);
  // CPU query.
  ohm::LineKeysQuery cpu_query;

  gpu_query.setMap(&map);
  gpu_query2.setMap(&map);
  cpu_query.setMap(&map);

#if 1
  for (int i = 0; i < query_count; ++i)
  {
    line_points.push_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
    line_points.push_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
  }
#else   // #
  // The following line highlights a deviation between CPU and GPU algorithms. If you push these points
  // as dvec3, then there is a discrepancy between the CPU/GPU results.
  line_points.push_back(glm::vec3(8.249521446748637743, 1.6640723158759520572, 15.084516018081700395));
  line_points.push_back(glm::vec3(11.186323857973910378, -14.315011276937029905, 6.2049214437162198976));
#endif  // #
  gpu_query.setRays(line_points.data(), line_points.size());
  gpu_query2.setRays(line_points.data(), line_points.size());
  cpu_query.setRays(line_points.data(), line_points.size());

  // Run the initial queries now so we don't measure initialisation and memory allocation and overheads.
  gpu_query.execute();
  gpu_query2.execute();
  cpu_query.execute();

  ohm::KeyList keys;
  const size_t line_count = line_points.size() / 2;
  for (int i = 0; i < iteration_count; ++i)
  {
    std::cout << "Calculating " << line_points.size() / 2 << " line segments.\n";
    std::cout << "GPU  " << std::flush;
    const auto gpu_start = TimingClock::now();
    bool gpu_exec_ok = gpu_query.execute();
    const auto gpu_end = TimingClock::now();
    const auto gpu_duration = gpu_end - gpu_start;
    const auto gpu_per_line = gpu_duration / line_count;
    ohm::util::logDuration(std::cout, gpu_duration);
    std::cout << " ";
    ohm::util::logDuration(std::cout, gpu_per_line);
    std::cout << " per line" << std::endl;
    EXPECT_TRUE(gpu_exec_ok);

    std::cout << "Calculating " << line_points.size() / 2 << " line segments.\n";
    std::cout << "GPU (cached)  " << std::flush;
    const auto gpu_start2 = TimingClock::now();
    bool gpu_exec_ok2 = gpu_query2.execute();
    const auto gpu_end2 = TimingClock::now();
    const auto gpu_duration2 = gpu_end2 - gpu_start2;
    const auto gpu_per_line2 = gpu_duration / line_count;
    ohm::util::logDuration(std::cout, gpu_duration2);
    std::cout << " ";
    ohm::util::logDuration(std::cout, gpu_per_line2);
    std::cout << " per line" << std::endl;
    EXPECT_TRUE(gpu_exec_ok2);

    std::cout << "CPU  " << std::flush;
    auto cpu_start = TimingClock::now();
    bool cpu_exec_ok = cpu_query.execute();
    auto cpu_end = TimingClock::now();
    auto cpu_duration = cpu_end - cpu_start;
    auto cpu_per_line = cpu_duration / line_count;
    ohm::util::logDuration(std::cout, cpu_duration);
    std::cout << " ";
    ohm::util::logDuration(std::cout, cpu_per_line);
    std::cout << " per line" << std::endl;
    EXPECT_TRUE(cpu_exec_ok);

    std::cout << "CPU2 " << std::flush;
    cpu_start = TimingClock::now();
    for (size_t j = 0; j < line_count; ++j)
    {
      map.calculateSegmentKeys(keys, line_points[j * 2 + 0], line_points[j * 2 + 1], true);
    }
    cpu_end = TimingClock::now();
    cpu_duration = cpu_end - cpu_start;
    cpu_per_line = cpu_duration / line_count;
    ohm::util::logDuration(std::cout, cpu_duration);
    std::cout << " ";
    ohm::util::logDuration(std::cout, cpu_per_line);
    std::cout << " per line" << std::endl;

    // std::cout << "GPU:\n";
    // dumpLines(gpuQuery);
    // std::cout << "CPU:\n";
    // dumpLines(cpuQuery);
    compareResults(gpu_query, cpu_query);
    // Compare cached GPU query against the original.
    compareResults(gpu_query, gpu_query2);
  }
}
}  // namespace linekeys
