// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/occupancykey.h>
#include <ohm/occupancykeylist.h>
#include <ohm/occupancylinekeysquery.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancyutil.h>
// #include <ohm/occupancytype.h>

#include <ohmutil/ohmutil.h>
#include <ohmtools/ohmgen.h>

#include <chrono>
#include <random>

#include <gtest/gtest.h>
#include <ohm/occupancylinekeysquery.h>

namespace linekeys
{
  typedef std::chrono::high_resolution_clock timing_clock;

  void compareResults(const ohm::LineKeysQuery &query, const ohm::LineKeysQuery &reference)
  {
    EXPECT_EQ(query.numberOfResults(), reference.numberOfResults());
    size_t resultCount = std::min(query.numberOfResults(), reference.numberOfResults());

    const ohm::OccupancyMap &map = *query.map();
    ohm::OccupancyKey key, keyRef;
    size_t indexOffset, indexOffsetRef;
    size_t keyCount, keyCountRef;
    size_t compareKeyCount;
    bool lineOk;
    bool failedOffsets = false;
    const int allowVoxelDeviation = 2;
    for (size_t r = 0; r < resultCount; ++r)
    {
      lineOk = true;
      indexOffset = query.resultIndices()[r];
      keyCount = query.resultCounts()[r];
      indexOffsetRef = reference.resultIndices()[r];
      keyCountRef = reference.resultCounts()[r];

      // Result count may differ by one for floating point error.
      if (keyCount >= keyCountRef)
      {
        EXPECT_LE(keyCount - keyCountRef, 1);
        lineOk = keyCount - keyCountRef <= 1;
      }
      else
      {
        EXPECT_LE(keyCountRef - keyCount, 1);
        lineOk = keyCountRef - keyCount <= 1;
      }

      compareKeyCount = std::min(keyCount, keyCountRef);

      // First and last keys must match exactly (start and end voxels).
      if (compareKeyCount)
      {
        EXPECT_EQ(query.intersectedVoxels()[indexOffset], reference.intersectedVoxels()[indexOffsetRef]);
        EXPECT_EQ(query.intersectedVoxels()[indexOffset + keyCount - 1], reference.intersectedVoxels()[indexOffsetRef + keyCountRef - 1]);
      }

      // Start at index 1. Already checked the start voxel.
      for (size_t k = 1; k < compareKeyCount; ++k)
      {
        bool keysOk = true;
        // The comparison of the voxels must account for differences in precision on CPU (double) and GPU (single).
        // We allow line keys to diverge by one voxel.
        key = query.intersectedVoxels()[indexOffset + k];
        keyRef = reference.intersectedVoxels()[indexOffsetRef + k];

        if (key != keyRef)
        {
          // Keys are not equal. Ensure we are only one voxel off.
          if (key.regionKey() == keyRef.regionKey())
          {
            // In the same region. Ensure we are only one voxel off.
            for (int a = 0; a < 3; ++a)
            {
              if (key.localKey()[a] != keyRef.localKey()[a])
              {
                // Divergent axis. We expect only a single voxel difference.
                bool allowedVoxelShift = std::abs(key.localKey()[a] - keyRef.localKey()[a]) <= allowVoxelDeviation;
                keysOk = keysOk && allowedVoxelShift;
                EXPECT_LE(std::abs(key.localKey()[a] - keyRef.localKey()[a]), allowVoxelDeviation);
              }
            }
          }
          else
          {
            // Region change.
            for (int a = 0; a < 3; ++a)
            {
              if (key.regionKey()[a] != keyRef.regionKey()[a])
              {
                // Divergent axis. We expect only a single voxel difference.
                bool singleRegionShift = std::abs(key.regionKey()[a] - keyRef.regionKey()[a]) <= 1;
                keysOk = keysOk && singleRegionShift;
                EXPECT_LE(std::abs(key.regionKey()[a] - keyRef.regionKey()[a]), 1);
              }
            }

            if (keysOk)
            {
              // Check the local key change.
              for (int a = 0; a < 3; ++a)
              {
                if (key.localKey()[a] != keyRef.localKey()[a])
                {
                  // Divergent axis. We expect one voxel to be zero and the other to be the maximum of
                  // the next region.
                  bool allowedVoxelShift = key.localKey()[a] == 0 && keyRef.localKey()[a] == map.regionVoxelDimensions()[a] - allowVoxelDeviation ||
                                           keyRef.localKey()[a] == 0 && key.localKey()[a] == map.regionVoxelDimensions()[a] - allowVoxelDeviation ||
                                           std::abs(key.regionKey()[a] - keyRef.regionKey()[a]) <= allowVoxelDeviation;
                  EXPECT_TRUE(allowedVoxelShift);
                  if (!allowedVoxelShift)
                  {
                    std::cout << "a: " << a << std::endl;
                    // EXPECT_EQ((int)key.localKey()[a], 0);
                    // EXPECT_EQ((int)keyB.localKey()[a], 0);
                    // EXPECT_EQ((int)keyB.localKey()[a], map.regionVoxelDimensions()[a] - 1);
                    // EXPECT_EQ((int)key.localKey()[a], map.regionVoxelDimensions()[a] - 1);
                  }
                  keysOk = keysOk && allowedVoxelShift;
                }
              }
            }
          }

          if (!keysOk)
          {
            std::cout << "More than one voxel difference : " << key << " vs. " << keyRef << std::endl;
            lineOk = false;
          }
        }
      }

      if (!lineOk)
      {
        std::cout << std::setprecision(20);
        std::cout << "Failed with line [" << r << "]: " << (query.rays()[r * 2 + 0]) << " to " << (query.rays()[r * 2 + 1]) << std::endl;
        EXPECT_TRUE(lineOk);
      }
    }
  }


  void dumpLines(const ohm::LineKeysQuery &query)
  {
    size_t resultCount = query.numberOfResults();
    size_t indexOffset;
    size_t keyCount;
    ohm::OccupancyKey key;

    for (size_t r = 0; r < resultCount; ++r)
    {
      indexOffset = query.resultIndices()[r];
      keyCount = query.resultCounts()[r];

      std::cout << r << " line length " << keyCount << '\n';
      for (size_t k = 0; k < keyCount; ++k)
      {
        key = query.intersectedVoxels()[indexOffset + k];
        std::cout << "  " << k << ": " << key << '\n';
      }
    }
  }


  TEST(LineKeys, QueryGpu)
  {
    const double queryHalfExtents = 10.0;
    const int queryCount = 10000;
    const int iterationCount = 1;
    ohm::OccupancyMap map(0.25);

    std::mt19937 randEngine;
    std::uniform_real_distribution<double> rand(-queryHalfExtents, queryHalfExtents);
    std::vector<glm::dvec3> linePoints;

    // No need to populate the map. We are just calculating ray keys.

    // Full GPU evaluation query.
    ohm::LineKeysQuery gpuQuery(ohm::QF_GpuEvaluate | ohm::QF_NoCache);
    // Allow using cached GPU results query.
    ohm::LineKeysQuery gpuQuery2(ohm::QF_GpuEvaluate);
    // CPU query.
    ohm::LineKeysQuery cpuQuery;

    gpuQuery.setMap(&map);
    gpuQuery2.setMap(&map);
    cpuQuery.setMap(&map);

#if 1
    for (int i = 0; i < queryCount; ++i)
    {
      linePoints.push_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
      linePoints.push_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
    }
#else  // #
    // The following line highlights a deviation between CPU and GPU algorithms. If you push these points
    // as dvec3, then there is a discrepancy between the CPU/GPU results.
    linePoints.push_back(glm::vec3(8.249521446748637743,1.6640723158759520572,15.084516018081700395));
    linePoints.push_back(glm::vec3(11.186323857973910378,-14.315011276937029905,6.2049214437162198976));
#endif // #
    gpuQuery.setRays(linePoints.data(), linePoints.size());
    gpuQuery2.setRays(linePoints.data(), linePoints.size());
    cpuQuery.setRays(linePoints.data(), linePoints.size());

    // Run the initial queries now so we don't measure initialisation and memory allocation and overheads.
    gpuQuery.execute();
    gpuQuery2.execute();
    cpuQuery.execute();

    ohm::OccupancyKeyList keys;
    const size_t lineCount = linePoints.size() / 2;
    for (int i = 0; i < iterationCount; ++i)
    {
      std::cout << "Calculating " << linePoints.size() / 2 << " line segments.\n";
      std::cout << "GPU  " << std::flush;
      auto gpuStart = timing_clock::now();
      bool gpuExecOk = gpuQuery.execute();
      auto gpuEnd = timing_clock::now();
      auto gpuDuration = gpuEnd - gpuStart;
      auto gpuPerLine = gpuDuration / lineCount;
      util::logDuration(std::cout, gpuDuration);
      std::cout << " ";
      util::logDuration(std::cout, gpuPerLine);
      std::cout << " per line" << std::endl;
      EXPECT_TRUE(gpuExecOk);

      std::cout << "Calculating " << linePoints.size() / 2 << " line segments.\n";
      std::cout << "GPU (cached)  " << std::flush;
      auto gpuStart2 = timing_clock::now();
      bool gpuExecOk2 = gpuQuery2.execute();
      auto gpuEnd2 = timing_clock::now();
      auto gpuDuration2 = gpuEnd2 - gpuStart2;
      auto gpuPerLine2 = gpuDuration / lineCount;
      util::logDuration(std::cout, gpuDuration2);
      std::cout << " ";
      util::logDuration(std::cout, gpuPerLine2);
      std::cout << " per line" << std::endl;
      EXPECT_TRUE(gpuExecOk2);

      std::cout << "CPU  " << std::flush;
      auto cpuStart = timing_clock::now();
      bool cpuExecOk = cpuQuery.execute();
      auto cpuEnd = timing_clock::now();
      auto cpuDuration = cpuEnd - cpuStart;
      auto cpuPerLine = cpuDuration / lineCount;
      util::logDuration(std::cout, cpuDuration);
      std::cout << " ";
      util::logDuration(std::cout, cpuPerLine);
      std::cout << " per line" << std::endl;
      EXPECT_TRUE(cpuExecOk);

      std::cout << "CPU2 " << std::flush;
      cpuStart = timing_clock::now();
      for (size_t j = 0; j < lineCount; ++j)
      {
        map.calculateSegmentKeys(keys, linePoints[j * 2 + 0], linePoints[j * 2 + 1], true);
      }
      cpuEnd = timing_clock::now();
      cpuDuration = cpuEnd - cpuStart;
      cpuPerLine = cpuDuration / lineCount;
      util::logDuration(std::cout, cpuDuration);
      std::cout << " ";
      util::logDuration(std::cout, cpuPerLine);
      std::cout << " per line" << std::endl;

      //std::cout << "GPU:\n";
      //dumpLines(gpuQuery);
      //std::cout << "CPU:\n";
      //dumpLines(cpuQuery);
      compareResults(gpuQuery, cpuQuery);
      // Compare cached GPU query against the original.
      compareResults(gpuQuery, gpuQuery2);
    }
  }
}
