// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/mapcache.h>
#include <ohm/occupancygpumap.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancykeylist.h>
#include <ohm/occupancyutil.h>
#include <ohm/mapprobability.h>
#include <ohm/mapchunk.h>

#include <ohmtools/ohmcloud.h>
#include <ohmutil/ohmutil.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <random>

using namespace ohm;

namespace gpumap
{
  typedef std::chrono::high_resolution_clock timing_clock;

  typedef std::function<void (OccupancyMap &, GpuMap &)> PostGpuMapTestFunc;

  bool dumpKeys = false;

  void integrateRays(OccupancyMap &map, const std::vector<glm::dvec3> &rays)
  {
    OccupancyKeyList keys;
    MapCache cache;

    for (size_t i = 0; i < rays.size(); i += 2)
    {
      map.calculateSegmentKeys(keys, rays[i], rays[i + 1], false);

      for (auto &&key : keys)
      {
        map.integrateMiss(key, &cache);
        if (dumpKeys)
        {
          std::cout << ". " << key << '\n';
        }
      }

      map.integrateHit(map.voxelKey(rays[i + 1]), &cache);

      if (dumpKeys)
      {
          std::cout << "* " << map.voxelKey(rays[i + 1]) << '\n';
      }

      dumpKeys = false;
    }
  }

  void gpuMapTest(double resolution, const glm::u8vec3 &regionSize,
                  const std::vector<glm::dvec3> &rays,
                  const PostGpuMapTestFunc &postPopulate,
                  const char *savePrefix = nullptr,
                  size_t batchSize = 0u,
                  size_t gpuMemSize = 0u)
  {
    // Test basic map populate using GPU and ensure it matches CPU (close enough).
    OccupancyMap cpuMap(resolution, regionSize);
    OccupancyMap gpuMap(resolution, regionSize);
    GpuMap gpuWrap(&gpuMap, true, unsigned(batchSize * 2), gpuMemSize); // Borrow pointer.

    ASSERT_TRUE(gpuWrap.gpuOk());

    if (!batchSize)
    {
      batchSize = rays.size() / 2;
    }

    std::cout << "Integrating " << rays.size() / 2 << " rays into each map.\n";

#if 0
    // Output to CSV for Intel Code Builder.
    {
      std::ofstream out("rays.csv");
      glm::dvec3 p = rays[0];
      out << std::setprecision(20);
      for (size_t i = 0; i < rays.size(); ++i)
      {
        p = rays[i];
        out << p.x << ',' << p.y << ',' << p.z;
        if (i + 1 < rays.size())
        {
          out << ',';
        }
        out << '\n';
      }
    }

    {
      std::ofstream out("voxels.csv");

      const MapChunk *chunk = cpuMap.region(glm::i16vec3(0, 0, 0), true);
      const uint8_t *voxelMem = (const uint8_t *)chunk->occupancy;

      for (size_t i = 0; i < cpuMap.regionVoxelVolume(); ++i)
      {
        for (size_t j = 0; j < sizeof(*chunk->occupancy); ++j)
        {
          out << int(voxelMem[j]);

          if (j + 1 < sizeof(*chunk->occupancy) || i + 1 < cpuMap.regionVoxelVolume())
          {
            out << ',' << '\n';
          }
        }
      }
      out << '\n';
    }
    return;
#endif // #

    std::cout << "GPU " << std::flush;
    auto gpuStart = timing_clock::now();
    for (size_t i = 0; i < rays.size(); i += batchSize * 2)
    {
      const unsigned pointCount = unsigned(std::min(batchSize * 2, rays.size() - i));
      gpuWrap.integrateRays(rays.data() + i, pointCount);
    }
    auto gpuQueued = timing_clock::now();
    std::cout << gpuQueued - gpuStart << '\n';

    std::cout << "GPU sync: " << std::flush;
    gpuWrap.syncOccupancy();
    auto gpuEnd = timing_clock::now();
    std::cout << (gpuEnd - gpuQueued) << '\n';
    std::cout << "Per ray: " << (gpuEnd - gpuStart) / (rays.size() / 2);
    std::cout << " queue: " << (gpuQueued - gpuStart) / (rays.size() / 2);
    std::cout << std::endl;

    std::cout << "CPU " << std::flush;
    auto cpuStart = timing_clock::now();
    integrateRays(cpuMap, rays);
    auto cpuEnd = timing_clock::now();
    auto cpuElapsed = cpuEnd - cpuStart;
    std::cout << cpuElapsed << ' ';
    std::cout << cpuElapsed / (rays.size() / 2) << " per ray\n";

    if (postPopulate)
    {
      postPopulate(cpuMap, gpuWrap);
    }

    // std::cout << "Comparing" << std::endl;
    if (savePrefix)
    {
      std::string filename;
      filename = savePrefix;
      filename += "cloud-gpu.ply";
      ohmtools::saveCloud(filename.c_str(), gpuMap);
      filename = savePrefix;
      filename += "cloud-cpu.ply";
      ohmtools::saveCloud(filename.c_str(), cpuMap);
    }
  }

  void compareMaps(const OccupancyMap &referenceMap, const OccupancyMap &testMap)
  {
    // We need to allow for some discrepancies as the GPU map is non-deterministic.
    const float allowedFailureRatio = 0.01f;

    // Note: this test may be too prescriptive.
    // Iterate the CPU map and lookup the GPU map.
    unsigned failures = 0;
    unsigned processed = 0;
    unsigned loggedFailures = 0;
    for (auto iter = referenceMap.begin(); iter != referenceMap.end(); ++iter)
    {
      if (iter->isValid() && iter->value() != ohm::NodeBase::invalidMarkerValue())
      {
        ++processed;
        ohm::OccupancyNodeConst gpuNode = testMap.node(iter->key());
        EXPECT_TRUE(gpuNode.isValid());
        if (gpuNode.isValid() && gpuNode.value())
        {
          if (std::abs(iter->value() - gpuNode.value()) >= referenceMap.hitValue() * 0.5f)
          {
            ++failures;
            if (processed >= 100 && float(failures) / float(processed) > allowedFailureRatio && loggedFailures < 1000)
            {
              EXPECT_NEAR(iter->value(), gpuNode.value(), referenceMap.hitValue() * 0.5f);
              ++loggedFailures;
            }
          }
        }
      }
    }

    if (processed)
    {
      EXPECT_LT(float(failures) / float(processed), allowedFailureRatio);
    }
  }

  void compareCpuGpuMaps(const OccupancyMap &referenceMap, const GpuMap &testMap)
  {
    return compareMaps(referenceMap, testMap.map());
  }

  TEST(GpuMap, PopulateTiny)
  {
    const double mapExtents = 50.0;
    const double resolution = 0.25;
    const unsigned batchSize = 1;
    const glm::u8vec3 regionSize(32);

    // Make a ray.
    std::vector<glm::dvec3> rays;
    rays.emplace_back(glm::dvec3(0.3));
    rays.emplace_back(glm::dvec3(1.1));

    rays.emplace_back(glm::dvec3(-5.0));
    rays.emplace_back(glm::dvec3(0.3));

    gpuMapTest(resolution, regionSize, rays, compareCpuGpuMaps, "tiny", batchSize);
  }

  TEST(GpuMap, PopulateSmall)
  {
    const double mapExtents = 50.0;
    const double resolution = 0.25;
    const unsigned rayCount = 64;
    const unsigned batchSize = 32;
    const glm::u8vec3 regionSize(32);
    // Make some rays.
    std::mt19937 randEngine;
    std::uniform_real_distribution<double> rand(-mapExtents, mapExtents);
    std::vector<glm::dvec3> rays;

    while (rays.size() < rayCount * 2)
    {
      rays.emplace_back(glm::dvec3(0.05));
      rays.emplace_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
    }

    gpuMapTest(resolution, regionSize, rays, compareCpuGpuMaps, "small", batchSize);
  }

  TEST(GpuMap, PopulateLarge)
  {
    const double mapExtents = 50.0;
    const double resolution = 0.25;
    const unsigned rayCount = 1024 * 128;
    const unsigned batchSize = 1024 * 2;
    const glm::u8vec3 regionSize(32);
    // Make some rays.
    std::mt19937 randEngine;
    std::uniform_real_distribution<double> rand(-mapExtents, mapExtents);
    std::vector<glm::dvec3> rays;

    while (rays.size() < rayCount * 2)
    {
      rays.emplace_back(glm::dvec3(0.05));
      rays.emplace_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
    }

    gpuMapTest(resolution, regionSize, rays, compareCpuGpuMaps, "large", batchSize);
  }

  TEST(GpuMap, PopulateSmallCache)
  {
    const double mapExtents = 50.0;
    const double resolution = 0.25;
    const unsigned rayCount = 1024 * 8;
    const unsigned batchSize = 1024 * 2;
    const glm::u8vec3 regionSize(32);
    // Make some rays.
    std::mt19937 randEngine;
    std::uniform_real_distribution<double> rand(-mapExtents, mapExtents);
    std::vector<glm::dvec3> rays;

    while (rays.size() < rayCount * 2)
    {
      rays.emplace_back(glm::dvec3(0.05));
      rays.emplace_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
    }

    // Small cache: 256MiB.
    gpuMapTest(resolution, regionSize, rays, PostGpuMapTestFunc(), "small-cache-", batchSize, 256u * 1024u * 1024);
  }

  TEST(GpuMap, PopulateMultiple)
  {
    // Test having multiple GPU maps operating at once to ensure we don't get any GPU management issues.
    const double mapExtents = 50.0;
    const double resolution = 0.25;
    const unsigned rayCount = 1024 * 8;
    const unsigned batchSize = 1024 * 2; // Must be even
    const glm::u8vec3 regionSize(32);
    // Make some rays.
    std::mt19937 randEngine;
    std::uniform_real_distribution<double> rand(-mapExtents, mapExtents);
    std::vector<glm::dvec3> rays;

    while (rays.size() < rayCount * 2)
    {
      rays.emplace_back(glm::dvec3(0.05));
      rays.emplace_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
    }

    // Two simultaneous, maps with the same scope.
    OccupancyMap map1(resolution, regionSize);
    GpuMap gpuMap1(&map1, true); // Borrow pointer.
    OccupancyMap map2(resolution, regionSize);
    GpuMap gpuMap2(&map2, true); // Borrow pointer.

    // Third map with transient GpuMap wrapper.
    OccupancyMap map3(resolution, regionSize);

    for (unsigned i = 0; i < rays.size(); i += batchSize)
    {
      std::cout << "\r" << i << " / " << rays.size() << std::flush;

      const unsigned remaining = unsigned(rays.size() - i);
      const unsigned currentBatchSize = std::min(batchSize, remaining);
      gpuMap1.integrateRays(rays.data() + i, currentBatchSize);
      gpuMap2.integrateRays(rays.data() + i, currentBatchSize);

      GpuMap gpuMap3(&map3, true); // Borrow pointer.
      gpuMap3.integrateRays(rays.data() + i, currentBatchSize);
      gpuMap3.syncOccupancy();

      // Forth, transient map.
      OccupancyMap map4(resolution, regionSize);
      // std::cout << "\n" << map4.origin() << std::endl;
      GpuMap gpuMap4(&map4, true); // Borrow pointer.
      gpuMap4.integrateRays(rays.data() + i, currentBatchSize);
      gpuMap4.syncOccupancy();
    }
    std::cout << "\r" << rays.size() << " / " << rays.size() << std::endl;

    gpuMap1.syncOccupancy();
    gpuMap2.syncOccupancy();

    std::cout << "Comparing maps" << std::endl;
    compareMaps(map1, map2);
    compareMaps(map1, map3);
  }

  TEST(GpuMap, Compare)
  {
    const double resolution = 0.25;
    const glm::u8vec3 regionSize(16);
    std::vector<glm::dvec3> rays;

    // Create a map for generating voxel centres.
    OccupancyMap gridMap(resolution, regionSize);
    OccupancyKey key(glm::i16vec3(0), 0, 0, 0);
    glm::dvec3 v;
    // Create a set of rays which will densely populate a single region.
    for (int z = 0; z < regionSize.z; ++z)
    {
      key.setLocalAxis(2, z);
      for (int y = 0; y < regionSize.y; ++y)
      {
        key.setLocalAxis(1, y);
        for (int x = 0; x < regionSize.x; ++x)
        {
          key.setLocalAxis(0, x);
          v = gridMap.voxelCentreGlobal(key);
          // Create a ray starting and ending in the selected voxel.
          rays.emplace_back(v);
          rays.emplace_back(v);
        }
      }
    }

    const auto compareResults = [regionSize] (OccupancyMap &cpuMap, OccupancyMap &gpuMap)
    {
      OccupancyKey key(glm::i16vec3(0), 0, 0, 0);
      OccupancyNodeConst cpuVoxel, gpuVoxel;
      // Walk the region pulling a voxel from both maps and comparing.
      for (int z = 0; z < regionSize.z; ++z)
      {
        key.setLocalAxis(2, z);
        for (int y = 0; y < regionSize.y; ++y)
        {
          key.setLocalAxis(1, y);
          for (int x = 0; x < regionSize.x; ++x)
          {
            key.setLocalAxis(0, x);
            cpuVoxel = cpuMap.node(key);
            gpuVoxel = gpuMap.node(key);

            EXPECT_TRUE(cpuVoxel.isValid());
            EXPECT_TRUE(gpuVoxel.isValid());

            EXPECT_EQ(cpuVoxel.value(), gpuVoxel.value());

            if (cpuVoxel.value() != gpuVoxel.value())
            {
              std::cout << "Voxel error: " << key << '\n';
            }
          }
        }
      }
    };

    const auto compareAndClear = [regionSize, compareResults] (OccupancyMap &cpuMap, GpuMap &gpuMap)
    {
      compareResults(cpuMap, gpuMap.map());

      // Now we will try clear all the voxels from the bottom slice, except for those at max Y in the region.
      // To help, we adjust the miss value to greater than the hit probability and then some.
      cpuMap.setMissProbability(valueToProbability(-cpuMap.hitValue() + cpuMap.missValue()));
      gpuMap.map().setMissProbability(valueToProbability(-gpuMap.map().hitValue() + gpuMap.map().missValue()));

      // Build the clearing rays.
      std::vector<glm::dvec3> clearRays;
      OccupancyKey fromKey(glm::i16vec3(0), 0, 0, 0);
      OccupancyKey toKey(glm::i16vec3(0), 0, regionSize.y - 1, 0);
      glm::dvec3 from, to;

      for (int x = 0; x < regionSize.x; ++x)
      {
        fromKey.setLocalAxis(0, x);
        toKey.setLocalAxis(0, x);

        from = cpuMap.voxelCentreGlobal(fromKey);
        to = cpuMap.voxelCentreGlobal(toKey);

        clearRays.emplace_back(from);
        clearRays.emplace_back(to);
      }

      // Add the rays.
      gpuMap.integrateRays(clearRays.data(), unsigned(clearRays.size()));
      // dumpKeys = true;
      integrateRays(cpuMap, clearRays);
      gpuMap.syncOccupancy();

      compareResults(cpuMap, gpuMap.map());
    };

    // gpuMapTest(resolution, regionSize, rays, compareResults, "grid-");
    gpuMapTest(resolution, regionSize, rays, compareAndClear, "grid-");
  }
}
