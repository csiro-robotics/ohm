// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Aabb.h>
#include <ohm/KeyList.h>
#include <ohm/MapChunk.h>
#include <ohm/MapProbability.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/VoxelData.h>
#include <ohmgpu/GpuCache.h>
#include <ohmgpu/GpuMap.h>

#include <ohmtools/OhmCloud.h>
#include <ohmutil/OhmUtil.h>

#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <random>
#include <thread>

// Assertion courtesy of https://github.com/google/googletest/issues/348
#define ASSERT_DURATION_LE(secs, stmt)                                                               \
  {                                                                                                  \
    std::promise<bool> completed;                                                                    \
    auto stmt_future = completed.get_future();                                                       \
    std::thread(                                                                                     \
      [&](std::promise<bool> &completed) {                                                           \
        stmt;                                                                                        \
        completed.set_value(true);                                                                   \
      },                                                                                             \
      std::ref(completed))                                                                           \
      .detach();                                                                                     \
    if (stmt_future.wait_for(std::chrono::seconds(secs)) == std::future_status::timeout)             \
      GTEST_FATAL_FAILURE_("       timed out (> " #secs " seconds). Check code for infinite loops"); \
  }

using namespace ohm;

namespace gpumap
{
typedef std::chrono::high_resolution_clock TimingClock;

typedef std::function<void(OccupancyMap &, GpuMap &)> PostGpuMapTestFunc;

void gpuMapTest(double resolution, const glm::u8vec3 &region_size, const std::vector<glm::dvec3> &rays,
                const PostGpuMapTestFunc &post_populate, const char *save_prefix = nullptr, size_t batch_size = 0u,
                size_t gpu_mem_size = 0u, bool voxel_means = false)
{
  // Test basic map populate using GPU and ensure it matches CPU (close enough).
  OccupancyMap cpu_map(resolution, region_size, voxel_means ? MapFlag::kVoxelMean : MapFlag::kNone);
  OccupancyMap gpu_map(resolution, region_size, voxel_means ? MapFlag::kVoxelMean : MapFlag::kNone);
  GpuMap gpu_wrap(&gpu_map, true, unsigned(batch_size * 2), gpu_mem_size);  // Borrow pointer.

  ASSERT_TRUE(gpu_wrap.gpuOk());

  if (!batch_size)
  {
    batch_size = rays.size() / 2;
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
#endif  // #

  std::cout << "GPU " << std::flush;
  const auto gpu_start = TimingClock::now();
  for (size_t i = 0; i < rays.size(); i += batch_size * 2)
  {
    const unsigned point_count = unsigned(std::min(batch_size * 2, rays.size() - i));
    gpu_wrap.integrateRays(rays.data() + i, point_count);
  }
  const auto gpu_queued = TimingClock::now();
  std::cout << gpu_queued - gpu_start << '\n';

  std::cout << "GPU sync: " << std::flush;
  gpu_wrap.syncVoxels();
  const auto gpu_end = TimingClock::now();
  std::cout << (gpu_end - gpu_queued) << '\n';
  std::cout << "Per ray: " << (gpu_end - gpu_start) / (rays.size() / 2);
  std::cout << " queue: " << (gpu_queued - gpu_start) / (rays.size() / 2);
  std::cout << std::endl;

  std::cout << "CPU " << std::flush;
  const auto cpu_start = TimingClock::now();
  cpu_map.integrateRays(rays.data(), unsigned(rays.size()));
  const auto cpu_end = TimingClock::now();
  const auto cpu_elapsed = cpu_end - cpu_start;
  std::cout << cpu_elapsed << ' ';
  std::cout << cpu_elapsed / (rays.size() / 2) << " per ray\n";

  if (post_populate)
  {
    post_populate(cpu_map, gpu_wrap);
  }

  // std::cout << "Comparing" << std::endl;
  if (save_prefix)
  {
    const auto save_start = TimingClock::now();
    std::cout << "Saving clouds " << std::flush;

    std::string filename;
    filename = save_prefix;
    filename += "cloud-gpu.ply";
    ohmtools::saveCloud(filename.c_str(), gpu_map);
    filename = save_prefix;
    filename += "cloud-cpu.ply";
    ohmtools::saveCloud(filename.c_str(), cpu_map);
    filename = save_prefix;
    filename += "gpu.ohm";
    ohm::save(filename.c_str(), gpu_map);
    filename = save_prefix;
    filename += "cpu.ohm";
    ohm::save(filename.c_str(), cpu_map);

    const auto save_end = TimingClock::now();
    const auto save_elapsed = save_end - save_start;
    std::cout << save_elapsed << std::endl;
  }
}

void compareMaps(const OccupancyMap &reference_map, const OccupancyMap &test_map)
{
  const auto compare_start = TimingClock::now();
  std::cout << "Compare maps " << std::flush;
  // We need to allow for some discrepancies as the GPU map is non-deterministic.
  const float allowed_failure_ratio = 0.01f;

  // Note: this test may be too prescriptive.
  // Iterate the CPU map and lookup the GPU map.
  unsigned failures = 0;
  unsigned processed = 0;
  unsigned logged_failures = 0;
  float expect, actual;

  glm::dvec3 ref_pos, test_pos;

  const auto should_report_failure = [&failures, &processed, &logged_failures, &allowed_failure_ratio]() {
    return float(failures) / float(processed) > allowed_failure_ratio && logged_failures < 100;
  };

  ohm::Voxel<const float> ref_occupancy(&reference_map, reference_map.layout().occupancyLayer());
  ohm::Voxel<const ohm::VoxelMean> ref_mean(&reference_map, reference_map.layout().meanLayer());
  ohm::Voxel<const float> test_occupancy(&test_map, test_map.layout().occupancyLayer());
  ohm::Voxel<const ohm::VoxelMean> test_mean(&test_map, test_map.layout().meanLayer());
  ASSERT_TRUE(ref_occupancy.isLayerValid());
  ASSERT_TRUE(test_occupancy.isLayerValid());
  ASSERT_EQ(test_mean.isLayerValid(), ref_mean.isLayerValid());
  for (auto iter = reference_map.begin(); iter != reference_map.end(); ++iter)
  {
    ohm::setVoxelKey(iter, ref_occupancy, ref_mean);
    if (!ohm::isUnobservedOrNull(ref_occupancy))
    {
      ++processed;
      // Note: we must deference the iterator and use the key value only because the iterator is from a difference
      // map.
      ohm::setVoxelKey(*iter, test_occupancy, test_mean);

      if (test_occupancy.isValid())
      {
        bool ok = true;
        ref_occupancy.read(&expect);
        test_occupancy.read(&actual);

        if (std::abs(expect - actual) >= reference_map.hitValue() * 0.5f)
        {
          ok = false;

          if (should_report_failure())
          {
            EXPECT_NEAR(expect, actual, reference_map.hitValue() * 0.5f);
            ++logged_failures;
          }
        }

        ref_pos = ohm::positionSafe(ref_mean);
        test_pos = ohm::positionSafe(test_mean);

        // Ensure we aren't just getting the default code path all the time.
        if (ref_mean.isValid())
        {
          const glm::dvec3 pos = ohm::positionUnsafe(ref_mean);
          EXPECT_NEAR(ref_pos.x, pos.x, 1e-4f);
          EXPECT_NEAR(ref_pos.y, pos.y, 1e-4f);
          EXPECT_NEAR(ref_pos.z, pos.z, 1e-4f);
        }

        if (glm::any(glm::greaterThan(glm::abs(ref_pos - test_pos), glm::dvec3(1e-5))))
        {
          ok = false;
          if (should_report_failure())
          {
            EXPECT_NEAR(ref_pos.x, test_pos.x, 1e-5);
            EXPECT_NEAR(ref_pos.y, test_pos.y, 1e-5);
            EXPECT_NEAR(ref_pos.z, test_pos.z, 1e-5);
          }
        }

        if (!ok)
        {
          ++failures;
        }
      }
      else
      {
        ++failures;
        if (should_report_failure())
        {
          // Revalidate the voxel for reporting purposes only. We know it's false here.
          EXPECT_TRUE(test_occupancy.isValid());
          ++logged_failures;
        }
      }
    }
  }

  if (processed)
  {
    EXPECT_LT(float(failures) / float(processed), allowed_failure_ratio);
  }

  const auto compare_end = TimingClock::now();
  const auto compare_elapsed = compare_end - compare_start;
  std::cout << compare_elapsed << std::endl;
}

void compareCpuGpuMaps(const OccupancyMap &reference_map, const GpuMap &test_map)
{
  return compareMaps(reference_map, test_map.map());
}

TEST(GpuMap, PopulateTiny)
{
  const double resolution = 0.25;
  const unsigned batch_size = 2;
  const glm::u8vec3 region_size(32);

  // Make a ray.
  std::vector<glm::dvec3> rays;
  rays.emplace_back(glm::dvec3(0.3));
  rays.emplace_back(glm::dvec3(1.1));

  rays.emplace_back(glm::dvec3(-5.0));
  rays.emplace_back(glm::dvec3(0.3));

  gpuMapTest(resolution, region_size, rays, compareCpuGpuMaps, "tiny", batch_size);
}

TEST(GpuMap, PopulateSmall)
{
  const double map_extents = 50.0;
  const double resolution = 0.25;
  const unsigned ray_count = 64;
  const unsigned batch_size = 32;
  const glm::u8vec3 region_size(32);
  // Make some rays.
  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> rand(-map_extents, map_extents);
  std::vector<glm::dvec3> rays;

  while (rays.size() < ray_count * 2)
  {
    rays.emplace_back(glm::dvec3(0.05));
    rays.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
  }

  gpuMapTest(resolution, region_size, rays, compareCpuGpuMaps, "small", batch_size);
}

TEST(GpuMap, PopulateLarge)
{
  const double map_extents = 25.0;
  const double resolution = 0.25;
  const unsigned ray_count = 1024 * 128;
  const unsigned batch_size = 1024 * 2;
  const glm::u8vec3 region_size(32);
  // Make some rays.
  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> rand(-map_extents, map_extents);
  std::vector<glm::dvec3> rays;

  while (rays.size() < ray_count * 2)
  {
    rays.emplace_back(glm::dvec3(0.05));
    rays.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
  }

  gpuMapTest(resolution, region_size, rays, compareCpuGpuMaps, "large", batch_size);
}

TEST(GpuMap, PopulateSmallCache)
{
  const double map_extents = 50.0;
  const double resolution = 0.25;
  const unsigned ray_count = 1024 * 8;
  const unsigned batch_size = 1024 * 2;
  const glm::u8vec3 region_size(32);
  // Make some rays.
  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> rand(-map_extents, map_extents);
  std::vector<glm::dvec3> rays;

  while (rays.size() < ray_count * 2)
  {
    rays.emplace_back(glm::dvec3(0.05));
    rays.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
  }

  // Small cache: 256MiB.
  gpuMapTest(resolution, region_size, rays, PostGpuMapTestFunc(), "small-cache-", batch_size, 256u * 1024u * 1024);
}

TEST(GpuMap, PopulateMultiple)
{
  // Test having multiple GPU maps operating at once to ensure we don't get any GPU management issues.
  const double map_extents = 50.0;
  const double resolution = 0.25;
  const unsigned ray_count = 1024 * 8;
  const unsigned batch_size = 1024 * 2;  // Must be even
  const size_t target_gpu_cache_size = GpuCache::kMiB * 200;
  const glm::u8vec3 region_size(32);
  // Make some rays.
  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> rand(-map_extents, map_extents);
  std::vector<glm::dvec3> rays;

  while (rays.size() < ray_count * 2)
  {
    rays.emplace_back(glm::dvec3(0.05));
    rays.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
  }

  // Two simultaneous, maps with the same scope.
  OccupancyMap map1(resolution, region_size);
  GpuMap gpu_map1(&map1, true, batch_size, target_gpu_cache_size);  // Borrow pointer.
  OccupancyMap map2(resolution, region_size);
  GpuMap gpu_map2(&map2, true, batch_size, target_gpu_cache_size);  // Borrow pointer.

  // Third map with transient GpuMap wrapper.
  OccupancyMap map3(resolution, region_size);

  for (unsigned i = 0; i < rays.size(); i += batch_size)
  {
    std::cout << "\r" << i << " / " << rays.size() << std::flush;

    const unsigned remaining = unsigned(rays.size() - i);
    const unsigned current_batch_size = std::min(batch_size, remaining);
    gpu_map1.integrateRays(rays.data() + i, current_batch_size);
    gpu_map2.integrateRays(rays.data() + i, current_batch_size);

    GpuMap gpu_map3(&map3, true, batch_size, target_gpu_cache_size);  // Borrow pointer.
    gpu_map3.integrateRays(rays.data() + i, current_batch_size);
    gpu_map3.syncVoxels();

    // Forth, transient map.
    OccupancyMap map4(resolution, region_size);
    // std::cout << "\n" << map4.origin() << std::endl;
    GpuMap gpu_map4(&map4, true, batch_size, target_gpu_cache_size);  // Borrow pointer.
    gpu_map4.integrateRays(rays.data() + i, current_batch_size);
    gpu_map4.syncVoxels();
  }
  std::cout << "\r" << rays.size() << " / " << rays.size() << std::endl;

  gpu_map1.syncVoxels();
  gpu_map2.syncVoxels();

  std::cout << "Comparing maps" << std::endl;
  compareMaps(map1, map2);
  compareMaps(map1, map3);
}

TEST(GpuMap, Compare)
{
  const double resolution = 0.25;
  const glm::u8vec3 region_size(16);
  std::vector<glm::dvec3> rays;

  // Create a map for generating voxel centres.
  OccupancyMap grid_map(resolution, region_size);
  Key key(glm::i16vec3(0), 0, 0, 0);
  glm::dvec3 v;
  // Create a set of rays which will densely populate a single region.
  for (int z = 0; z < region_size.z; ++z)
  {
    key.setLocalAxis(2, z);
    for (int y = 0; y < region_size.y; ++y)
    {
      key.setLocalAxis(1, y);
      for (int x = 0; x < region_size.x; ++x)
      {
        key.setLocalAxis(0, x);
        v = grid_map.voxelCentreGlobal(key);
        // Create a ray starting and ending in the selected voxel.
        rays.emplace_back(v);
        rays.emplace_back(v);
      }
    }
  }

  const auto compare_results = [region_size](OccupancyMap &cpu_map, OccupancyMap &gpu_map) {
    Key key(glm::i16vec3(0), 0, 0, 0);
    ohm::Voxel<const float> cpu_voxel(&cpu_map, cpu_map.layout().occupancyLayer());
    ohm::Voxel<const float> gpu_voxel(&gpu_map, gpu_map.layout().occupancyLayer());
    ASSERT_TRUE(cpu_voxel.isLayerValid());
    ASSERT_TRUE(gpu_voxel.isLayerValid());
    // Walk the region pulling a voxel from both maps and comparing.
    for (int z = 0; z < region_size.z; ++z)
    {
      key.setLocalAxis(2, z);
      for (int y = 0; y < region_size.y; ++y)
      {
        key.setLocalAxis(1, y);
        for (int x = 0; x < region_size.x; ++x)
        {
          key.setLocalAxis(0, x);
          cpu_voxel.setKey(key);
          gpu_voxel.setKey(key);

          ASSERT_TRUE(cpu_voxel.isValid());
          ASSERT_TRUE(gpu_voxel.isValid());

          float cpu_occupancy, gpu_occupancy;
          cpu_voxel.read(&cpu_occupancy);
          gpu_voxel.read(&gpu_occupancy);

          EXPECT_EQ(cpu_occupancy, gpu_occupancy);

          if (cpu_occupancy != gpu_occupancy)
          {
            std::cout << "Voxel error: " << key << '\n';
          }
        }
      }
    }
  };

  const auto compare_and_clear = [region_size, compare_results](OccupancyMap &cpu_map, GpuMap &gpu_map) {
    compare_results(cpu_map, gpu_map.map());

    // Now we will try clear all the voxels from the bottom slice, except for those at max Y in the region.
    // To help, we adjust the miss value to greater than the hit probability and then some.
    cpu_map.setMissProbability(valueToProbability(-cpu_map.hitValue() + cpu_map.missValue()));
    gpu_map.map().setMissProbability(valueToProbability(-gpu_map.map().hitValue() + gpu_map.map().missValue()));

    // Build the clearing rays.
    std::vector<glm::dvec3> clear_rays;
    Key from_key(glm::i16vec3(0), 0, 0, 0);
    Key to_key(glm::i16vec3(0), 0, region_size.y - 1, 0);
    glm::dvec3 from, to;

    for (int x = 0; x < region_size.x; ++x)
    {
      from_key.setLocalAxis(0, x);
      to_key.setLocalAxis(0, x);

      from = cpu_map.voxelCentreGlobal(from_key);
      to = cpu_map.voxelCentreGlobal(to_key);

      clear_rays.emplace_back(from);
      clear_rays.emplace_back(to);
    }

    // Add the rays.
    gpu_map.integrateRays(clear_rays.data(), unsigned(clear_rays.size()));
    // dumpKeys = true;
    cpu_map.integrateRays(clear_rays.data(), unsigned(clear_rays.size()));
    gpu_map.syncVoxels();

    compare_results(cpu_map, gpu_map.map());
  };

  // gpuMapTest(resolution, regionSize, rays, compareResults, "grid-");
  gpuMapTest(resolution, region_size, rays, compare_and_clear, "grid-");
}


TEST(GpuMap, ClipBox)
{
  // Test clipping of rays to an Aabb on insert.
  const double resolution = 0.2;
  const unsigned batch_size = 2 * 1024u;
  const uint8_t region_size = 32u;
  OccupancyMap gpu_map(resolution, glm::u8vec3(region_size));
  GpuMap gpu_wrap(&gpu_map, true, unsigned(batch_size * 2));  // Borrow pointer.

  Aabb clip_box(glm::dvec3(-1.0), glm::dvec3(2.0));

  const auto clip_filter = [&clip_box](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) {
    return clipBounded(start, end, filter_flags, clip_box);
  };
  gpu_wrap.setRayFilter(clip_filter);

  std::vector<glm::dvec3> rays;

  // Start with rays which pass through the box.
  rays.push_back(glm::dvec3(-2, 0, 0));
  rays.push_back(glm::dvec3(3, 0, 0));

  rays.push_back(glm::dvec3(0, -2, 0));
  rays.push_back(glm::dvec3(0, 3, 0));

  rays.push_back(glm::dvec3(0, 0, 3));
  rays.push_back(glm::dvec3(0, 0, -2));

  gpu_wrap.integrateRays(rays.data(), unsigned(rays.size()));
  gpu_wrap.syncVoxels();

  // Validate the map contains no occupied points; only free and unknown.
  const glm::dvec3 voxel_half_extents(0.5 * gpu_map.resolution());
  bool touched = false;
  ohm::Voxel<const float> voxel(&gpu_map, gpu_map.layout().occupancyLayer());
  ASSERT_TRUE(voxel.isLayerValid());
  for (auto iter = gpu_map.begin(); iter != gpu_map.end(); ++iter)
  {
    voxel.setKey(iter);
    touched = true;

    if (voxel.isValid())
    {
      float occupancy;
      voxel.read(&occupancy);
      if (!ohm::isUnobserved(occupancy))
      {
        EXPECT_LT(occupancy, gpu_map.occupancyThresholdValue());
        EXPECT_FALSE(ohm::isOccupied(voxel));

        // Voxel should also be with in the bounds of the Aabb. Check this.
        const Aabb voxel_box(gpu_map.voxelCentreGlobal(voxel.key()) - voxel_half_extents,
                             gpu_map.voxelCentreGlobal(voxel.key()) + voxel_half_extents);
        EXPECT_TRUE(clip_box.overlaps(voxel_box)) << "Voxel box does not overlap extents";
      }
    }
  }
  voxel.reset();

  EXPECT_TRUE(touched);

  // Reset the map. This also tests that resetting a GPU map works.
  gpu_map.clear();

  // Now rays which enter the box, ending at the origin.
  // Start with rays which pass through the box.
  rays.push_back(glm::dvec3(-2, 0, 0));
  rays.push_back(glm::dvec3(0, 0, 0));

  rays.push_back(glm::dvec3(0, -2, 0));
  rays.push_back(glm::dvec3(0, 0, 0));

  rays.push_back(glm::dvec3(0, 0, 3));
  rays.push_back(glm::dvec3(0, 0, 0));

  gpu_wrap.integrateRays(rays.data(), unsigned(rays.size()));
  gpu_wrap.syncVoxels();

  // Validate the map contains an occupied point at the origin voxel.
  const Key target_key = gpu_map.voxelKey(glm::dvec3(0));
  touched = false;
  voxel = ohm::Voxel<const float>(&gpu_map, gpu_map.layout().occupancyLayer());
  ASSERT_TRUE(voxel.isLayerValid());
  for (auto iter = gpu_map.begin(); iter != gpu_map.end(); ++iter)
  {
    voxel.setKey(iter);
    touched = true;

    if (voxel.isValid())
    {
      float occupancy;
      voxel.read(&occupancy);
      if (voxel.key() != target_key)
      {
        if (!ohm::isUnobserved(voxel))
        {
          EXPECT_LT(occupancy, gpu_map.occupancyThresholdValue());
        }
        EXPECT_FALSE(ohm::isOccupied(voxel));
      }
      else
      {
        EXPECT_GE(occupancy, gpu_map.occupancyThresholdValue());
        EXPECT_TRUE(ohm::isOccupied(voxel));
      }

      // Touched voxels should also be with in the bounds of the Aabb. Check this.
      if (!ohm::isUnobserved(voxel))
      {
        const Aabb voxel_box(gpu_map.voxelCentreGlobal(voxel.key()) - voxel_half_extents,
                             gpu_map.voxelCentreGlobal(voxel.key()) + voxel_half_extents);
        EXPECT_TRUE(clip_box.overlaps(voxel_box)) << "Voxel box does not overlap extents";
      }
    }
  }
  voxel.reset();

  EXPECT_TRUE(touched);
}


TEST(GpuMap, ClipBoxCompare)
{
  // Test clipping of rays to an Aabb on insert.
  const double resolution = 0.2;
  const unsigned batch_size = 2 * 1024u;
  const uint8_t region_size = 32u;
  OccupancyMap cpu_map(resolution, glm::u8vec3(region_size));
  OccupancyMap gpu_map(resolution, glm::u8vec3(region_size));
  GpuMap gpu_wrap(&gpu_map, true, unsigned(batch_size * 2));  // Borrow pointer.

  Aabb clip_box(glm::dvec3(-1.0), glm::dvec3(2.0));

  const auto clip_filter = [&clip_box](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) {
    return clipBounded(start, end, filter_flags, clip_box);
  };

  cpu_map.setRayFilter(clip_filter);
  gpu_wrap.setRayFilter(clip_filter);

  std::vector<glm::dvec3> rays;

  // Compare GPU/CPU map clipping results.
  // Start with rays which pass through the box.
  rays.push_back(glm::dvec3(-2, 0, 0));
  rays.push_back(glm::dvec3(0, 0, 0));

  rays.push_back(glm::dvec3(0, -2, 0));
  rays.push_back(glm::dvec3(0, 0, 0));

  rays.push_back(glm::dvec3(0, 0, 3));
  rays.push_back(glm::dvec3(0, 0, 0));

  cpu_map.integrateRays(rays.data(), unsigned(rays.size()));
  gpu_wrap.integrateRays(rays.data(), unsigned(rays.size()));
  gpu_wrap.syncVoxels();

  compareMaps(cpu_map, gpu_map);
}

TEST(GpuMap, VoxelMean)
{
  // Populate with voxel means
  const double map_extents = 50.0;
  const double resolution = 0.25;
  const unsigned ray_count = 64;
  const unsigned batch_size = 32;
  const glm::u8vec3 region_size(32);

  // Make some rays.
  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> rand(-map_extents, map_extents);
  std::vector<glm::dvec3> rays;

  while (rays.size() < ray_count * 2)
  {
    rays.emplace_back(glm::dvec3(0.05));
    rays.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
  }

  gpuMapTest(resolution, region_size, rays, compareCpuGpuMaps, "voxelmean", batch_size, 0, true);
}

TEST(GpuMap, CheckBadRays)
{
  const double resolution = 0.1;
  const glm::u8vec3 region_size(32);
  const unsigned batch_size = 32;

  // Rays which have been known to cause infinite loops.
  std::vector<glm::dvec3> rays =  //
    {                             //
      // Infinite loop walking regions before integrating into the map.
      glm::dvec3{ -2.699077907025583, -1.5999031032475868, 1.0755428728082643 },
      glm::dvec3{ -2.6998157732186034, -1.6000298354709896, 1.0756803244026165 }
    };


  ASSERT_DURATION_LE(5, gpuMapTest(resolution, region_size, rays, compareCpuGpuMaps, "bad-rays", batch_size, 0, true));
}
}  // namespace gpumap
