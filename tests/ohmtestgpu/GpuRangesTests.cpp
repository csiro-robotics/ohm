// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/MapChunk.h>
#include <ohm/MapLayout.h>
#include <ohm/MapProbability.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/QueryFlag.h>
#include <ohm/VoxelData.h>
#include <ohmgpu/ClearanceProcess.h>
#include <ohmgpu/GpuMap.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>

#include <logutil/LogUtil.h>

#include <ohmutil/PlyMesh.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

using namespace ohm;

namespace ranges
{
typedef std::chrono::high_resolution_clock TimingClock;


void saveClearanceCloud(const char *file_name, const OccupancyMap &map, const glm::i16vec3 &region, float search_radius)
{
  PlyMesh ply;
  Key key(region, 0, 0, 0);
  glm::vec3 voxel_pos;
  // auto region_size = map.regionVoxelDimensions();
  // std::cout << "regionSize: " << regionSize << std::endl;

  Voxel<const float> clearance(&map, map.layout().clearanceLayer(), key);
  if (clearance.isValid())
  {
    do
    {
      uint8_t r = 255;
      uint8_t g = 128;
      uint8_t b = 0;
      float range_value;
      clearance.read(&range_value);
      if (range_value < 0)
      {
        range_value = search_radius;
      }
      r = uint8_t(255 * std::max(0.0f, (search_radius - range_value) / search_radius));
      voxel_pos = map.voxelCentreGlobal(clearance.key());
      ply.addVertex(voxel_pos, Colour(r, g, b));
    } while (clearance.nextInRegion());
  }

  ply.save(file_name, true);
}


void showVoxel(ohm::OccupancyMap &map, const ohm::Key &key, float expected_range = FLT_MAX)
{
  Voxel<const float> occupancy(&map, map.layout().occupancyLayer(), key);
  Voxel<const float> clearance(&map, map.layout().clearanceLayer(), key);
  float occupancy_value = 0, clearance_value = 0;
  if (occupancy.isValid())
  {
    occupancy.read(&occupancy_value);
  }
  if (occupancy.isValid())
  {
    clearance.read(&clearance_value);
  }

  if (occupancy.isValid() && clearance.isValid())
  {
    std::cout << occupancy.key() << " " << occupancy_value << ": " << clearance_value << '\n';
  }
  else
  {
    std::cout << key << " invalid\n";
  }

  if (expected_range != FLT_MAX)
  {
    EXPECT_TRUE(clearance.isValid());
    if (clearance.isValid())
    {
      EXPECT_NEAR(clearance_value, expected_range, 1e-3f);
    }
  }
}


TEST(Ranges, Simple)
{
  const double resolution = 1.0;
  const glm::u8vec3 region_size(32);
  // Deliberately require search padding of more than one region to validate GPU memory alignment.
  const float search_range = float(resolution) * float(region_size.x);
  OccupancyMap map(resolution, region_size);

  std::cout << "Populate map: " << std::flush;
  auto start_time = TimingClock::now();

  // Build a map in which we clear all voxels except the one at key (0, 0, 0 : 0, 0, 0).
  // We clear region (0, 0, 0) and make sure we don't query with the flag kQfUnknownAsOccupied.
  ohmgen::fillMapWithEmptySpace(map, 0, 0, 0, region_size.x, region_size.y, region_size.z);
  {
    ohm::Voxel<float> occupied_voxel(&map, map.layout().occupancyLayer(), Key(0, 0, 0, 0, 0, 0));
    ASSERT_TRUE(occupied_voxel.isValid());
    occupied_voxel.write(map.occupancyThresholdValue());
  }

  auto end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  std::cout << "Obstacle query: " << std::flush;
  start_time = TimingClock::now();
  // No need to sync the map yet.
  Key key(0, 0, 0, 0, 0, 0);

  ClearanceProcess clearance_process(search_range, kQfGpuEvaluate);
  clearance_process.calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key));
  end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  // Now check obstacle ranges in the map.
  std::cout << "Post query\n";
  key = Key(0, 0, 0, 0, 0, 0);
  showVoxel(map, key, 0.0f);
  map.stepKey(key, 0, 1);
  showVoxel(map, key, 1.0f);
  map.stepKey(key, 0, 1);
  showVoxel(map, key, 2.0f);

  saveClearanceCloud("ranges-simple-region.ply", map, glm::i16vec3(0, 0, 0), clearance_process.searchRadius());
  ohmtools::saveCloud("ranges-simple-cloud.ply", map);

  key = Key(0, 0, 0, 0, 0, 0);
  Voxel<const float> clearance(&map, map.layout().clearanceLayer(), key);
  ASSERT_TRUE(clearance.isValid());
  const unsigned max_failures = 20;
  unsigned failure_count = 0;
  do
  {
    ASSERT_TRUE(clearance.isValid());
    const float epsilon = 1e-3f;
    const float dist_to_region_origin = glm::length(glm::vec3(clearance.key().localKey()));
    const float expected_range =
      (dist_to_region_origin <= clearance_process.searchRadius()) ? dist_to_region_origin : -1.0f;
    float clearance_value;
    clearance.read(&clearance_value);
    if (std::abs(expected_range - clearance_value) > epsilon)
    {
      std::cout << "Fail: " << clearance.key() << " to RO: " << dist_to_region_origin << " expect: " << expected_range
                << " actual: " << clearance_value << '\n';
      ++failure_count;
    }
    EXPECT_NEAR(expected_range, clearance_value, epsilon);
  } while (clearance.nextInRegion() && failure_count < max_failures);

  if (failure_count == max_failures)
  {
    FAIL() << "Too many failure";
  }
  clearance.reset();
}


TEST(Ranges, Serialised)
{
  // Validate that if we save and reload a map, we can get expected clearance values back.
  // This test came about due to a bug where MapChunk::touched_stamps was not saved. This meant that on load, all
  // MapChunk::touched_stamps would be set back to zero and match when the LineQuery checked the need to recalculate
  // clearance values (on for GPU). No clearance values would be calculated as a result.
  const double resolution = 1.0;
  const glm::u8vec3 region_size(32);
  const char *map_file = "ranges-serialise.ohm";

  // Deliberately require search padding of more than one region to validate GPU memory alignment.
  const float search_range = float(resolution) * float(region_size.x);
  OccupancyMap map(resolution, region_size);

  std::cout << "Populate map: " << std::flush;
  auto start_time = TimingClock::now();

  // Build a map in which we clear all voxels except the one at key (0, 0, 0 : 0, 0, 0).
  // We clear region (0, 0, 0) and make sure we don't query with the flag kQfUnknownAsOccupied.
  ohmgen::fillMapWithEmptySpace(map, 0, 0, 0, region_size.x, region_size.y, region_size.z);
  {
    ohm::Voxel<float> occupied_voxel(&map, map.layout().occupancyLayer(), Key(0, 0, 0, 0, 0, 0));
    ASSERT_TRUE(occupied_voxel.isValid());
    occupied_voxel.write(map.occupancyThresholdValue());
  }

  auto end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  // Save and load the map.
  std::cout << "Save map file " << map_file << std::endl;
  ohm::save(map_file, map);

  // Clear the map.
  map.clear();

  // And load what we saved.
  std::cout << "Load map file " << map_file << std::endl;
  ohm::load(map_file, map);

  // Perform a GPU range query.
  std::cout << "Query cycle 1: " << std::flush;
  start_time = TimingClock::now();
  // No need to sync the map yet.
  Key key(0, 0, 0, 0, 0, 0);

  std::unique_ptr<ClearanceProcess> clearance_process(new ClearanceProcess(search_range, kQfGpuEvaluate));
  clearance_process->calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key), false);
  end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  // Now check obstacle ranges in the map.
  key = Key(0, 0, 0, 0, 0, 0);
  showVoxel(map, key, 0.0f);
  map.stepKey(key, 0, 1);
  showVoxel(map, key, 1.0f);
  map.stepKey(key, 0, 1);
  showVoxel(map, key, 2.0f);

  saveClearanceCloud("ranges-serialise-region.ply", map, glm::i16vec3(0, 0, 0), clearance_process->searchRadius());
  ohmtools::saveCloud("ranges-serialise-cloud.ply", map);

  key = Key(0, 0, 0, 0, 0, 0);
  Voxel<const float> clearance(&map, map.layout().clearanceLayer(), key);
  ASSERT_TRUE(clearance.isValid());
  const unsigned max_failures = 20;
  unsigned failure_count = 0;
  do
  {
    ASSERT_TRUE(clearance.isValid());
    const float epsilon = 1e-3f;
    const float dist_to_region_origin = glm::length(glm::vec3(clearance.key().localKey()));
    const float expected_range =
      (dist_to_region_origin <= clearance_process->searchRadius()) ? dist_to_region_origin : -1.0f;
    float clearance_value;
    clearance.read(&clearance_value);
    if (std::abs(expected_range - clearance_value) > epsilon)
    {
      std::cout << "Fail: " << clearance.key() << " to RO: " << dist_to_region_origin << " expect: " << expected_range
                << " actual: " << clearance_value << '\n';
      ++failure_count;
    }
    EXPECT_NEAR(expected_range, clearance_value, epsilon);
  } while (clearance.nextInRegion() && failure_count < max_failures);

  // About to invalidate the map.
  clearance.reset();

  EXPECT_LE(failure_count, max_failures) << "Too many failure";

  clearance_process.reset(nullptr);

  // Now save the map with clearance data embedded and reload that.
  // Save and load the map.
  std::cout << "Save map file " << map_file << std::endl;
  ohm::save(map_file, map);

  // Clear the map.
  map.clear();

  // And load what we saved.
  std::cout << "Load map file " << map_file << std::endl;
  ohm::load(map_file, map);

  // More queries.
  std::cout << "Query cycle 2: " << std::flush;
  clearance_process.reset(new ClearanceProcess(search_range, kQfGpuEvaluate));
  clearance_process->calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key), false);
  end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  // Now check obstacle ranges in the map.
  std::cout << "Post query\n";
  key = Key(0, 0, 0, 0, 0, 0);
  showVoxel(map, key, 0.0f);
  map.stepKey(key, 0, 1);
  showVoxel(map, key, 1.0f);
  map.stepKey(key, 0, 1);
  showVoxel(map, key, 2.0f);

  saveClearanceCloud("ranges-serialise-region.ply", map, glm::i16vec3(0, 0, 0), clearance_process->searchRadius());
  ohmtools::saveCloud("ranges-serialise-cloud.ply", map);

  key = Key(0, 0, 0, 0, 0, 0);
  clearance = Voxel<const float>(&map, map.layout().clearanceLayer(), key);
  // const float *values = (const float *)map.layout().layer(1).voxels(*map.region(glm::i16vec3(0, 0, 0)));
  ASSERT_TRUE(clearance.isValid());
  failure_count = 0;
  do
  {
    ASSERT_TRUE(clearance.isValid());
    const float epsilon = 1e-3f;
    const float dist_to_region_origin = glm::length(glm::vec3(clearance.key().localKey()));
    const float expected_range =
      (dist_to_region_origin <= clearance_process->searchRadius()) ? dist_to_region_origin : -1.0f;
    float clearance_value;
    clearance.read(&clearance_value);
    if (std::abs(expected_range - clearance_value) > epsilon)
    {
      std::cout << "Fail: " << clearance.key() << " to RO: " << dist_to_region_origin << " expect: " << expected_range
                << " actual: " << clearance_value << '\n';
      ++failure_count;
    }
    EXPECT_NEAR(expected_range, clearance_value, epsilon);
  } while (clearance.nextInRegion() && failure_count < max_failures);
  clearance.reset();

  EXPECT_LE(failure_count, max_failures) << "Too many failure";

  clearance_process.reset(nullptr);

  EXPECT_LE(failure_count, max_failures) << "Too many failure";
}


void testMapOuterEdge(OccupancyMap &map, float search_range, bool unknown_as_occupied)
{
  const glm::u8vec3 region_size = map.regionVoxelDimensions();
  const auto start_time = TimingClock::now();

  std::cout << "Obstacle query: " << std::flush;
  // No need to sync the map yet.
  Key key(0, 0, 0, 0, 0, 0);

  ClearanceProcess clearance_process(search_range, kQfGpuEvaluate | !!unknown_as_occupied * kQfUnknownAsOccupied);
  clearance_process.calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key));
  const auto end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  // Now check obstacle ranges in the map.
  std::cout << "Post query\n";

  // Don't bother saving the cloud. There are no occupied voxels: we treat unknown as occupied.
  if (unknown_as_occupied)
  {
    saveClearanceCloud("ranges-outer-unknown-region.ply", map, glm::i16vec3(0, 0, 0), clearance_process.searchRadius());
  }
  else
  {
    saveClearanceCloud("ranges-outer-region.ply", map, glm::i16vec3(0, 0, 0), clearance_process.searchRadius());
    ohmtools::saveCloud("ranges-outer-cloud.ply", map);
  }

  // Iterate the outer edge of the region. Note that this loop will recheck various faces.
  // Iterate XY, YZ, ZX.
  Voxel<const float> clearance(&map, map.layout().clearanceLayer());
  ASSERT_TRUE(clearance.isLayerValid());
  for (int i = 0; i < 3; ++i)
  {
    const int primary_axis = i;
    const int second_axis = (i + 1) % 3;
    const int third_axis = (i + 2) % 3;
    for (int b = 0; b < region_size[second_axis]; ++b)
    {
      for (int a = 0; a < region_size[primary_axis]; ++a)
      {
        glm::u8vec3 local_key;
        local_key[primary_axis] = a;
        local_key[second_axis] = b;
        local_key[third_axis] = 0;
        clearance.setKey(Key(glm::i16vec3(0), local_key));
        float clearance_value;
        clearance.read(&clearance_value);
        ASSERT_TRUE(clearance.isValid());
        EXPECT_NEAR(clearance_value, map.resolution(), 1e-2f);
        local_key[third_axis] = region_size[third_axis] - 1;
        clearance.setKey(Key(glm::i16vec3(0), local_key));
        ASSERT_TRUE(clearance.isValid());
        EXPECT_NEAR(clearance_value, map.resolution(), 1e-2f);
      }
    }
  }
}


TEST(Ranges, OuterEdgeFromUnknown)
{
  // Test the voxels from the outside propagate into the ROI correctly.
  const double resolution = 1.0;
  const glm::u8vec3 region_size(32);
  const float search_range = float(resolution) * 2;
  OccupancyMap map(resolution, region_size);

  // Build a map in which we clear all voxels except the one at key (0, 0, 0 : 0, 0, 0).
  // We clear region (0, 0, 0) then make sure we enable kQfUnknownAsOccupied.
  // This will treat the boarder voxels as occupied.
  ohmgen::fillMapWithEmptySpace(map, 0, 0, 0, region_size.x, region_size.y, region_size.z);
  testMapOuterEdge(map, search_range, true);
}


TEST(Ranges, OuterEdgeBordered)
{
  // Test the voxels from the outside propagate into the ROI correctly.
  const double resolution = 1.0;
  const glm::u8vec3 region_size(32);
  const float search_range = float(resolution) * 2;

  OccupancyMap map(resolution, region_size);
  // Build an empty cube map where the cube lies just outside the region.
  ohmgen::boxRoom(map, -glm::dvec3(0.5 * resolution) * glm::dvec3(region_size) - glm::dvec3(resolution),
                  glm::dvec3(0.5 * resolution) * glm::dvec3(region_size), 1);
  testMapOuterEdge(map, search_range, false);
}


TEST(Ranges, OuterCorners)
{
  // Add one voxel outside the ROI and ensure it propagates into the ROI.
  const double resolution = 1.0;
  const glm::u8vec3 region_size(32);
  const float search_range = float(resolution) * 8;
  OccupancyMap map(resolution, region_size);

  std::cout << "Populate map: " << std::flush;
  auto start_time = TimingClock::now();

  // Build a map in which we clear all voxels then add an something at key (-1, -1, -1 : 31, 31, 31).
  ohmgen::fillMapWithEmptySpace(map, 0, 0, 0, region_size.x, region_size.y, region_size.z);

  // Add obstacles at all the corners just outside the ROI.
  const glm::i16vec3 origin_offset[] = {
    glm::i16vec3(-1, -1, -1),
    glm::i16vec3(region_size.x, -1, -1),
    glm::i16vec3(-1, region_size.y, -1),
    glm::i16vec3(region_size.x, region_size.y, -1),
    glm::i16vec3(-1, -1, region_size.z),
    glm::i16vec3(region_size.x, -1, region_size.z),
    glm::i16vec3(-1, region_size.y, region_size.z),
    glm::i16vec3(region_size.x, region_size.y, region_size.z),
  };

  {
    Voxel<float> occupancy(&map, map.layout().occupancyLayer());
    ASSERT_TRUE(occupancy.isLayerValid());
    for (auto &&offset : origin_offset)
    {
      Key key(0, 0, 0, 0, 0, 0);
      map.moveKey(key, offset.x, offset.y, offset.z);
      occupancy.setKey(key);
      ASSERT_TRUE(occupancy.isValid());
      occupancy.write(map.occupancyThresholdValue() + map.hitValue());
    }
  }

  auto end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  std::cout << "Obstacle query: " << std::flush;
  start_time = TimingClock::now();
  // No need to sync the map yet.
  Key key(0, 0, 0, 0, 0, 0);

  ClearanceProcess clearance_process(search_range, kQfGpuEvaluate);
  clearance_process.calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key), true);
  end_time = TimingClock::now();
  std::cout << (end_time - start_time) << std::endl;

  // Now check obstacle ranges in the map.
  std::cout << "Post query\n";

  saveClearanceCloud("ranges-outer-corners-region.ply", map, glm::i16vec3(0, 0, 0), clearance_process.searchRadius());
  ohmtools::saveCloud("ranges-outer-corners-cloud.ply", map);

  const Key test_keys[] = { Key(0, 0, 0, 0, 0, 0),
                            Key(0, 0, 0, region_size.x - 1, 0, 0),
                            Key(0, 0, 0, 0, region_size.y - 1, 0),
                            Key(0, 0, 0, region_size.x - 1, region_size.y - 1, 0),
                            Key(0, 0, 0, 0, 0, region_size.z - 1),
                            Key(0, 0, 0, region_size.x - 1, 0, region_size.z - 1),
                            Key(0, 0, 0, 0, region_size.y - 1, region_size.z - 1),
                            Key(0, 0, 0, region_size.x - 1, region_size.y - 1, region_size.z - 1) };

  {
    Voxel<const float> clearance(&map, map.layout().clearanceLayer());
    ASSERT_TRUE(clearance.isLayerValid());
    for (const auto &test_key : test_keys)
    {
      clearance.setKey(test_key);
      float clearance_value;
      clearance.read(&clearance_value);
      ASSERT_TRUE(clearance.isValid());
      EXPECT_NEAR(clearance_value, glm::length(glm::vec3(float(map.resolution()))), 1e-2f);
    }
  }
}


void scalingTest(bool gpu, bool voxel_mean = false, unsigned region_dim = 8)
{
  const double resolution = 0.25;
  const glm::u8vec3 region_size(region_dim);
  const float search_range = 2.0f;

  ohm::Profile profile;
  OccupancyMap map(resolution, region_size, (voxel_mean) ? MapFlag::kVoxelMean : MapFlag::kNone);

  // Offset the map origin so that 0, 0, 0 is the centre of a voxel.
  map.setOrigin(glm::dvec3(-0.5 * resolution));

  // Populate a map with points the following coordinates:
  // 1. (0.5, 0, 0)
  // 2. (0, 0.75, 0)
  // 3. (0, 0, 1)
  // We will focus on the voxel at (0, 0, 0) and use weighting to make it report the closest obstacle as
  // 1, 2 and 3 respectively.

  integrateHit(map, map.voxelKey(glm::dvec3(2 * resolution, 0, 0)));
  integrateHit(map, map.voxelKey(glm::dvec3(0, 3 * resolution, 0)));
  integrateHit(map, map.voxelKey(glm::dvec3(0, 0, 4 * resolution)));

  const char *label = gpu ? "GPU" : "CPU";
  std::cout << "Evaluating " << label << std::endl;
  ohm::ProfileMarker mark(label, &profile);

  ClearanceProcess clearance_process(search_range, kQfGpuEvaluate * !!gpu);

  const auto make_query = [&clearance_process, &map, &profile](const char *context, const glm::vec3 &axis_scaling,
                                                               float expected, bool report_scaling) {
    ohm::ProfileMarker mark("Query", &profile);
    unsigned flags = clearance_process.queryFlags();
    flags &= ~kQfReportUnscaledResults;
    flags |= !report_scaling * kQfReportUnscaledResults;
    clearance_process.setQueryFlags(flags);
    clearance_process.setAxisScaling(axis_scaling);
    clearance_process.reset();
    clearance_process.calculateForExtents(map, glm::dvec3(0), glm::dvec3(0));
    const Key origin_key = map.voxelKey(glm::dvec3(0, 0, 0));
    const Voxel<const float> clearance(&map, map.layout().clearanceLayer(), origin_key);
    ASSERT_TRUE(clearance.isValid());
    float clearance_value;
    clearance.read(&clearance_value);
    EXPECT_NEAR(expected, clearance_value, 1e-2f) << context;
  };

  // Don't report scale results for the first tests.
  // No weighting first. Should get 0.5.
  make_query("no scale", glm::vec3(1.0f), float(2 * resolution), false);

  // De-emphasise X. Should report a range of 0.75.
  make_query("less X", glm::vec3(4, 1.0f, 1.0f), float(3 * resolution), false);

  // Emphasise Z. Should report a range of 1.
  make_query("more Z", glm::vec3(1, 1, 1.0f / 3.0f), float(4 * resolution), false);

  // Now report scaled results.
  make_query("scaled results", glm::vec3(1.1f, 1.1f, 1.0f / 4.0f), float(4 * resolution / 4.0f), true);
}


TEST(Ranges, Scaling)
{
  scalingTest(false);
}


TEST(Ranges, ScalingGpu)
{
  scalingTest(true);
}


TEST(Ranges, VoxelMeanGpu)
{
  // A duplicate of ScalingGpu with voxel mean positioning enabled (but not really used).
  scalingTest(true, true);
}
}  // namespace ranges
