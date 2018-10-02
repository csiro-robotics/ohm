// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/mapcache.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancyqueryflag.h>
#include <ohm/occupancyutil.h>
#include <ohm/ohmclearanceprocess.h>
#include <ohm/mapprobability.h>
#include <ohm/mapchunk.h>
#include <ohm/maplayout.h>
#include <ohm/occupancygpumap.h>

#include <ohmtools/ohmgen.h>
#include <ohmtools/ohmcloud.h>

#include <ohmutil/ohmutil.h>
#include <ohmutil/profile.h>
#include <ohmutil/plymesh.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <random>

using namespace ohm;

namespace ranges
{
  typedef std::chrono::high_resolution_clock timing_clock;


  void saveClearanceCloud(const char *fileName, const OccupancyMap &map, const glm::i16vec3 &region, float searchRadius)
  {
    PlyMesh ply;
    OccupancyKey key(region, 0, 0, 0);
    glm::vec3 voxelPos;
    auto regionSize = map.regionVoxelDimensions();
    // std::cout << "regionSize: " << regionSize << std::endl;

    OccupancyNodeConst node = map.node(key);
    if (node.isValid())
    {
      do
      {
        uint8_t r = 255;
        uint8_t g = 128;
        uint8_t b = 0;
        float rangeValue = node.clearance();
        if (rangeValue < 0)
        {
          rangeValue = searchRadius;
        }
        r = (uint8_t)(255 * std::max(0.0f, (searchRadius - rangeValue) / searchRadius));
        voxelPos = map.voxelCentreGlobal(node.key());
        ply.addVertex(voxelPos, Colour(r, g, b));
      } while (node.nextInRegion());
    }

    ply.save(fileName, true);
  }


  void showVoxel(ohm::OccupancyMap &map, const ohm::OccupancyKey &key, float expectedRange = FLT_MAX)
  {
    const OccupancyNodeConst node = map.node(key, false);
    if (node.isValid())
    {
      std::cout << node.key() << " " << node.value() << ": " << node.clearance() << '\n';
    }
    else
    {
      std::cout << key << " invalid\n";
    }

    if (expectedRange != FLT_MAX)
    {
      EXPECT_TRUE(node.isValid());
      if (node.isValid())
      {
        EXPECT_NEAR(node.clearance(), expectedRange, 1e-3f);
      }
    }
  };


  TEST(Ranges, Simple)
  {
    const double resolution = 1.0;
    const glm::u8vec3 regionSize(32);
    // Deliberately require search padding of more than one region to validate GPU memory alignment.
    const float searchRange = (float)resolution * (float)regionSize.x;
    OccupancyMap map(resolution, regionSize);

    MapCache cache;

    std::cout << "Populate map: " << std::flush;
    auto startTime = timing_clock::now();

    // Build a map in which we clear all voxels except the one at key (0, 0, 0 : 0, 0, 0).
    // We clear region (0, 0, 0) and make sure we don't query with the flag QF_UnknownAsOccupied.
    ohmgen::fillMapWithEmptySpace(map, 0, 0, 0, regionSize.x, regionSize.y, regionSize.z);
    auto occupiedVoxel = map.node(OccupancyKey(0, 0, 0, 0, 0, 0), true);
    occupiedVoxel.setValue(map.occupancyThresholdValue());

    auto endTime = timing_clock::now();
    std::cout << (endTime - startTime) << std::endl;

    std::cout << "Obstacle query: " << std::flush;
    startTime = timing_clock::now();
    // No need to sync the map yet.
    OccupancyKey key(0, 0, 0, 0, 0, 0);

    ClearanceProcess clearanceProcess(searchRange, QF_GpuEvaluate);
    clearanceProcess.calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key));
    endTime = timing_clock::now();
    std::cout << (endTime - startTime) << std::endl;

    // Now check obstacle ranges in the map.
    std::cout << "Post query\n";
    key = OccupancyKey(0, 0, 0, 0, 0, 0);
    showVoxel(map, key, 0.0f);
    map.stepKey(key, 0, 1);
    showVoxel(map, key, 1.0f);
    map.stepKey(key, 0, 1);
    showVoxel(map, key, 2.0f);

    saveClearanceCloud("ranges-simple-region.ply", map, glm::i16vec3(0, 0, 0), clearanceProcess.searchRadius());
    ohmtools::saveCloud("ranges-simple-cloud.ply", map);

    key = OccupancyKey(0, 0, 0, 0, 0, 0);
    OccupancyNodeConst node = map.node(key);
    // const float *values = (const float *)map.layout().layer(1).voxels(*map.region(glm::i16vec3(0, 0, 0)));
    ASSERT_TRUE(node.isValid());
    const unsigned maxFailures = 20;
    unsigned failureCount = 0;
    do
    {
      ASSERT_TRUE(node.isValid());
      const float epsilon = 1e-3f;
      const float distToRegionOrigin = glm::length(glm::vec3(node.key().localKey()));
      const float expectedRange = (distToRegionOrigin <= clearanceProcess.searchRadius()) ? distToRegionOrigin : -1.0f;
      const float clearance = node.clearance();
      if (std::abs(expectedRange - clearance) > epsilon)
      {
        std::cout << "Fail: " << node.key() << ' ' << node.value() << " to RO: " << distToRegionOrigin << " expect: " << expectedRange << " actual: " << clearance << '\n';
        ++failureCount;
      }
      EXPECT_NEAR(expectedRange, clearance, epsilon);
    } while (node.nextInRegion() && failureCount < maxFailures);

    if (failureCount == maxFailures)
    {
      FAIL() << "Too many failure";
    }
  }


  void testMapOuterEdge(OccupancyMap &map, float searchRange, bool unknownAsOccupied)
  {
    const glm::u8vec3 regionSize = map.regionVoxelDimensions();
    auto startTime = timing_clock::now();

    std::cout << "Obstacle query: " << std::flush;
    // No need to sync the map yet.
    OccupancyKey key(0, 0, 0, 0, 0, 0);

    ClearanceProcess clearanceProcess(searchRange, QF_GpuEvaluate | !!unknownAsOccupied * QF_UnknownAsOccupied);
    clearanceProcess.calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key));
    auto endTime = timing_clock::now();
    std::cout << (endTime - startTime) << std::endl;

    // Now check obstacle ranges in the map.
    std::cout << "Post query\n";

    // Don't bother saving the cloud. There are no occupied voxels: we treat unknown as occupied.
    if (unknownAsOccupied)
    {
      saveClearanceCloud("ranges-outer-unknown-region.ply", map, glm::i16vec3(0, 0, 0), clearanceProcess.searchRadius());
    }
    else
    {
      saveClearanceCloud("ranges-outer-region.ply", map, glm::i16vec3(0, 0, 0), clearanceProcess.searchRadius());
      ohmtools::saveCloud("ranges-outer-cloud.ply", map);
    }

    // Iterate the outer edge of the region. Note that this loop will recheck various faces.
    // Iterate XY, YZ, ZX.
    MapCache cache;
    for (int i = 0; i < 3; ++i)
    {
      int primaryAxis = i;
      int secondAxis = (i + 1) % 3;
      int thirdAxis = (i + 2) % 3;
      for (int b = 0; b < regionSize[secondAxis]; ++b)
      {
        for (int a = 0; a < regionSize[primaryAxis]; ++a)
        {
          glm::u8vec3 localKey;
          localKey[primaryAxis] = a;
          localKey[secondAxis] = b;
          localKey[thirdAxis] = 0;
          auto voxel = map.node(OccupancyKey(glm::i16vec3(0), localKey), false, &cache);
          ASSERT_TRUE(voxel.isValid());
          EXPECT_NEAR(voxel.clearance(), map.resolution(), 1e-2f);
          localKey[thirdAxis] = regionSize[thirdAxis] - 1;
          voxel = map.node(OccupancyKey(glm::i16vec3(0), localKey), false, &cache);
          ASSERT_TRUE(voxel.isValid());
          EXPECT_NEAR(voxel.clearance(), map.resolution(), 1e-2f);
        }
      }
    }
  }


  TEST(Ranges, OuterEdgeFromUnknown)
  {
    // Test the voxels from the outside propagate into the ROI correctly.
    const double resolution = 1.0;
    const glm::u8vec3 regionSize(32);
    const float searchRange = float(resolution) * 2;
    OccupancyMap map(resolution, regionSize);

    // Build a map in which we clear all voxels except the one at key (0, 0, 0 : 0, 0, 0).
    // We clear region (0, 0, 0) then make sure we enable QF_UnknownAsOccupied.
    // This will treat the boarder voxels as occupied.
    ohmgen::fillMapWithEmptySpace(map, 0, 0, 0, regionSize.x, regionSize.y, regionSize.z);
    testMapOuterEdge(map, searchRange, true);
  }


  TEST(Ranges, OuterEdgeBordered)
  {
    // Test the voxels from the outside propagate into the ROI correctly.
    const double resolution = 1.0;
    const glm::u8vec3 regionSize(32);
    const float searchRange = float(resolution) * 2;
    OccupancyMap map(resolution, regionSize);

    // Build an empty cube map where the cube lies just outside the region.
    ohmgen::cubicRoom(map, float(resolution) * (1 + regionSize.x / 2), 1);
    testMapOuterEdge(map, searchRange, false);
  }


  TEST(Ranges, OuterCorners)
  {
    // Add one voxel outside the ROI and ensure it propagates into the ROI.
    const double resolution = 1.0;
    const glm::u8vec3 regionSize(32);
    const float searchRange = (float)resolution * 8;
    OccupancyMap map(resolution, regionSize);

    MapCache cache;

    std::cout << "Populate map: " << std::flush;
    auto startTime = timing_clock::now();

    // Build a map in which we clear all voxels then add an something at key (-1, -1, -1 : 31, 31, 31).
    ohmgen::fillMapWithEmptySpace(map, 0, 0, 0, regionSize.x, regionSize.y, regionSize.z);

    // Add obstacles at all the corners just outside the ROI.
    const glm::i16vec3 originOffset[] =
    {
      glm::i16vec3(-1, -1, -1),
      glm::i16vec3(regionSize.x, -1, -1),
      glm::i16vec3(-1, regionSize.y, -1),
      glm::i16vec3(regionSize.x, regionSize.y, -1),
      glm::i16vec3(-1, -1, regionSize.z),
      glm::i16vec3(regionSize.x, -1, regionSize.z),
      glm::i16vec3(-1, regionSize.y, regionSize.z),
      glm::i16vec3(regionSize.x, regionSize.y, regionSize.z),
    };

    for (auto &&offset : originOffset)
    {
      OccupancyKey key(0, 0, 0, 0, 0, 0);
      map.moveKey(key, offset.x, offset.y, offset.z);
      map.node(key, true).setValue(map.occupancyThresholdValue() + map.hitValue());
    }

    auto endTime = timing_clock::now();
    std::cout << (endTime - startTime) << std::endl;

    std::cout << "Obstacle query: " << std::flush;
    startTime = timing_clock::now();
    // No need to sync the map yet.
    OccupancyKey key(0, 0, 0, 0, 0, 0);

    ClearanceProcess clearanceProcess(searchRange, QF_GpuEvaluate);
    clearanceProcess.calculateForExtents(map, map.voxelCentreGlobal(key), map.voxelCentreGlobal(key), true);
    endTime = timing_clock::now();
    std::cout << (endTime - startTime) << std::endl;

    // Now check obstacle ranges in the map.
    std::cout << "Post query\n";

    saveClearanceCloud("ranges-outer-corners-region.ply", map, glm::i16vec3(0, 0, 0), clearanceProcess.searchRadius());
    ohmtools::saveCloud("ranges-outer-corners-cloud.ply", map);

    const OccupancyKey testKeys[] =
    {
      OccupancyKey(0, 0, 0, 0, 0, 0),
      OccupancyKey(0, 0, 0, regionSize.x - 1, 0, 0),
      OccupancyKey(0, 0, 0, 0, regionSize.y - 1, 0),
      OccupancyKey(0, 0, 0, regionSize.x - 1, regionSize.y - 1, 0),
      OccupancyKey(0, 0, 0, 0, 0, regionSize.z - 1),
      OccupancyKey(0, 0, 0, regionSize.x - 1, 0, regionSize.z - 1),
      OccupancyKey(0, 0, 0, 0, regionSize.y - 1, regionSize.z - 1),
      OccupancyKey(0, 0, 0, regionSize.x - 1, regionSize.y - 1, regionSize.z - 1)
    };

    for (const auto &testKey : testKeys)
    {
      auto voxel = map.node(testKey);
      ASSERT_TRUE(voxel.isValid());
      EXPECT_NEAR(voxel.clearance(), glm::length(glm::vec3(float(map.resolution()))), 1e-2f);
    }
  }


  void scalingTest(bool gpu)
  {
    const double resolution = 0.25;
    const glm::u8vec3 regionSize(32);
    const float searchRange = 2.0f;

    ohmutil::Profile profile;
    OccupancyMap map(resolution, regionSize);

    // Offset the map origin so that 0, 0, 0 is the centre of a voxel.
    map.setOrigin(glm::dvec3(-0.5 * resolution));

    // Populate a map with points the following coordinates:
    // 1. (0.5, 0, 0)
    // 2. (0, 0.75, 0)
    // 3. (0, 0, 1)
    // We will focus on the voxel at (0, 0, 0) and use weighting to make it report the closest obstacle as
    // 1, 2 and 3 respectively.

    OccupancyKey originKey = map.voxelKey(glm::dvec3(0, 0, 0));

    map.integrateHit(glm::dvec3(2 * resolution, 0, 0));
    map.integrateHit(glm::dvec3(0, 3 * resolution, 0));
    map.integrateHit(glm::dvec3(0, 0, 4 * resolution));

    const char *label = gpu ? "GPU" : "CPU";
    std::cout << "Evaluating " << label << std::endl;
    ohmutil::ProfileMarker mark(label, &profile);

    ClearanceProcess clearanceProcess(searchRange, QF_GpuEvaluate * !!gpu);
    OccupancyNodeConst node = map.node(originKey);

    auto makeQuery = [&clearanceProcess, &map, &node, &profile]
    (const char *context, const glm::vec3 &axisScaling, float expected, bool reportScaling)
    {
      ohmutil::ProfileMarker mark("Query", &profile);
      unsigned flags = clearanceProcess.queryFlags();
      flags &= ~QF_ReportUnscaledResults;
      flags |= !reportScaling * QF_ReportUnscaledResults;
      clearanceProcess.setQueryFlags(flags);
      clearanceProcess.setAxisScaling(axisScaling);
      clearanceProcess.reset();
      clearanceProcess.calculateForExtents(map, glm::dvec3(0), glm::dvec3(0));
      const float clearance = node.clearance();
      EXPECT_NEAR(expected, clearance, 1e-2f) << context;
    };

    // Don't report scale results for the first tests.
    // No weighting first. Should get 0.5.
    makeQuery("no scale", glm::vec3(1.0f), float(2 * resolution), false);

    // De-emphasise X. Should report a range of 0.75.
    makeQuery("less X", glm::vec3(4, 1.0f, 1.0f), float(3 * resolution), false);

    // Emphasise Z. Should report a range of 1.
    makeQuery("more Z", glm::vec3(1, 1, 1.0f / 3.0f), float(4 * resolution), false);

    // Now report scaled results.
    makeQuery("scaled results", glm::vec3(1.1f, 1.1f, 1.0f / 4.0f), float(4 * resolution / 4.0f), true);
  }


  TEST(Ranges, Scaling)
  {
    scalingTest(false);
  }


  TEST(Ranges, ScalingGpu)
  {
    scalingTest(true);
  }
}
