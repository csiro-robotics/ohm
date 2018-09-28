// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancykey.h"
#include "occupancymap.h"
#include "mapchunk.h"
#include "mapnode.h"

#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

// TODO: Convert from legacy test layout to use gtest macros and features.

using namespace ohm;

namespace keytests
{
  struct SampleInfo
  {
    glm::dvec3 point;
    ohm::OccupancyKey expectedKey;
  };

  static const float epsilon = 0.1f;

  void addSample(std::vector<SampleInfo> &samples, ohm::OccupancyMap &map, const glm::dvec3 p)
  {
    SampleInfo sample;

    sample.point = p;
    sample.expectedKey = map.voxelKey(p);

    samples.push_back(sample);
  }

  void generateInternalSamples(std::vector<SampleInfo> &samples, OccupancyMap &map)
  {
    OccupancyKey originKey;
    OccupancyKey key;
    glm::dvec3 regionSpatialDim;
    glm::dvec3 regionMin, regionMax;

    // We'll use the central region as a base line and generate keys wholy within the bounds of this reagion.
    originKey = map.voxelKey(map.origin());
    regionSpatialDim = map.regionSpatialResolution();

    regionMin = map.origin() - 0.5 * regionSpatialDim;
    regionMax = map.origin() + 0.5 * regionSpatialDim;

    // For internal samples we start by adding points at the corners of the region. Note the
    // maximal corners should be for the next region so we remove epsilon.
    addSample(samples, map, glm::vec3(regionMin.x, regionMin.y, regionMin.z));
    addSample(samples, map, glm::vec3(regionMax.x - epsilon, regionMin.y, regionMin.z));
    addSample(samples, map, glm::vec3(regionMin.x, regionMax.y - epsilon, regionMin.z));
    addSample(samples, map, glm::vec3(regionMax.x - epsilon, regionMax.y - epsilon, regionMin.z));
    addSample(samples, map, glm::vec3(regionMin.x, regionMin.y, regionMax.z - epsilon));
    addSample(samples, map, glm::vec3(regionMax.x - epsilon, regionMin.y, regionMax.z - epsilon));
    addSample(samples, map, glm::vec3(regionMin.x, regionMax.y - epsilon, regionMax.z - epsilon));
    addSample(samples, map, glm::vec3(regionMax.x - epsilon, regionMax.y - epsilon, regionMax.z - epsilon));

    // Add the map origin.
    addSample(samples, map, map.origin());

    key.setRegionKey(originKey.regionKey());
    // Now add the centre of each voxel within the region.
    for (int z = 0; z < map.regionVoxelDimensions().z; ++z)
    {
      key.setLocalAxis(2, z);
      for (int y = 0; y < map.regionVoxelDimensions().y; ++y)
      {
        key.setLocalAxis(1, y);
        for (int x = 0; x < map.regionVoxelDimensions().x; ++x)
        {
          key.setLocalAxis(0, x);
          addSample(samples, map, map.voxelCentreGlobal(key));
        }
      }
    }
  }

  void generateExternalSamples(std::vector<SampleInfo> &samples, OccupancyMap &map)
  {
    glm::dvec3 regionSpatialDim;
    glm::dvec3 regionMin, regionMax;

    // We'll use the central region as a base line and generate keys wholy within the bounds of this reagion.
    regionSpatialDim = map.regionSpatialResolution();

    regionMin = map.origin() - 0.5 * regionSpatialDim;
    regionMax = map.origin() + 0.5 * regionSpatialDim;

    // For external samples we start with the maximal corners (which should lie in adjacent
    // regions), and the minimal corners with an epsilon value shift.
    addSample(samples, map, glm::vec3(regionMin.x - epsilon, regionMin.y - epsilon, regionMin.z - epsilon));
    addSample(samples, map, glm::vec3(regionMax.x, regionMin.y - epsilon, regionMin.z - epsilon));
    addSample(samples, map, glm::vec3(regionMin.x - epsilon, regionMax.y, regionMin.z - epsilon));
    addSample(samples, map, glm::vec3(regionMax.x, regionMax.y, regionMin.z - epsilon));
    addSample(samples, map, glm::vec3(regionMin.x - epsilon, regionMin.y - epsilon, regionMax.z));
    addSample(samples, map, glm::vec3(regionMax.x, regionMin.y - epsilon, regionMax.z));
    addSample(samples, map, glm::vec3(regionMin.x - epsilon, regionMax.y, regionMax.z));
    addSample(samples, map, glm::vec3(regionMax.x, regionMax.y, regionMax.z));
  }

  bool testSamples(const std::vector<SampleInfo> &samples, OccupancyMap &map, const OccupancyKey &referenceKey, bool expectRegionMatch)
  {
    OccupancyKey key;
    glm::dvec3 coord;
    glm::dvec3 separation;
    bool testsOk = true;
    enum FailureFlag
    {
      FF_Zero = 0,
      FF_Region = (1<<0),
      FF_Local = (1<<1),
      FF_Coord = (1<<2),
      FF_ExpectedKey = (1<<3)
    };
    unsigned failures;

    // Validate the internal samples.
    // Don't use range based loop so I can adjust 'i' in the debugger and retest failures.
    for (size_t i = 0; i < samples.size(); ++i)
    {
      const SampleInfo &sample = samples[i];
      failures = 0u;
      key = map.voxelKey(sample.point);
      coord = map.voxelCentreGlobal(key);
      separation = sample.point - coord;

      // Validate results.
      // 1. Region check.
      if (expectRegionMatch && key.regionKey() != referenceKey.regionKey() ||
          !expectRegionMatch && key.regionKey() == referenceKey.regionKey())
      {
        // Region mismatch.
        failures |= FF_Region;
      }

      if (key.localKey().x >= map.regionVoxelDimensions().x ||
          key.localKey().y >= map.regionVoxelDimensions().y ||
          key.localKey().z >= map.regionVoxelDimensions().z)
      {
        failures |= FF_Local;
      }

      if (separation.x < -0.5 * map.resolution() || separation.x >= 0.5 * map.resolution() ||
          separation.y < -0.5 * map.resolution() || separation.y >= 0.5 * map.resolution() ||
          separation.z < -0.5 * map.resolution() || separation.z >= 0.5 * map.resolution())
      {
        failures |= FF_Coord;
      }

      if (key != sample.expectedKey)
      {
        failures |= FF_ExpectedKey;
      }

      if (failures)
      {
        testsOk = false;

        fprintf(stderr, "Sample/key failure: (%g %g %g) : [(%d %d %d) (%u %u %u)]]n",
            sample.point.x, sample.point.y, sample.point.z,
            int(sample.expectedKey.localKey().x), int(sample.expectedKey.localKey().y), int(sample.expectedKey.localKey().z),
            sample.expectedKey.regionKey().x, sample.expectedKey.regionKey().y, sample.expectedKey.regionKey().z
          );

        if (failures & FF_Region)
        {
          fprintf(stderr, "  Incorrect region: (%u %u %u)\n", key.regionKey().x, key.regionKey().y, key.regionKey().z);
        }

        if (failures & FF_Local)
        {
          fprintf(stderr, "  Invalid local index: (%i %i %i) Max (%i %i %i)\n",
              int(key.localKey().x), int(key.localKey().y), int(key.localKey().z),
              int(map.regionVoxelDimensions().x), int(map.regionVoxelDimensions().y), int(map.regionVoxelDimensions().z)
            );
        }

        if (failures & FF_Coord)
        {
          fprintf(stderr, "  Voxel centre too far from sample (%g): S:(%g %g %g) V:(%g %g %g) D:(%g %g %g)\n",
              map.resolution(), sample.point.x, sample.point.y, sample.point.z,
              coord.x, coord.y, coord.z,
              separation.x, separation.y, separation.z
            );
        }

        if (failures & FF_ExpectedKey)
        {
          fprintf(stderr, "  Unexpected key: [(%d %d %d) (%u %u %u)]]\n",
              int(key.localKey().x), int(key.localKey().y), int(key.localKey().z),
              key.regionKey().x, key.regionKey().y, key.regionKey().z
            );
        }
      }
    }

    return testsOk;
  }

  TEST(Keys, Indexing)
  {
    // Define a (dummy) map to work with.
    OccupancyMap map(0.3f);
    OccupancyKey originKey;
    std::vector<SampleInfo> internalSamples;
    std::vector<SampleInfo> externalSamples;
    bool testOk = true;

    // We'll use the central region as a base line and generate keys wholy within the bounds of this reagion.
    originKey = map.voxelKey(map.origin());

    // Now generate a list of points which are expected to line inside and a list of points
    // expected to lie outside the central region.
    generateInternalSamples(internalSamples, map);
    generateExternalSamples(externalSamples, map);

    EXPECT_TRUE(testSamples(internalSamples, map, originKey, true));
    EXPECT_TRUE(testSamples(externalSamples, map, originKey, false));
  }

  // Test conversion from voxel key to centre and back.
  TEST(Keys, Conversion)
  {
    const double resolution = 0.25;
    const glm::u8vec3 regionSize(16);
    OccupancyMap map(resolution, regionSize);
    OccupancyKey key, testKey;
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
          v = map.voxelCentreGlobal(key);
          testKey = map.voxelKey(v);
          EXPECT_EQ(key, testKey);
        }
      }
    }
  }

  template <typename real>
  void quantisationTest(const real coord, const int regionSize, const real voxelResolution)
  {
    int r = ohm::pointToRegionCoord(coord, regionSize * voxelResolution);
    real rmin = (ohm::regionCentreCoord(r, regionSize * voxelResolution) - real(0.5) * regionSize * voxelResolution);
    int v = ohm::pointToRegionVoxel(coord - rmin, voxelResolution, regionSize * voxelResolution);
    EXPECT_LT(v, regionSize);
  }

  // Test conversion from voxel key to centre and back.
  TEST(Keys, Quantisation)
  {
    const int regionSize = 32;
    const double resolution = 0.4;
    const float resolutionF = float(resolution);

    // Attempting to quantise a coordinate at the upper boundary of region -1 would generate a bad voxel coordinate.
    // This is testing that this case is fixed.
    {
      //const float badValue = -6.4000005722045898f;
      const float epsilon = 5e-7f;
      const float badValue = regionSize * resolutionF * -0.5f - epsilon;
      quantisationTest(badValue, regionSize, resolutionF);
    }

    {
      const double epsilon = 1e-15;
      const double badValue = regionSize * resolution * -0.5 - epsilon;
      quantisationTest(badValue, regionSize, resolution);
    }
  }
}
