// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/Key.h>
#include <ohm/MapChunk.h>
#include <ohm/MapCoord.h>
#include <ohm/OccupancyMap.h>

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
    ohm::Key expected_key;
  };

  static const float kEpsilon = 0.1f;

  void addSample(std::vector<SampleInfo> &samples, ohm::OccupancyMap &map, const glm::dvec3 p)
  {
    SampleInfo sample;

    sample.point = p;
    sample.expected_key = map.voxelKey(p);

    samples.push_back(sample);
  }

  void generateInternalSamples(std::vector<SampleInfo> &samples, OccupancyMap &map)
  {
    Key origin_key;
    Key key;
    glm::dvec3 region_spatial_dim;
    glm::dvec3 region_min, region_max;

    // We'll use the central region as a base line and generate keys wholy within the bounds of this reagion.
    origin_key = map.voxelKey(map.origin());
    region_spatial_dim = map.regionSpatialResolution();

    region_min = map.origin() - 0.5 * region_spatial_dim;
    region_max = map.origin() + 0.5 * region_spatial_dim;

    // For internal samples we start by adding points at the corners of the region. Note the
    // maximal corners should be for the next region so we remove epsilon.
    addSample(samples, map, glm::vec3(region_min.x, region_min.y, region_min.z));
    addSample(samples, map, glm::vec3(region_max.x - kEpsilon, region_min.y, region_min.z));
    addSample(samples, map, glm::vec3(region_min.x, region_max.y - kEpsilon, region_min.z));
    addSample(samples, map, glm::vec3(region_max.x - kEpsilon, region_max.y - kEpsilon, region_min.z));
    addSample(samples, map, glm::vec3(region_min.x, region_min.y, region_max.z - kEpsilon));
    addSample(samples, map, glm::vec3(region_max.x - kEpsilon, region_min.y, region_max.z - kEpsilon));
    addSample(samples, map, glm::vec3(region_min.x, region_max.y - kEpsilon, region_max.z - kEpsilon));
    addSample(samples, map, glm::vec3(region_max.x - kEpsilon, region_max.y - kEpsilon, region_max.z - kEpsilon));

    // Add the map origin.
    addSample(samples, map, map.origin());

    key.setRegionKey(origin_key.regionKey());
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
    glm::dvec3 region_spatial_dim;
    glm::dvec3 region_min, region_max;

    // We'll use the central region as a base line and generate keys wholy within the bounds of this reagion.
    region_spatial_dim = map.regionSpatialResolution();

    region_min = map.origin() - 0.5 * region_spatial_dim;
    region_max = map.origin() + 0.5 * region_spatial_dim;

    // For external samples we start with the maximal corners (which should lie in adjacent
    // regions), and the minimal corners with an epsilon value shift.
    addSample(samples, map, glm::vec3(region_min.x - kEpsilon, region_min.y - kEpsilon, region_min.z - kEpsilon));
    addSample(samples, map, glm::vec3(region_max.x, region_min.y - kEpsilon, region_min.z - kEpsilon));
    addSample(samples, map, glm::vec3(region_min.x - kEpsilon, region_max.y, region_min.z - kEpsilon));
    addSample(samples, map, glm::vec3(region_max.x, region_max.y, region_min.z - kEpsilon));
    addSample(samples, map, glm::vec3(region_min.x - kEpsilon, region_min.y - kEpsilon, region_max.z));
    addSample(samples, map, glm::vec3(region_max.x, region_min.y - kEpsilon, region_max.z));
    addSample(samples, map, glm::vec3(region_min.x - kEpsilon, region_max.y, region_max.z));
    addSample(samples, map, glm::vec3(region_max.x, region_max.y, region_max.z));
  }

  bool testSamples(const std::vector<SampleInfo> &samples, OccupancyMap &map, const Key &reference_key,
                   bool expect_region_match)
  {
    Key key;
    glm::dvec3 coord;
    glm::dvec3 separation;
    bool tests_ok = true;
    enum FailureFlag
    {
      kFfZero = 0,
      kFfRegion = (1 << 0),
      kFfLocal = (1 << 1),
      kFfCoord = (1 << 2),
      kFfExpectedKey = (1 << 3)
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
      if (expect_region_match && key.regionKey() != reference_key.regionKey() ||
          !expect_region_match && key.regionKey() == reference_key.regionKey())
      {
        // Region mismatch.
        failures |= kFfRegion;
      }

      if (key.localKey().x >= map.regionVoxelDimensions().x || key.localKey().y >= map.regionVoxelDimensions().y ||
          key.localKey().z >= map.regionVoxelDimensions().z)
      {
        failures |= kFfLocal;
      }

      if (separation.x < -0.5 * map.resolution() || separation.x >= 0.5 * map.resolution() ||
          separation.y < -0.5 * map.resolution() || separation.y >= 0.5 * map.resolution() ||
          separation.z < -0.5 * map.resolution() || separation.z >= 0.5 * map.resolution())
      {
        failures |= kFfCoord;
      }

      if (key != sample.expected_key)
      {
        failures |= kFfExpectedKey;
      }

      if (failures)
      {
        tests_ok = false;

        fprintf(stderr, "Sample/key failure: (%g %g %g) : [(%d %d %d) (%u %u %u)]]n", sample.point.x, sample.point.y,
                sample.point.z, int(sample.expected_key.localKey().x), int(sample.expected_key.localKey().y),
                int(sample.expected_key.localKey().z), sample.expected_key.regionKey().x,
                sample.expected_key.regionKey().y, sample.expected_key.regionKey().z);

        if (failures & kFfRegion)
        {
          fprintf(stderr, "  Incorrect region: (%u %u %u)\n", key.regionKey().x, key.regionKey().y, key.regionKey().z);
        }

        if (failures & kFfLocal)
        {
          fprintf(stderr, "  Invalid local index: (%i %i %i) Max (%i %i %i)\n", int(key.localKey().x),
                  int(key.localKey().y), int(key.localKey().z), int(map.regionVoxelDimensions().x),
                  int(map.regionVoxelDimensions().y), int(map.regionVoxelDimensions().z));
        }

        if (failures & kFfCoord)
        {
          fprintf(stderr, "  Voxel centre too far from sample (%g): S:(%g %g %g) V:(%g %g %g) D:(%g %g %g)\n",
                  map.resolution(), sample.point.x, sample.point.y, sample.point.z, coord.x, coord.y, coord.z,
                  separation.x, separation.y, separation.z);
        }

        if (failures & kFfExpectedKey)
        {
          fprintf(stderr, "  Unexpected key: [(%d %d %d) (%u %u %u)]]\n", int(key.localKey().x), int(key.localKey().y),
                  int(key.localKey().z), key.regionKey().x, key.regionKey().y, key.regionKey().z);
        }
      }
    }

    return tests_ok;
  }

  TEST(Keys, Indexing)
  {
    // Define a (dummy) map to work with.
    OccupancyMap map(0.3f);
    Key origin_key;
    std::vector<SampleInfo> internal_samples;
    std::vector<SampleInfo> external_samples;

    // We'll use the central region as a base line and generate keys wholy within the bounds of this reagion.
    origin_key = map.voxelKey(map.origin());

    // Now generate a list of points which are expected to line inside and a list of points
    // expected to lie outside the central region.
    generateInternalSamples(internal_samples, map);
    generateExternalSamples(external_samples, map);

    EXPECT_TRUE(testSamples(internal_samples, map, origin_key, true));
    EXPECT_TRUE(testSamples(external_samples, map, origin_key, false));
  }

  // Test conversion from voxel key to centre and back.
  TEST(Keys, Conversion)
  {
    const double resolution = 0.25;
    const glm::u8vec3 region_size(16);
    OccupancyMap map(resolution, region_size);
    Key key, test_key;
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
          v = map.voxelCentreGlobal(key);
          test_key = map.voxelKey(v);
          EXPECT_EQ(key, test_key);
        }
      }
    }
  }

  template <typename REAL>
  void quantisationTest(const REAL coord, const int region_size, const REAL voxel_resolution)
  {
    int r = ohm::pointToRegionCoord(coord, region_size * voxel_resolution);
    REAL rmin =
      (ohm::regionCentreCoord(r, region_size * voxel_resolution) - REAL(0.5) * region_size * voxel_resolution);
    int v = ohm::pointToRegionVoxel(coord - rmin, voxel_resolution, region_size * voxel_resolution);
    EXPECT_LT(v, region_size);
  }

  // Test conversion from voxel key to centre and back.
  TEST(Keys, Quantisation)
  {
    const int region_size = 32;
    const double resolution = 0.4;
    const float resolution_f = float(resolution);

    // Attempting to quantise a coordinate at the upper boundary of region -1 would generate a bad voxel coordinate.
    // This is testing that this case is fixed.
    {
      // const float badValue = -6.4000005722045898f;
      const float epsilon = 5e-7f;
      const float bad_value = region_size * resolution_f * -0.5f - epsilon;
      quantisationTest(bad_value, region_size, resolution_f);
    }

    {
      const double epsilon = 1e-15;
      const double bad_value = region_size * resolution * -0.5 - epsilon;
      quantisationTest(bad_value, region_size, resolution);
    }
  }
}  // namespace keytests
