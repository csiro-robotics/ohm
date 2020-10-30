// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include "ohmtestcommon/OhmTestUtil.h"

#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/LineQuery.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/VoxelOccupancy.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>

#include <gtest/gtest.h>

using namespace ohm;

namespace searialisationtests
{
class ProgressDisplay : public SerialiseProgress
{
public:
  unsigned target() const { return target_; }

  void setTarget(unsigned target) { target_ = target; }

  unsigned progress() const { return progress_; }

  void setProgress(unsigned progress) { progress_ = progress; }

  bool quit() const override { return false; }

  void setTargetProgress(unsigned target) override { target_ = target; }
  void incrementProgress(unsigned inc = 1u) override
  {
    const unsigned cur_decimation = 100 * progress_ / target_;
    progress_ += inc;
    const unsigned decimation = 100 * progress_ / target_;
    if (decimation > cur_decimation)
    {
      std::cout << "\r" << progress_ << "/" << target_ << std::flush;
    }
  }

  void reset() { target_ = progress_ = 0; }

private:
  unsigned target_ = 0;
  unsigned progress_ = 0;
};


TEST(Serialisation, Basic)
{
  const char *map_name = "test-map.ohm";
  // Profile profile;
  int error_code = 0;
  const double boundary_distance = 2.5;
  OccupancyMap save_map(0.25);
  OccupancyMap load_map(1);  // Initialise at the wrong resolution. Will be fixed on load.

  // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
  ohmgen::boxRoom(save_map, glm::dvec3(-boundary_distance), glm::dvec3(boundary_distance));

  ProgressDisplay progress;
  std::cout << "Saving" << std::endl;
  error_code = save(map_name, save_map, &progress);
  std::cout << std::endl;
  ASSERT_EQ(error_code, 0);

  std::cout << "Validate header" << std::endl;
  MapVersion version;
  error_code = loadHeader(map_name, load_map, &version);
  ASSERT_EQ(error_code, 0);
  ASSERT_EQ(version.major, kCurrentVersion.major);
  ASSERT_EQ(version.minor, kCurrentVersion.minor);
  ASSERT_EQ(version.patch, kCurrentVersion.patch);

  progress.reset();
  std::cout << "Loading" << std::endl;
  error_code = load(map_name, load_map, &progress);
  std::cout << std::endl;
  ASSERT_EQ(error_code, 0);

  ohmtestutil::compareMaps(load_map, save_map, ohmtestutil::kCfCompareAll);
}


// Legacy code used to generate the test map for Serialisation.Upgrade.
void cubicRoomLegacy(OccupancyMap &map, float boundary_range, int voxel_step)
{
  int extents = int(boundary_range / map.resolution());

  // Set legacy probability values.
  map.setHitProbability(0.7f);
  map.setMissProbability(0.4f);

  const auto build_walls = [&map, extents, voxel_step](int a0, int a1, int a2) {
    const double map_res = map.resolution();
    KeyList ray;
    glm::dvec3 point;
    glm::dvec3 origin = map.origin();
    for (int i = -extents + 1; i <= extents; i += voxel_step)
    {
      for (int j = -extents + 1; j <= extents; j += voxel_step)
      {
        for (int k = 0; k < 2; ++k)
        {
          point = map.origin();
          point[a0] = i * map_res;
          point[a1] = j * map_res;
          point[a2] = (k == 0 ? 1.0 : -1.0) * extents * map_res;
          map.calculateSegmentKeys(ray, origin, point, false);
          for (auto key : ray)
          {
            ohm::integrateMiss(map, key);
          }
          ohm::integrateHit(map, map.voxelKey(point));
        }
      }
    }
  };

  build_walls(0, 1, 2);
  build_walls(1, 2, 0);
  build_walls(0, 2, 1);
}


TEST(Serialisation, Upgrade)
{
  // Test loading older version data files.
  std::string map_name = std::string(ohmtestutil::applicationDir()) + "test-map.0.ohm";
  // Profile profile;
  int error_code = 0;
  const float boundary_distance = 5.0f;
  OccupancyMap test_map(0.25);
  OccupancyMap load_map(1);  // Initialise at the wrong resolution. Will be fixed on load.

  // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
  // Need to use the same code with which test-map.0.ohm was generated.
  cubicRoomLegacy(test_map, boundary_distance, 3);

  std::cout << "Validate header" << std::endl;
  MapVersion version;
  error_code = loadHeader(map_name.c_str(), load_map, &version);
  ASSERT_EQ(error_code, 0);
  ASSERT_EQ(version.major, 0);
  ASSERT_EQ(version.minor, 0);
  ASSERT_EQ(version.patch, 0);

  ProgressDisplay progress;
  std::cout << "Loading" << std::endl;
  error_code = load(map_name.c_str(), load_map, &progress);
  std::cout << std::endl;
  ASSERT_EQ(error_code, 0);

  ohmtestutil::compareMaps(load_map, test_map, ohmtestutil::kCfDefault);
}
}  // namespace searialisationtests
