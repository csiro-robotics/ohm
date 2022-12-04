// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmtestcommon/OhmTestUtil.h"

#include <ohm/CalculateSegmentKeys.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/LineQuery.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/VoxelOccupancy.h>
#include <ohmgpu/ClearanceProcess.h>
#include <ohmgpu/OhmGpu.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>

#include <logutil/LogUtil.h>

#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>

#include <gtest/gtest.h>

namespace searialisationtests
{
class ProgressDisplay : public ohm::SerialiseProgress
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


TEST(Serialisation, GpuClearance)
{
  const std::string base_name = "gpu-clearance";
  const std::string map_name = base_name + ".ohm";
  const std::string cloud_name = base_name + "-cloud.ply";
  const std::string clearance_name = base_name + "-clearance.ply";
  // Profile profile;
  int error_code = 0;
  const double boundary_distance = 2.5;
  ohm::OccupancyMap save_map(0.25);
  ohm::OccupancyMap load_map(1);  // Initialise at the wrong resolution. Will be fixed on load.

  // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
  ohmgen::boxRoom(save_map, glm::dvec3(-boundary_distance), glm::dvec3(boundary_distance));

  // Calculate clearance values.
  ohm::ClearanceProcess clearance(1.0f, ohm::kQfGpuEvaluate);
  clearance.update(save_map, 0);

  ProgressDisplay progress;
  std::cout << "Saving" << std::endl;
  error_code = ohm::save(map_name.c_str(), save_map, &progress);
  std::cout << std::endl;
  ASSERT_EQ(error_code, 0);

  ohmtools::saveCloud(cloud_name.c_str(), save_map);
  glm::dvec3 min_ext{};
  glm::dvec3 max_ext{};
  save_map.calculateExtents(&min_ext, &max_ext);
  ohmtools::saveClearanceCloud(clearance_name.c_str(), save_map, min_ext, max_ext, boundary_distance);

  std::cout << "Validate header" << std::endl;
  ohm::MapVersion version;
  error_code = ohm::loadHeader(map_name.c_str(), load_map, &version);
  ASSERT_EQ(error_code, 0);
  ASSERT_EQ(version.major, ohm::kCurrentVersion.major);
  ASSERT_EQ(version.minor, ohm::kCurrentVersion.minor);
  ASSERT_EQ(version.patch, ohm::kCurrentVersion.patch);

  progress.reset();
  std::cout << "Loading" << std::endl;
  error_code = ohm::load(map_name.c_str(), load_map, &progress);
  std::cout << std::endl;
  ASSERT_EQ(error_code, 0);

  ohmtestutil::compareMaps(load_map, save_map, ohmtestutil::kCfCompareExtended | ohmtestutil::kCfExpectClearance);
}


// Legacy code used to generate the test map for Serialisation.Upgrade.
void cubicRoomLegacy(ohm::OccupancyMap &map, float boundary_range, int voxel_step)
{
  int extents = int(boundary_range / map.resolution());

  const auto build_walls = [&map, extents, voxel_step](int a0, int a1, int a2) {
    const double map_res = map.resolution();
    ohm::KeyList ray;
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
          calculateSegmentKeys(ray, map, origin, point, false);
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
}  // namespace searialisationtests
