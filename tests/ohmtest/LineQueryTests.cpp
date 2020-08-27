// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/LineQuery.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/VoxelData.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

using namespace ohm;

namespace linequerytests
{
  typedef std::chrono::high_resolution_clock TimingClock;

  void sparseMap(OccupancyMap &map, bool add_hit = true)
  {
    ohmgen::fillMapWithEmptySpace(map, -64, -64, -64, 63, 63, 63);
    Key key = map.voxelKey(glm::dvec3(0.5 * map.resolution()));
    Voxel<float> voxel(&map, map.layout().occupancyLayer(), key);
    ASSERT_TRUE(voxel.isValid());
    if (add_hit)
    {
      for (int i = 0; i < 1; ++i)
      {
        integrateHit(voxel);
      }
      EXPECT_TRUE(voxel.data() >= 0);
    }
  }

  void lineQueryTest(OccupancyMap &map)
  {
    const double map_res = map.resolution();
    LineQuery query(map, glm::dvec3(0) - glm::dvec3(map_res), glm::dvec3(0), 2.0f,
                    LineQuery::kDefaultFlags | kQfNearestResult);
    query.setStartPoint(glm::dvec3(-2, 0, 0));
    query.setEndPoint(glm::dvec3(2, 0, 0));

    const bool exec_ok = query.execute();
    ASSERT_TRUE(exec_ok);

    ASSERT_GT(query.numberOfResults(), 0);
    EXPECT_EQ(query.numberOfResults(), 1);

    std::cout << "Nearest result: " << query.intersectedVoxels()[0] << " @ " << query.ranges()[0] << std::endl;

    EXPECT_EQ(query.intersectedVoxels()[0], map.voxelKey(glm::dvec3(0)))
      << query.intersectedVoxels()[0] << " != " << map.voxelKey(glm::dvec3(0));
    EXPECT_EQ(query.ranges()[0], 0);

    std::string file_name("line-query-");
    file_name += "cpu";
    file_name += ".ohm";
    ohm::save(file_name.c_str(), map);
  }

  TEST(LineQuery, Cpu)
  {
    OccupancyMap map(0.1);
    sparseMap(map);
    lineQueryTest(map);
  }
}  // namespace linequerytests
