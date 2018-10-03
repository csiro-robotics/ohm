// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestUtil.h"

#include <ohm/MapCache.h>
#include <ohm/OhmGpu.h>
#include <ohm/GpuMap.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/LineQuery.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/ClearanceProcess.h>
#include <ohm/Mapper.h>
#include <ohm/MapSerialise.h>
#include <ohm/DefaultLayers.h>

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
using namespace ohmutil;

namespace mappertests
{
  TEST(Mapper, Clearance)
  {
    Profile profile;
    const double map_extents = 10.0;
    const double resolution = 0.25;
    const unsigned ray_count = 1024 * 128;
    const unsigned batch_size = 1024 * 2;
    const glm::u8vec3 region_size(32);
    const float clearance_range = 3.0f;
    const unsigned clearance_flags = kQfGpuEvaluate | kQfUnknownAsOccupied;

    // Make some rays.
    std::mt19937 rand_engine;
    std::uniform_real_distribution<double> rand(-map_extents, map_extents);
    std::vector<glm::dvec3> rays;

    while (rays.size() < ray_count * 2)
    {
      rays.push_back(glm::dvec3(0.05));
      rays.push_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
    }

    // Setup the map and mapper
    // Test basic map populate using GPU and ensure it matches CPU (close enough).
    OccupancyMap map(resolution, region_size);
    GpuMap gpu_map(&map, true, unsigned(batch_size * 2));  // Borrow pointer.
    Mapper mapper(&map);

    mapper.addProcess(new ClearanceProcess(clearance_range, clearance_flags));

    ASSERT_TRUE(gpu_map.gpuOk());

    std::cout << "Integrating " << rays.size() / 2 << " rays.\n";

    ProfileMarker populate_marker("populate", false);
    ProfileMarker mapper_marker("mapper", false);
    for (size_t i = 0; i < rays.size(); i += batch_size * 2)
    {
      populate_marker.restart();
      const unsigned point_count = unsigned(std::min<size_t>(batch_size * 2, rays.size() - i));
      gpu_map.integrateRays(rays.data() + i, point_count);
      populate_marker.end();
      mapper_marker.restart();
      // Partial update unless this is the last update. Full update for the last update.s
      mapper.update(0.01);
      mapper_marker.end();
    }
    populate_marker.end();

    {
      std::cout << "Finalising clearance" << std::endl;
      ProfileMarker mapper_finalise_marker("mapper-finalise");
      mapper.update(0.0);
    }
    {
      ProfileMarker sync_marker("sync");
      std::cout << "Syncing occupancy" << std::endl;
      gpu_map.syncOccupancy();
    }

    // Debug feature
    static bool save_map = false;
    if (save_map)
    {
      std::cout << "Saving map" << std::endl;
      ohm::save("mapper.ohm", map);
      //ohmtools::saveClearanceCloud("mapper-clearance.ply", map, glm::dvec3(-10), glm::dvec3(10), clearanceRange);
    }

    // Clone the map and calculate clearance values accross the whole map for verification.
    std::cout << "Cloning map" << std::endl;
    std::unique_ptr<OccupancyMap> ref_map(map.clone());
    ref_map->touch();  // Force clearance recalculation.
    // FIXME: we are not recalculating the clearance values. The stamping prevents this.
    {
      std::cout << "Calculating cloned clearance" << std::endl;
      ClearanceProcess ref_clearance(clearance_range, clearance_flags);
      ProfileMarker clearance_only_marker("clearanceOnly");
      ref_clearance.update(*ref_map, 0.0);
    }

    // Compare maps.
    std::cout << "Comparing maps" << std::endl;
    ohmtestutil::compareMaps(map, *ref_map, ohmtestutil::kCfCompareAll | ohmtestutil::kCfExpectClearance);

    ohm::save("map1.ohm", map);
    ohm::save("ref1.ohm", *ref_map);
  }
}
