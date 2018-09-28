// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmtestutil.h"

#include "mapcache.h"
#include "occupancygpu.h"
#include "occupancygpumap.h"
#include "occupancykey.h"
#include "occupancykeylist.h"
#include "occupancylinequery.h"
#include "occupancymap.h"
#include "occupancytype.h"
#include "occupancyutil.h"
#include "ohmclearanceprocess.h"
#include "ohmmapper.h"

#include "ohmcloud.h"
#include "ohmgen.h"

#include "ohmutil.h"
#include "profile.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include "occupancymapserialise.h"
#include "ohmdefaultlayers.h"
#include <gtest/gtest.h>

using namespace ohm;
using namespace ohmutil;

namespace mappertests
{
  TEST(Mapper, Clearance)
  {
    Profile profile;
    const double mapExtents = 10.0;
    const double resolution = 0.25;
    const unsigned rayCount = 1024 * 128;
    const unsigned batchSize = 1024 * 2;
    const glm::u8vec3 regionSize(32);
    const float clearanceRange = 3.0f;
    const unsigned clearanceFlags = QF_GpuEvaluate | QF_UnknownAsOccupied;

    // Make some rays.
    std::mt19937 randEngine;
    std::uniform_real_distribution<double> rand(-mapExtents, mapExtents);
    std::vector<glm::dvec3> rays;

    while (rays.size() < rayCount * 2)
    {
      rays.push_back(glm::dvec3(0.05));
      rays.push_back(glm::dvec3(rand(randEngine), rand(randEngine), rand(randEngine)));
    }

    // Setup the map and mapper
    // Test basic map populate using GPU and ensure it matches CPU (close enough).
    OccupancyMap map(resolution, regionSize);
    GpuMap gpuMap(&map, true, unsigned(batchSize * 2));  // Borrow pointer.
    Mapper mapper(&map);

    mapper.addProcess(new ClearanceProcess(clearanceRange, clearanceFlags));

    ASSERT_TRUE(gpuMap.gpuOk());

    std::cout << "Integrating " << rays.size() / 2 << " rays.\n";

    ProfileMarker populateMarker("populate", false);
    ProfileMarker mapperMarker("mapper", false);
    for (size_t i = 0; i < rays.size(); i += batchSize * 2)
    {
      populateMarker.restart();
      const unsigned pointCount = unsigned(std::min<size_t>(batchSize * 2, rays.size() - i));
      gpuMap.integrateRays(rays.data() + i, pointCount);
      populateMarker.end();
      mapperMarker.restart();
      // Partial update unless this is the last update. Full update for the last update.s
      mapper.update(0.01);
      mapperMarker.end();
    }
    populateMarker.end();

    {
      std::cout << "Finalising clearance" << std::endl;
      ProfileMarker mapperFinaliseMarker("mapper-finalise");
      mapper.update(0.0);
    }
    {
      ProfileMarker syncMarker("sync");
      std::cout << "Syncing occupancy" << std::endl;
      gpuMap.syncOccupancy();
    }

    // Debug feature
    static bool saveMap = false;
    if (saveMap)
    {
      std::cout << "Saving map" << std::endl;
      ohm::save("mapper.ohm", map);
      //ohmtools::saveClearanceCloud("mapper-clearance.ply", map, glm::dvec3(-10), glm::dvec3(10), clearanceRange);
    }

    // Clone the map and calculate clearance values accross the whole map for verification.
    std::cout << "Cloning map" << std::endl;
    std::unique_ptr<OccupancyMap> refMap(map.clone());
    refMap->touch();  // Force clearance recalculation.
    // FIXME: we are not recalculating the clearance values. The stamping prevents this.
    {
      std::cout << "Calculating cloned clearance" << std::endl;
      ClearanceProcess refClearance(clearanceRange, clearanceFlags);
      ProfileMarker clearanceOnlyMarker("clearanceOnly");
      refClearance.update(*refMap, 0.0);
    }

    // Compare maps.
    std::cout << "Comparing maps" << std::endl;
    ohmtestutil::compareMaps(map, *refMap, ohmtestutil::CF_CompareAll | ohmtestutil::CF_ExpectClearance);

    ohm::save("map1.ohm", map);
    ohm::save("ref1.ohm", *refMap);
  }
}
