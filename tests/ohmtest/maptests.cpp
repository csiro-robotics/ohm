// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
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

#include "ohmtestutil.h"
#include <gtest/gtest.h>

using namespace ohm;
using namespace ohmutil;

namespace maptests
{
  TEST(Map, Clone)
  {
    OccupancyMap map(0.25);

    // Generate occupancy.
    ohmgen::cubicRoom(map, 5.0f);

    // Generate clearance values.
    ClearanceProcess refClearance(2.0f, QF_GpuEvaluate);
    refClearance.update(map, 0.0);

    // Clone the map
    std::unique_ptr<OccupancyMap> mapCopy(map.clone());

    // Compare results.
    MapCache cache;

    // Compare maps.
    ohmtestutil::compareMaps(*mapCopy, map, ohmtestutil::CF_CompareAll | ohmtestutil::CF_ExpectClearance);
  }


  TEST(Map, CloneSubmap)
  {
    const glm::dvec3 cloneMin(0);
    const glm::dvec3 cloneMax(4.5);
    OccupancyMap map(0.2);

    // Generate occupancy.
    ohmgen::cubicRoom(map, 5.0f);

    // Generate clearance values.
    ClearanceProcess refClearance(2.0f, QF_GpuEvaluate);
    refClearance.update(map, 0.0);

    // Clone the map
    std::unique_ptr<OccupancyMap> mapCopy(map.clone(cloneMin, cloneMax));

    // Compare results.
    MapCache cache;

    // Compare maps.
    ohmtestutil::compareMaps(*mapCopy, map, cloneMin, cloneMax,
                             ohmtestutil::CF_CompareAll | ohmtestutil::CF_ExpectClearance);
  }
}
