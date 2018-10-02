// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmtestutil.h"

#include <ohm/occupancygpu.h>
#include <ohm/occupancykey.h>
#include <ohm/occupancykeylist.h>
#include <ohm/occupancylinequery.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancymapserialise.h>
#include <ohm/occupancytype.h>
#include <ohm/occupancyutil.h>
#include <ohm/ohmclearanceprocess.h>
#include <ohm/mapcache.h>

#include <ohmtools/ohmcloud.h>
#include <ohmtools/ohmgen.h>

#include <ohmutil/ohmutil.h>
#include <ohmutil/profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>

#include <gtest/gtest.h>

using namespace ohm;
using namespace ohmutil;

// TODO: move declaration into a shared header.
const char *dataRelPath();

namespace searialisationtests
{
  class ProgressDisplay : public SerialiseProgress
  {
  public:
    unsigned target() const { return _target; }

    void set_target(unsigned target) { _target = target; }

    unsigned progress() const { return _progress; }

    void set_progress(unsigned progress) { _progress = progress; }

    bool quit() const override { return false; }

    void setTargetProgress(unsigned target) override { _target = target; }
    void incrementProgress(unsigned inc = 1) override
    {
      const unsigned curDecimation = 100 * _progress / _target;
      _progress += inc;
      const unsigned decimation = 100 * _progress / _target;
      if (decimation > curDecimation)
      {
        std::cout << "\r" << _progress << "/" << _target << std::flush;
      }
    }

    void reset() { _target = _progress = 0; }

  private:
    unsigned _target = 0;
    unsigned _progress = 0;
  };


  TEST(Serialisation, Basic)
  {
    const char *MapName = "test-map.ohm";
    // Profile profile;
    int errorCode = 0;
    const float boundaryDistance = 5.0f;
    OccupancyMap saveMap(0.25);
    OccupancyMap loadMap(1);  // Initialise at the wrong resolution. Will be fixed on load.

    // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
    ohmgen::cubicRoom(saveMap, boundaryDistance, 3);

    // Calculate clearance values.
    ClearanceProcess clearance(1.0f, QF_GpuEvaluate);
    clearance.update(saveMap, 0);

    ProgressDisplay progress;
    std::cout << "Saving" << std::endl;
    errorCode = save(MapName, saveMap, &progress);
    std::cout << std::endl;
    ASSERT_EQ(errorCode, 0);

    std::cout << "Validate header" << std::endl;
    MapVersion version;
    errorCode = loadHeader(MapName, loadMap, &version);
    ASSERT_EQ(errorCode, 0);
    ASSERT_EQ(version.major, CurrentVersion.major);
    ASSERT_EQ(version.minor, CurrentVersion.minor);
    ASSERT_EQ(version.patch, CurrentVersion.patch);

    progress.reset();
    std::cout << "Loading" << std::endl;
    errorCode = load(MapName, loadMap, &progress);
    std::cout << std::endl;
    ASSERT_EQ(errorCode, 0);

    ohmtestutil::compareMaps(loadMap, saveMap, ohmtestutil::CF_CompareAll | ohmtestutil::CF_ExpectClearance);
  }


  // Legacy code used to generate the test map for Serialisation.Upgrade.
  void cubicRoomLegacy(OccupancyMap &map, float boundaryRange, int voxelStep)
  {
    int extents = int(boundaryRange / map.resolution());

    auto buildWalls = [&map, extents, voxelStep](int a0, int a1, int a2)
    {
      const double mapRes = map.resolution();
      OccupancyNodeConst node;
      OccupancyKey key;
      OccupancyKeyList ray;
      glm::dvec3 point;
      glm::dvec3 origin = map.origin();
      for (int i = -extents + 1; i <= extents; i += voxelStep)
      {
        for (int j = -extents + 1; j <= extents; j += voxelStep)
        {
          for (int k = 0; k < 2; ++k)
          {
            point = map.origin();
            point[a0] = i * mapRes;
            point[a1] = j * mapRes;
            point[a2] = (k == 0 ? 1.0 : -1.0) * extents * mapRes;
            map.calculateSegmentKeys(ray, origin, point, false);
            for (auto key : ray)
            {
              map.integrateMiss(key);
            }
            map.integrateHit(point);
          }
        }
      }
    };

    buildWalls(0, 1, 2);
    buildWalls(1, 2, 0);
    buildWalls(0, 2, 1);
  }



  TEST(Serialisation, Upgrade)
  {
    // Test loading older version data files.
    std::string mapName = std::string(dataRelPath()) + "test-map.0.ohm";
    // Profile profile;
    int errorCode = 0;
    const float boundaryDistance = 5.0f;
    OccupancyMap testMap(0.25);
    OccupancyMap loadMap(1);  // Initialise at the wrong resolution. Will be fixed on load.

    // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
    // Need to use the same code with which test-map.0.ohm was generated.
    cubicRoomLegacy(testMap, boundaryDistance, 3);

    std::cout << "Validate header" << std::endl;
    MapVersion version;
    errorCode = loadHeader(mapName.c_str(), loadMap, &version);
    ASSERT_EQ(errorCode, 0);
    ASSERT_EQ(version.major, 0);
    ASSERT_EQ(version.minor, 0);
    ASSERT_EQ(version.patch, 0);

    ProgressDisplay progress;
    std::cout << "Loading" << std::endl;
    errorCode = load(mapName.c_str(), loadMap, &progress);
    std::cout << std::endl;
    ASSERT_EQ(errorCode, 0);

    ohmtestutil::compareMaps(loadMap, testMap, ohmtestutil::CF_Default);
  }
}
