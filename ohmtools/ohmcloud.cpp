// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmcloud.h"

#include <ohm/occupancymap.h>
#include <ohm/occupancyquery.h>
#include <ohm/occupancytype.h>

#include <ohmutil/plymesh.h>

#include <algorithm>

namespace ohmtools
{
  void saveCloud(const char *fileName, const ohm::OccupancyMap &map, const ProgressCallback &prog)
  {
    PlyMesh ply;
    glm::vec3 v;
    size_t regionCount = map.regionCount();
    size_t processedRegionCount = 0;
    glm::i16vec3 lastRegion = map.begin().key().regionKey();

    for (auto iter = map.begin(); iter != map.end(); ++iter)
    {
      const ohm::OccupancyNodeConst node = *iter;
      if (lastRegion != iter.key().regionKey())
      {
        ++processedRegionCount;
        if (prog)
        {
          prog(processedRegionCount, regionCount);
        }
        lastRegion = iter.key().regionKey();
      }
      if (map.occupancyType(node) == ohm::Occupied)
      {
        v = map.voxelCentreLocal(node.key());
        ply.addVertex(v);
      }
    }

    ply.save(fileName, true);
  }


  void saveQueryCloud(const char *fileName, const ohm::OccupancyMap &map, const ohm::Query &query,
                      float colourRange, const ProgressCallback &prog)
  {
    size_t resultCount = query.numberOfResults();
    const ohm::OccupancyKey *keys = query.intersectedVoxels();
    const float *ranges = query.ranges();
    glm::dvec3 voxelPos;

    PlyMesh ply;
    for (size_t i = 0; i < resultCount; ++i)
    {
      const ohm::OccupancyKey &key = keys[i];
      uint8_t c = 255;
      if (colourRange > 0 && ranges)
      {
        float rangeValue = ranges[i];
        if (rangeValue < 0)
        {
          rangeValue = colourRange;
        }
        c = (uint8_t)(255 * std::max(0.0f, (colourRange - rangeValue) / colourRange));
      }
      voxelPos = map.voxelCentreGlobal(key);
      ply.addVertex(voxelPos, Colour(c, 128, 0));
      if (prog)
      {
        prog(i + 1, resultCount);
      }
    }

    ply.save(fileName, true);
  }


  size_t saveClearanceCloud(const char *fileName, const ohm::OccupancyMap &map, const glm::dvec3 &minExtents,
                            const glm::dvec3 &maxExtents, float colourRange, int exportType,
                            const ProgressCallback &prog)
  {
    size_t regionCount = map.regionCount();
    size_t processedRegionCount = 0;
    glm::i16vec3 minRegion, maxRegion;
    glm::dvec3 v;
    glm::i16vec3 lastRegion = map.begin().key().regionKey();
    PlyMesh ply;
    size_t pointCount = 0;

    minRegion = map.regionKey(minExtents);
    maxRegion = map.regionKey(maxExtents);

    const float colourScale = colourRange;
    auto mapEndIter = map.end();
    for (auto iter = map.begin(); iter != mapEndIter; ++iter)
    {
      const ohm::OccupancyNodeConst node = *iter;
      if (lastRegion != iter.key().regionKey())
      {
        ++processedRegionCount;
        if (prog)
        {
          prog(processedRegionCount, regionCount);
        }
        lastRegion = iter.key().regionKey();
      }

      // Ensure the node is in a region we have calculated data for.
      if (minRegion.x <= lastRegion.x && lastRegion.x <= maxRegion.x &&
          minRegion.y <= lastRegion.y && lastRegion.y <= maxRegion.y &&
          minRegion.z <= lastRegion.z && lastRegion.z <= maxRegion.z)
      {
        //if (node.isOccupied() || node.isFree())
        bool exportMatch = !node.isNull() && map.occupancyType(node) >= exportType;
        if (exportMatch)
        {
          float rangeValue = node.clearance();
          if (rangeValue < 0)
          {
            rangeValue = colourRange;
          }
          if (rangeValue >= 0)
          {
            uint8_t c = (uint8_t)(255 * std::max(0.0f, (colourScale - rangeValue) / colourScale));
            v = map.voxelCentreLocal(node.key());
            ply.addVertex(v, Colour(c, 128, 0));
            ++pointCount;
          }
        }
      }
    }

    ply.save(fileName, true);

    return pointCount;
  }
}
