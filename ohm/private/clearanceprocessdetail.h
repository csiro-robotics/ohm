// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CLEARANCEPROCESSEDETAIL_H_
#define CLEARANCEPROCESSEDETAIL_H_

#include "ohmconfig.h"

#include <glm/glm.hpp>

#include "RoiRangeFill.h"

#include <memory>

// #define CACHE_LOCAL_RESULTS
// #define VALIDATE_KEYS

namespace ohm
{
  class OccupancyMap;
  struct MapChunk;

  struct ClearanceProcessDetail
  {
    unsigned queryFlags = 0;
    glm::vec3 axisScaling = glm::vec3(1);
    glm::i16vec3 minDirtyRegion = glm::i16vec3(1);
    glm::i16vec3 maxDirtyRegion = glm::i16vec3(0);
    glm::i16vec3 currentDirtyCursor = glm::i16vec3(0);
    /// Last value of the @c OccupancyMap::stamp(). Used to see if the cache needs to be cleared.
    uint64_t mapStamp = 0;
    float searchRadius = 0;

    std::unique_ptr<RoiRangeFill> gpuQuery;

    inline bool haveWork() const
    {
      return glm::all(glm::lessThanEqual(minDirtyRegion, maxDirtyRegion)) &&
             glm::all(glm::lessThanEqual(currentDirtyCursor, maxDirtyRegion));
    }

    inline void resetWorking()
    {
      minDirtyRegion = glm::i16vec3(1);
      maxDirtyRegion = currentDirtyCursor = glm::i16vec3(0);
      mapStamp = 0;
    }

    void stepCursor(const glm::i16vec3 &step = glm::i16vec3(1));

    void getWork(OccupancyMap &map);
  };
}

#endif // CLEARANCEPROCESSEDETAIL_H_
