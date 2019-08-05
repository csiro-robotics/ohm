// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_CLEARANCEPROCESSEDETAIL_H
#define OHMGPU_CLEARANCEPROCESSEDETAIL_H

#include "OhmGpuConfig.h"

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
    unsigned query_flags = 0;
    glm::vec3 axis_scaling = glm::vec3(1);
    glm::i16vec3 min_dirty_region = glm::i16vec3(1);
    glm::i16vec3 max_dirty_region = glm::i16vec3(0);
    glm::i16vec3 current_dirty_cursor = glm::i16vec3(0);
    /// Last value of the @c OccupancyMap::stamp(). Used to see if the cache needs to be cleared.
    uint64_t map_stamp = 0;
    float search_radius = 0;

    std::unique_ptr<RoiRangeFill> gpu_query;

    inline bool haveWork() const
    {
      return glm::all(glm::lessThanEqual(min_dirty_region, max_dirty_region)) &&
             glm::all(glm::lessThanEqual(current_dirty_cursor, max_dirty_region));
    }

    inline void resetWorking()
    {
      min_dirty_region = glm::i16vec3(1);
      max_dirty_region = current_dirty_cursor = glm::i16vec3(0);
      map_stamp = 0;
    }

    void stepCursor(const glm::i16vec3 &step = glm::i16vec3(1));

    void getWork(OccupancyMap &map);  // NOLINT(google-runtime-references)
  };
}  // namespace ohm

#endif  // OHMGPU_CLEARANCEPROCESSEDETAIL_H
