//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPERBASE_H
#define RAYMAPPERBASE_H

#include "OhmConfig.h"

#include "RayFlag.h"

#include <glm/fwd.hpp>

namespace ohm
{
  class OccupancyMap;
  class KeyList;

  class RayMapperBase
  {
  public:
    RayMapperBase();
    virtual ~RayMapperBase();

    virtual size_t integrateRays(const glm::dvec3 *rays, size_t element_count,
                                 unsigned ray_update_flags = kRfDefault) = 0;

    static size_t calculateSegmentKeys(const OccupancyMap &map, KeyList &keys, const glm::dvec3 &start_point,
                                       const glm::dvec3 &end_point, bool include_end_point);
  };
}  // namespace ohm


#endif  // RAYMAPPERBASE_H
