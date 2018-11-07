// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPDETAIL_H
#define HEIGHTMAPDETAIL_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

#include <memory>

namespace ohm
{
  class OccupancyMap;

  struct HeightmapDetail
  {
    ohm::OccupancyMap *occupancy_map = nullptr;
    /// Use a very thin occupancy map for the heightmap representation.
    std::unique_ptr<ohm::OccupancyMap> heightmap;
    int up_axis_id = 0;
    glm::dvec3 up;
    double min_clearance = 1.0;
    /// Voxel layer containing the @c HeightmapVoxel data.
    unsigned heightmap_layer = 0;
  };
} // namespace ohm

#endif // HEIGHTMAPDETAIL_H
