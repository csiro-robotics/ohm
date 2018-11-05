// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPDETAIL_H
#define HEIGHTMAPDETAIL_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

namespace ohm
{
  class OccupancyMap;

  struct HeightmapDetail
  {
    ohm::OccupancyMap *occupancy_map = nullptr;
    /// Use a very thin occupancy map for the heightmap representation.
    ohm::OccupancyMap *heightmap = nullptr;
    glm::dvec4 heightmap_plane;
    /// Voxel layer containing the @c HeightmapVoxel data.
    unsigned heightmap_layer = 0;
  };

  struct HeightmapVoxel
  {
    float min_offset;
    float clearance_offset;
  };
} // namespace ohm

#endif // HEIGHTMAPDETAIL_H
