// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPVOXEL_H
#define HEIGHTMAPVOXEL_H

#include "OhmConfig.h"

namespace ohm
{
  /// A voxel within the heightmap.
  struct HeightmapVoxel
  {
    /// The name of the layer which stores these voxels.
    static const char *heightmap_layer;
    /// The name of the layer used to build the first pass heightmap. This is the layer without blur.
    /// Only used when using blur.
    static const char *heightmap_build_layer;

    float height;
    float clearance;
  };
}  // namespace ohm

#endif  // HEIGHTMAPVOXEL_H
