// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAP_VOXEL_TYPE_H_
#define HEIGHTMAP_VOXEL_TYPE_H_

#include "OhmConfig.h"

namespace ohm
{
  /// Type enumeration for voxels in a heightmap.
  enum class HeightmapVoxelType : uint8_t
  {
    /// Unkown or unobserved voxel.
    Unknown = 0,
    /// A voxel which is being kept forcibly vacant. Normally this represents voxels which have yet to be able to
    /// observed at the startup location.
    Vacant,
    /// Represents a real surface voxel in the heightmap.
    Surface,
    /// Represents a virtual surface voxel in the heightmap. Virtual surfaces are created at the interface between
    /// free and unknown voxels (free supported by unknown).
    VirtualSurface,
    /// A voxel which has been inferred as fatal. This occurs where virtual surfaces appear near the reference point.
    InferredFatal,
    /// An otherwise forcibly fatal cost voxel.
    Fatal
  };
}

#endif // HEIGHTMAP_VOXEL_TYPE_H_
