// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef OHM_HEIGHTMAPMODE_H
#define OHM_HEIGHTMAPMODE_H

#include "OhmConfig.h"

namespace ohm
{
/// Enumeration of the available heightmap generation modes.
enum class HeightmapMode
{
  /// Simple planar traversal from the reference position. Heightmap grid cells are only visited once each.
  kPlanar,
  /// Use a simple flood fill out from the reference position. The resulting heightmap may have better continuity than
  /// @c kPlanar. Voxels may be revisited when a lower candidate voxel is found.
  kSimpleFill,
  /// Use a flood fill which supports layering. The fill attempts to expand on all available surfaces and can generate
  /// a multi layer heightmap.
  kLayeredFill
};
}  // namespace ohm

#endif  // OHM_HEIGHTMAPMODE_H
