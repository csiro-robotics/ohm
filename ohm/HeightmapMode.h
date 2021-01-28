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
  /// a multi layer heightmap. The resulting heightmap is 2.5D and each column can contain multiple entries. The height
  /// values of entries in each column are unsorted with undefined height ordering.
  kLayeredFill,
  /// Same as @c kLayeredFill except that the resulting heightmap ensures each column of voxels in the generated
  /// heightmap are in ascending height order.
  kLayeredFillOrdered,
  /// Similar to @c kLayeredFillOrdered except that the cell at the bottom of each column may be out of order and
  /// mimics the results of @c kPlanar.
  kLayeredFillOrderedBase
};
}  // namespace ohm

#endif  // OHM_HEIGHTMAPMODE_H
