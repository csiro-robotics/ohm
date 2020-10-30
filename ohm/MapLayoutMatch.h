//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef MAPLAYOUTMATCH_H
#define MAPLAYOUTMATCH_H

#include "OhmConfig.h"

namespace ohm
{
/// Return values for @c MapLayout, @p MapLayer and @p VoxelLayout checks.
enum class MapLayoutMatch : int
{
  /// Layers are different and do not match.
  Different = 0,
  /// The layout matches, but some of the layer and voxel member names may differ.
  Equivalent,
  /// The layer match exactly.
  Exact
};
}  // namespace ohm

#endif  // MAPLAYOUTMATCH_H
