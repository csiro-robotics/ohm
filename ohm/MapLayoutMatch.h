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
  kDifferent = 0,
  /// The layout matches, but some of the layer and voxel member names may differ.
  kEquivalent,
  /// The layer match exactly.
  kExact
};
}  // namespace ohm

#endif  // MAPLAYOUTMATCH_H
