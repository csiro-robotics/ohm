// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef UPAXIS_H
#define UPAXIS_H

#include "OhmConfig.h"

namespace ohm
{
  /// Up axis identification values. Used for @c Heightmap generation.
  ///
  /// Documentation for each ID identifies the up axis.
  enum class UpAxis : int
  {
    /// (0, 0, -1)
    kNegZ = -3,
    /// (0, -1, 0)
    kNegY = -2,
    /// (-1, 0, 0)
    kNegX = -1,
    /// (1, 0, 0)
    kX,
    /// (0, 1, 0)
    kY,
    /// (0, 0, 1)
    kZ,
  };
}

#endif // UPAXIS_H
