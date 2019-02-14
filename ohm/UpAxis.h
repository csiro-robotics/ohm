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
    NegZ = -3,
    /// (0, -1, 0)
    NegY = -2,
    /// (-1, 0, 0)
    NegX = -1,
    /// (1, 0, 0)
    X,
    /// (0, 1, 0)
    Y,
    /// (0, 0, 1)
    Z,
  };
}

#endif // UPAXIS_H
