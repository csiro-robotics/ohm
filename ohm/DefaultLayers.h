// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDEFAULTLAYERS_H
#define OHMDEFAULTLAYERS_H

#include "OhmConfig.h"

namespace ohm
{
  /// Default @c MapLayer identification.
  enum DefaultLayers
  {
    /// Voxel occupancy values.
    kDlOccupancy,
    /// Voxel clearance values: distance to nearest obstruction.
    kDlClearance,
    /// Downsampled @c DL_Clearance
    kDlCoarseClearance,

    /// Number of default layers.
    kDlCount
  };
}

#endif // OHMDEFAULTLAYERS_H
