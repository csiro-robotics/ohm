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
  enum DefaultLayer
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

  /// Resolve the name for one of the @c DefaultLayer identifiers.
  /// @param layer The layer identifier of interest.
  /// @return The name of the layer, or nullptr if @p layer is out of range.
  const char * ohm_API defaultLayerName(DefaultLayer layer);
}

#endif // OHMDEFAULTLAYERS_H
