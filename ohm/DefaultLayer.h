// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDEFAULTLAYER_H
#define OHMDEFAULTLAYER_H

#include "OhmConfig.h"

namespace ohm
{
  namespace default_layer
  {
    /// Name of the occupancy layer.
    /// @return "occupancy"
    const char *ohm_API occupancyLayerName();
    /// Name of the @c VoxelMean layer containing mean voxel coordinates.
    /// @return "mean"
    const char *ohm_API meanLayerName();
    /// Name of the voxel clearance layer.
    /// @return "clearance"
    const char *ohm_API clearanceLayerName();
  }  // namespace default_layer

  class MapLayout;
  class MapLayer;

  /// Add the @c VoxelMean layer to @p layout.
  ///
  /// This ensures @p layout has a layer with a name matching @p meanLayerName() setup to hold @c VoxelMean data.
  ///
  /// The function makes no changes if @p layout already has a layer named according to @c meanLayerName() , but no
  /// validation is performed to ensure that the data contained in that layer matches @c VoxelMean .
  ///
  /// @param layout The @p MapLayout to modify.
  /// @return The map layer added or the pre-existing layer named according to @c meanLayerName() .
  /// @see @c voxelmean
  MapLayer * ohm_API addVoxelMean(MapLayout &layout);
}  // namespace ohm

#endif  // OHMDEFAULTLAYER_H
