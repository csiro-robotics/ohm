// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPVOXEL_H
#define HEIGHTMAPVOXEL_H

#include "OhmConfig.h"

namespace ohm
{
enum class HmVoxelFlag : unsigned
{
  kAmbiguous = (1u << 0u)
};

/// A voxel within the heightmap.
struct HeightmapVoxel
{
  /// The name of the layer which stores these voxels.
  static const char *const kHeightmapLayer;
  /// The name of the layer used to build the first pass heightmap. This is the layer without blur.
  /// Only used when using blur.
  static const char *const kHeightmapBuildLayer;

  /// The voxel height, relative to the voxel centre.
  float height;
  /// Clearance above the voxel height to the next, occupied voxel. Zero indicates no known overhead obstruction.
  float clearance;
  /// Local surface normal X coordinate.
  ///
  /// The three normal coordinates are only valid when the heightmap has been generated from an occupancy map
  /// which includes @c CovarianceVoxel . The normal is an estimate from the covariance.
  float normal_x;
  /// Local surface normal Y coordinate.
  float normal_y;
  /// Local surface normal Z coordinate.
  float normal_z;
  /// Informational flags : @c HmVoxelFlag.
  uint8_t flags;
  /// Reserved for padding
  int8_t reserved[3];
};
}  // namespace ohm

#endif  // HEIGHTMAPVOXEL_H
