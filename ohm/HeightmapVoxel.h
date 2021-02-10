// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPVOXEL_H
#define HEIGHTMAPVOXEL_H

#include "OhmConfig.h"

#include <cstdint>

namespace ohm
{
/// Voxel layer values for @c HeightmapVoxel::type.
enum HeightmapVoxelLayer : uint8_t
{
  kHvlBaseLayer = 0,  ///< Voxel belongs to the base layer or is a base layer candidate
  kHvlExtended        ///< Voxel is outside the base layer bounds - only used for layered heightmaps.
};

/// A voxel within the heightmap.
///
/// @note Use of @c alignas will need to be removed should this structure be needed in GPU code. OpenCL 1.2 supports
/// `__attribute__ ((aligned (n)))` style alignment. A macro may be used to manage this.
/// See https://www.khronos.org/registry/OpenCL/sdk/1.2/docs/man/xhtml/
///
/// CUDA compatibilty would also need to be assessed.
struct alignas(8) HeightmapVoxel
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
  /// Layer information. Currently restricted to values of @c HeightmapVoxelLayer.
  uint8_t layer;
};
}  // namespace ohm

#endif  // HEIGHTMAPVOXEL_H
