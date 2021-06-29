// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMHEIGHTMAP_HEIGHTMAPVOXEL_H
#define OHMHEIGHTMAP_HEIGHTMAPVOXEL_H

#include "OhmHeightmapConfig.h"

#include <cstdint>

namespace ohm
{
/// Voxel layer values for @c HeightmapVoxel::type.
enum HeightmapVoxelLayer : uint8_t
{
  kHvlBaseLayer = 0,  ///< Voxel belongs to the base layer or is a base layer candidate
  kHvlExtended,       ///< Voxel is outside the base layer bounds - only used for layered heightmaps.
  kHvlInvalid         ///< Voxel has been determined to be invalid and should be removed.
};

/// A voxel within the heightmap.
///
/// @note Use of @c alignas will need to be removed should this structure be needed in GPU code. OpenCL 1.2 supports
/// `__attribute__ ((aligned (n)))` style alignment. A macro may be used to manage this.
/// See https://www.khronos.org/registry/OpenCL/sdk/1.2/docs/man/xhtml/
///
/// CUDA compatibility would also need to be assessed.
struct ohmheightmap_API alignas(8) HeightmapVoxel
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
  /// Alignment padding to add @c contributing_samples without changing the data structure.
  uint8_t reserved;
  /// The number of samples in the source voxel which contributed to this result - only available if @c VoxelMean
  /// is available to accumulate the sample count. Note that this may be lower than the value in @c VoxelMean
  /// because it is of lower precision. In this case, the value is set to @c 0xffffu.
  uint16_t contributing_samples;
};
}  // namespace ohm

#endif  // OHMHEIGHTMAP_HEIGHTMAPVOXEL_H
