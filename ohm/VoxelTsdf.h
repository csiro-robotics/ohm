// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELTSDF_H
#define VOXELTSDF_H

#include "OhmConfig.h"

#include "Voxel.h"

#include <glm/vec3.hpp>

/// @defgroup voxeltsdf Voxel Truncated Signed Distance Fields
/// These functions are used to manipulate the voxel TSDF values. @c VoxelTsdf allows a voxel to store
/// a weight and distance value for the TSDF algorithm.
///
/// The TSDF algorithm used here is based on Voxblox TSDF and is licensed under BSD 3-clause license.
///

namespace ohm
{
class MapInfo;

struct TsdfOptions
{
  /// Maximum TSDF voxel weight.
  float max_weight = 1e4f;
  /// Default truncation distance.
  float default_truncation_distance = 0.1f;
  /// Dropoff for the dropoff adjustment. Zero or negative to disable.
  float dropoff_epsilon = 0.0f;
  /// Compensation for sparse data sets. Zero or negative to disable.
  float sparsity_compensation_factor = 1.0;
};

/// Write the given tsdf options to @c MapInfo .
void ohm_API updateMapInfo(MapInfo &info, const TsdfOptions &tsdf_options);

/// Read the given tsdf options from @c MapInfo .
void ohm_API fromMapInfo(TsdfOptions &tsdf_options, const MapInfo &info);

#include "VoxelTsdfCompute.h"
}  // namespace ohm

#endif  // VOXELTSDF_H
