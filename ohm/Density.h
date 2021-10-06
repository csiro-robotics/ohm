// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_DENSITY_H
#define OHM_DENSITY_H

#include "OhmConfig.h"

#include "Voxel.h"
#include "VoxelMean.h"

namespace ohm
{
class OccupancyMap;
class Key;

/// Query the density for the voxel at @p key . The density is calculated as the number of hits (samples) in a
/// voxel divided by the traversal (see @c default_layer::addTraversalLayer() ). It is left to the user to interpret or
/// threshold this value.
///
/// This function requires the @p map have a traversal layer and a voxel mean layer. Otherwise the result will be zero.
/// The result is also zero if the @p key is invalid.
///
/// @todo Should the traversal value be scaled by the voxel resolution? That would make the @c density() value
/// independent of voxel size.
///
/// @param map The map object to query.
/// @param key The key identifying the voxel of interest.
/// @return The density or zero on failure.
float ohm_API voxelDensity(const OccupancyMap &map, const Key &key);

/// Calculate the density given a traversal layer voxel and the corresponding @c VoxelMean . This function assumes
/// that both are referencing the same voxel key.
///
/// The density is calculated as the @c `VoxelMean::count / traversal`. The result is zero if the mean count is zero
/// or either voxel is invalid. The result is infinite if the count is non zero, but the traversal is zero.
///
/// @param traversal_voxel Details of the voxel traversal accumulation. See @c default_layer::addTraversalLayer()
/// @param mean_voxel The voxel mean and hit count.
/// @return The density or zero on failure.
inline float ohm_API voxelDensity(const Voxel<const float> &traversal_voxel, const Voxel<const VoxelMean> &mean_voxel)
{
  if (traversal_voxel.isValid() && mean_voxel.isValid())
  {
    const float count = float(mean_voxel.data().count);
    const float traversal = traversal_voxel.data();
    if (count > 0)
    {
      return (traversal > 0) ? count / traversal : std::numeric_limits<float>::infinity();
    }
  }
  return 0.0f;
}
}  // namespace ohm

#endif  // OHM_DENSITY_H