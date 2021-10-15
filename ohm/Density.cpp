// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Density.h"

#include "Key.h"
#include "MapLayout.h"
#include "OccupancyMap.h"

#include <limits>

namespace ohm
{
float voxelDensity(const OccupancyMap &map, const Key &key)
{
  if (!key.isNull())
  {
    Voxel<const float> traversal_voxel(&map, map.layout().traversalLayer());
    Voxel<const VoxelMean> mean_voxel(&map, map.layout().meanLayer());
    if (traversal_voxel.isLayerValid() && mean_voxel.isLayerValid())
    {
      setVoxelKey(key, traversal_voxel, mean_voxel);
      return voxelDensity(traversal_voxel, mean_voxel);
    }
  }
  return 0.0f;
}
}  // namespace ohm
