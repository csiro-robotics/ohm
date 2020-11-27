// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_HEIGHTMAPUTIL_H
#define OHM_HEIGHTMAPUTIL_H

#include "OhmConfig.h"

#include "Key.h"
#include "UpAxis.h"

namespace ohm
{
struct HeightmapDetail;
class OccupancyMap;
class MapInfo;

/// This namespace contains utility functions used to help build and maintain heightmap instances.
///
/// These functions are intended to only be used as part of the internal ohm API.
namespace heightmap
{
/// Initialise the @p heightmap voxel layout.
int ohm_API setupHeightmap(ohm::OccupancyMap &heightmap, HeightmapDetail &detail);

/// Query the heightmap axis configured in the given @c MapInfo.
/// @param info The map info data to query.
/// @return The configured heigthmap axis or @c UpAxis::kZ if nothing is configured.
UpAxis ohm_API queryHeightmapAxis(const MapInfo &info);
}  // namespace heightmap
}  // namespace ohm

#endif  // OHM_HEIGHTMAPUTIL_H
