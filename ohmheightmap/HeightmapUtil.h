// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMHEIGHTMAP_HEIGHTMAPUTIL_H
#define OHMHEIGHTMAP_HEIGHTMAPUTIL_H

#include "OhmHeightmapConfig.h"

#include "UpAxis.h"

#include <ohm/Key.h>

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
int ohmheightmap_API setupHeightmap(ohm::OccupancyMap &heightmap, HeightmapDetail &detail);

/// Query the heightmap axis configured in the given @c MapInfo.
/// @param info The map info data to query.
/// @return The configured heigthmap axis or @c UpAxis::kZ if nothing is configured.
UpAxis ohmheightmap_API queryHeightmapAxis(const MapInfo &info);

/// Query the configured heightmap clearance value.
/// @param info The map info data to query.
/// @return The configured clearance used to generate the heightmap. Zero if not present.
double ohmheightmap_API queryHeightmapClearance(const MapInfo &info);

/// Resolve an array of indices which may be used to reference XYZ axes from a given heightmap up axis.
///
/// Each element in the returned array indexes one of the {X, Y, Z} axes as {0, 1, 2} respectively. The resulting
/// array is constructed such that the element at index 2 always matches the axis specified by @p up, while the other
/// two elements - at indices {0, 1} - define the axes of the heightmap plane.
///
/// Given the statement `auto array = heightmapAxisIndices(up_axis)`, the full set of possible results is tabulated
/// below;
///
/// `up_axis`       | `array[0]`  | `array[1]`  | `array[2]`  |
/// --------------- | ----------: | ----------: | ----------: |
/// `UpAxis::kX     | 1           | 2           | 0           |
/// `UpAxis::kNegX  | 1           | 2           | 0           |
/// `UpAxis::kY     | 0           | 2           | 1           |
/// `UpAxis::kNegY  | 0           | 2           | 1           |
/// `UpAxis::kZ     | 0           | 1           | 2           |
/// `UpAxis::kNegX  | 0           | 1           | 2           |
///
/// @param up_axis The up axis to generate a set of indices for.
/// @return A set of indices which can be used to reference axes as described above.
std::array<int, 3> ohmheightmap_API heightmapAxisIndices(UpAxis up_axis);
}  // namespace heightmap
}  // namespace ohm

#endif  // OHMHEIGHTMAP_HEIGHTMAPUTIL_H
