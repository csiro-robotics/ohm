// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef OHMHEIGHTMAP_HEIGHTMAPMODE_H
#define OHMHEIGHTMAP_HEIGHTMAPMODE_H

#include "OhmHeightmapConfig.h"

#include <string>

namespace ohm
{
/// Enumeration of the available heightmap generation modes.
enum class HeightmapMode
{
  /// Simple planar traversal from the reference position. Heightmap grid cells are only visited once each.
  kPlanar,
  /// Use a simple flood fill out from the reference position. The resulting heightmap may have better continuity than
  /// @c kPlanar. Voxels may be revisited when a lower candidate voxel is found.
  kSimpleFill,
  /// Use a flood fill which supports layering. The fill attempts to expand on all available surfaces and can generate
  /// a multi layer heightmap. The resulting heightmap is 2.5D and each column can contain multiple entries. The height
  /// values of entries in each column are unsorted with undefined height ordering.
  kLayeredFillUnordered,
  /// Same as @c kLayeredFillUnordered except that the resulting heightmap ensures each column of voxels in the
  /// generated heightmap are in ascending height order.
  kLayeredFill,

  /// First mode value
  kFirst = kPlanar,
  /// Last mode value.
  kLast = kLayeredFill
};

/// Convert a @c HeightmapMode to a string.
/// @param mode The mode value to convert.
/// @return The string name for @p mode or `"<unknown>"` if @p mode is out of range.
std::string ohmheightmap_API heightmapModeToString(HeightmapMode mode);
/// Convert a string to a @c HeightampMode - reverse of @c heightmapModeToString().
/// @param str The string to convert.
/// @param valid_string Optional - set to true if @p str is a valid mode name, false otherwise.
/// @return The mode matching @p str or @c HeightmapMode::kPlanar when @p str is not a valid mode name.
HeightmapMode ohmheightmap_API heightmapModeFromString(const std::string &str, bool *valid_string = nullptr);
}  // namespace ohm

#endif  // OHMHEIGHTMAP_HEIGHTMAPMODE_H
