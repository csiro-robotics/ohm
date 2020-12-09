// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMTOOLS_OHMCLOUD_H
#define OHMTOOLS_OHMCLOUD_H

#include "OhmToolsConfig.h"

#include <ohm/KeyRange.h>

#include <ohmutil/Colour.h>

#include <glm/glm.hpp>

#include <array>
#include <functional>

namespace ohm
{
class Key;
class OccupancyMap;
class Query;
template <typename T>
class Voxel;
}  // namespace ohm

namespace ohmtools
{
/// Callback used by @c saveCloud() etc to report on progress.
/// The arguments passed are the current progress and the target progress respectively.
using ProgressCallback = std::function<void(size_t, size_t)>;

/// Colour selection filter function. May be used to override colour assignment.
using ColourSelect = std::function<ohm::Colour(const ohm::Voxel<const float> &)>;

/// Options used to adjust how a cloud is saved from an occupancy map.
struct SaveCloudOptions
{
  /// Overrides the default colour selection.
  ColourSelect colour_select{};
  /// When true and @c colour_select is empty, uses the default colour selection strategy which best suites the
  /// cloud type. For example, a general cloud may @c ColourByHeight while a heightmap is better suited to
  /// @c ColourByHeightmapClearance .
  bool allow_default_colour_selection = true;
  /// Export free space voxels? Required to get virtual surfaces from heightmaps.
  bool export_free = false;
  /// Ingore voxel mean forcing voxel centres for positions?
  bool ignore_voxel_mean = false;
};

//------------------------------------------------------------------------------
// Colour selection helpers
//------------------------------------------------------------------------------
/// A helper for @c saveCloud() to colour a cloud by height.
class ColourByHeight
{
public:
  static const ohm::Colour s_default_from;
  static const ohm::Colour s_default_to;

  std::array<ohm::Colour, 2> colours;

  explicit ColourByHeight(const ohm::OccupancyMap &map);
  ColourByHeight(const ohm::OccupancyMap &map, const ohm::Colour &from, const ohm::Colour &to);
  explicit ColourByHeight(const ohm::KeyRange &extents);
  ColourByHeight(const ohm::KeyRange &extents, const ohm::Colour &from, const ohm::Colour &to);

  ohm::Colour select(const ohm::Key &key) const;
  ohm::Colour select(const ohm::Voxel<const float> &occupancy) const;

private:
  ohm::KeyRange range_;
  int up_axis_ = 2;
};

/// Colour heightmap voxels by @c HeightmapVoxelType::kSurface or @c HeightmapVoxelType::kVirtualSurface type.
class ColourHeightmapType
{
public:
  ohm::Colour surface_colour{ 32, 255, 32 };
  ohm::Colour virtual_colour{ 255, 128, 32 };

  explicit ColourHeightmapType(const ohm::OccupancyMap &map);

  ohm::Colour select(const ohm::Voxel<const float> &occupancy) const;

private:
  int heightmap_layer_ = -1;
};

/// A helper for @c saveCloud() to colour a cloud by height.
class ColourByHeightmapClearance
{
public:
  static const ohm::Colour s_default_low;
  static const ohm::Colour s_default_high;

  std::array<ohm::Colour, 2> colours;

  explicit ColourByHeightmapClearance(const ohm::OccupancyMap &map, double clearance_scale = 2.0);
  ColourByHeightmapClearance(const ohm::OccupancyMap &map, const ohm::Colour &low, const ohm::Colour &high,
                             double clearance_scale = 2.0);

  ohm::Colour select(const ohm::Voxel<const float> &occupancy) const;

private:
  double min_clearance_ = 0;
  double max_clearance_ = 0;
  int heightmap_layer_ = -1;
};

//------------------------------------------------------------------------------
// Cloud ply saving functions.
//------------------------------------------------------------------------------

/// Save @p map to a ply file, exporting only Occupied voxels.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save.
/// @param opt Additional export controls.
/// @param prog Optional function called to report on progress.
/// @return The number of points saved.
uint64_t ohmtools_API saveCloud(const char *file_name, const ohm::OccupancyMap &map,
                                const SaveCloudOptions &opt = SaveCloudOptions(),
                                const ProgressCallback &prog = ProgressCallback());

/// Save @p map assuming it is a heightmap (contains @c HeightmapVoxel data) to a ply cloud.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save.
/// @param opt Additional export controls.
/// @param prog Optional function called to report on progress.
/// @return The number of points saved.
uint64_t ohmtools_API saveHeightmapCloud(const char *file_name, const ohm::OccupancyMap &heightmap,
                                         const SaveCloudOptions &opt = SaveCloudOptions(),
                                         const ProgressCallback &prog = ProgressCallback());

/// Save the results of @p query on @p map to a ply file, exporting all intersected voxels.
/// Voxels are coloured by the reported query range values where voxels at or beyond @p colourRange
/// are green, tending to orange closer to zero.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save voxels from.
/// @param query The query to save results from.
/// @param colour_range Affects voxel colouring as described above. Green at this range.
/// @param prog Optional function called to report on progress.
void ohmtools_API saveQueryCloud(const char *file_name, const ohm::OccupancyMap &map, const ohm::Query &query,
                                 float colour_range = 0.0f, const ProgressCallback &prog = ProgressCallback());

/// Save a point cloud representing the @c Voxel::clearance() values for voxels in @p map.
///
/// Voxels are coloured by the reported query range values where voxels at or beyond @p colourRange
/// are green, tending to orange closer to zero.
///
/// Only saves data from regions overlapping the @p minExtents and @p maxExtents, though all voxels from
/// such regions are exported, not just those within the extents.
///
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save voxels from.
/// @param min_extents Min extents to save overlapping regions from.
/// @param max_extents Max extents to save overlapping regions from.
/// @param colour_range Affects voxel colouring as described above. Green at this range.
/// @param export_type Type of voxels to export. Voxels of this @c OccupancyType or greater are exported.
/// @param prog Optional function called to report on progress.
size_t ohmtools_API saveClearanceCloud(const char *file_name, const ohm::OccupancyMap &map,
                                       const glm::dvec3 &min_extents, const glm::dvec3 &max_extents,
                                       float colour_range = 0.0f, int export_type = 0,
                                       const ProgressCallback &prog = ProgressCallback());
}  // namespace ohmtools

#endif  // OHMTOOLS_OHMCLOUD_H
