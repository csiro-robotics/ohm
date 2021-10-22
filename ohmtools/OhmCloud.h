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
#include <string>

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
struct ohmtools_API SaveCloudOptions
{
  /// Overrides the default colour selection.
  ColourSelect colour_select{};
  /// When true and @c colour_select is empty, uses the default colour selection strategy which best suites the
  /// cloud type. For example, a general cloud may @c ColourByHeight while a heightmap is better suited to
  /// @c ColourByHeightmapClearance .
  bool allow_default_colour_selection = true;
  /// Export free space voxels? Required to get virtual surfaces from heightmaps.
  bool export_free = false;
  /// Ignore voxel mean forcing voxel centres for positions?
  bool ignore_voxel_mean = false;
};

/// Options for saving a density cloud.
struct SaveDensityCloudOptions : SaveCloudOptions
{
  /// Density rate threshold to pass.
  float density_threshold = 0;
};

/// Specialised options for saving heightmap clouds. Supports construction from a @c SaveCloudOptions setting default
/// values for heightmap extended parameters.
struct ohmtools_API SaveHeightmapCloudOptions : SaveCloudOptions
{
  bool collapse = false;  ///< Collapse to a 2.5D heightmap, reducing layered heightmaps to 2D?

  /// Instantiate default options.
  inline SaveHeightmapCloudOptions() = default;
  /// Copy constructor
  /// @param other Object to copy.
  inline SaveHeightmapCloudOptions(const SaveHeightmapCloudOptions &other) = default;
  /// Copy constructor copying the base @c SaveCloudOptions.
  /// @param other Object to copy.
  inline SaveHeightmapCloudOptions(const SaveCloudOptions &other)  // NOLINT(google-explicit-constructor)
    : SaveCloudOptions(other)
  {}

  /// Assignment operator.
  /// @param other Object to copy.
  inline SaveHeightmapCloudOptions &operator=(const SaveHeightmapCloudOptions &other) = default;
  /// Assignment operator copying the base @c SaveCloudOptions.
  /// @param other Object to copy.
  inline SaveHeightmapCloudOptions &operator=(const SaveCloudOptions &other)
  {
    SaveCloudOptions::operator=(other);
    return *this;
  }
};

//------------------------------------------------------------------------------
// Colour selection helpers
//------------------------------------------------------------------------------
/// A helper for @c saveCloud() to colour a cloud by height.
class ohmtools_API ColourByHeight
{
public:
  /// Default colour for the mimimum height value
  static const ohm::Colour kDefaultFrom;
  /// Default colour for the maximum height value
  static const ohm::Colour kDefaultTo;

  /// Colours to interpolate from `[0]` and to `[1]` across the height range.
  std::array<ohm::Colour, 2> colours;

  /// Create a height based colouriser for the given @p map.
  /// @param map The map to colour by - extracts extents.
  explicit ColourByHeight(const ohm::OccupancyMap &map);
  /// Create a height based colouriser for the given @p map using custom colours.
  /// @param map The map to colour by - extracts extents.
  /// @param from The lowest height colour.
  /// @param to The highest height colour.
  ColourByHeight(const ohm::OccupancyMap &map, const ohm::Colour &from, const ohm::Colour &to);
  /// Create a height based colouriser across the given @p extents.
  /// @param extents Key range overwhich voxels are to be coloured.
  explicit ColourByHeight(const ohm::KeyRange &extents);
  /// Create a height based colouriser across the given @p extents using custom colours.
  /// @param extents Key range overwhich voxels are to be coloured.
  /// @param from The lowest height colour.
  /// @param to The highest height colour.
  ColourByHeight(const ohm::KeyRange &extents, const ohm::Colour &from, const ohm::Colour &to);

  /// Select a colour for the given voxel @p key.
  /// @param key The voxel to colour.
  /// @return The colour for the voxel at @p key.
  ohm::Colour select(const ohm::Key &key) const;
  /// Select a colour for the given voxel @p occupancy.
  /// @param occupancy The voxel to colour.
  /// @return The colour for the voxel at @p occupancy.
  ohm::Colour select(const ohm::Voxel<const float> &occupancy) const;

private:
  ohm::KeyRange range_;
  int up_axis_ = 2;
};

/// Colour heightmap voxels by @c ohm::OccupancyType colouring occupied and free voxels.
class ohmtools_API ColourByType
{
public:
  /// Colour for occupied voxels
  ohm::Colour occupied_colour{ 32, 255, 32 };
  /// Colour for free occupied voxels (if enabled)
  ohm::Colour free_colour{ 255, 128, 32 };

  /// Create a voxel type colouriser for @p map.
  /// @param map The map to be colourised.
  explicit ColourByType(const ohm::OccupancyMap &map);

  /// Select a colour for the given voxel @p occupancy.
  /// @param occupancy The voxel to colour.
  /// @return The colour for the voxel at @p occupancy.
  ohm::Colour select(const ohm::Voxel<const float> &occupancy) const;

private:
  float occupancy_threshold_ = 0;
};

/// Colour heightmap voxels such that voxels in matching heightmap layers have the same colour.
class ohmtools_API ColourHeightmapLayer
{
public:
  /// Create a layer colouriser for @p map.
  /// @param map The map to be colourised.
  explicit ColourHeightmapLayer(const ohm::OccupancyMap &map);

  /// Select a colour for the given voxel @p occupancy.
  /// @param occupancy The voxel to colour.
  /// @return The colour for the voxel at @p occupancy.
  ohm::Colour select(const ohm::Voxel<const float> &occupancy) const;

private:
  int heightmap_up_axis_ = 2;
};

/// Colour heightmap voxels by @c HeightmapVoxelType::kSurface or @c HeightmapVoxelType::kVirtualSurface type.
class ohmtools_API ColourHeightmapType
{
public:
  /// Colour to use for surface voxels.
  ohm::Colour surface_colour{ 32, 255, 32 };
  /// Colour to use for virtual surface voxels.
  ohm::Colour virtual_colour{ 255, 128, 32 };

  /// Create a heightmap voxel type colouriser.
  /// @param map The map to colour by - extracts extents.
  explicit ColourHeightmapType(const ohm::OccupancyMap &map);

  /// Select a colour for the given voxel @p occupancy.
  /// @param occupancy The voxel to colour.
  /// @return The colour for the voxel at @p occupancy.
  ohm::Colour select(const ohm::Voxel<const float> &occupancy) const;

private:
  int heightmap_layer_ = -1;
};

/// A helper for @c saveCloud() to colour a cloud by height.
class ohmtools_API ColourByHeightmapClearance
{
public:
  /// Colour used when there is zero clearance.
  static const ohm::Colour kDefaultLow;
  /// Colour used when the maximum clearance height is available.
  static const ohm::Colour kDefaultHigh;

  /// Colours to interpolate from `[0]` and to `[1]` across the clearance range.
  std::array<ohm::Colour, 2> colours;

  /// Create a clearance based colouriser for the given @p map.
  /// @param map The map to colour by - extracts extents.
  /// @param clearance_scale The maximum clearance height requested.
  explicit ColourByHeightmapClearance(const ohm::OccupancyMap &map, double clearance_scale = 2.0);
  /// Create a clearance based colouriser for the given @p map with custom colours.
  /// @param map The map to colour by - extracts extents.
  /// @param low The colour to use at zero clearance.
  /// @param high The colour to use at @p clearance_scale available clearance.
  /// @param clearance_scale The maximum clearance height requested.
  ColourByHeightmapClearance(const ohm::OccupancyMap &map, const ohm::Colour &low, const ohm::Colour &high,
                             double clearance_scale = 2.0);

  /// Select a colour for the given voxel @p occupancy.
  /// @param occupancy The voxel to colour.
  /// @return The colour for the voxel at @p occupancy.
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
uint64_t ohmtools_API saveCloud(const std::string &file_name, const ohm::OccupancyMap &map,
                                const SaveCloudOptions &opt = SaveCloudOptions(),
                                const ProgressCallback &prog = ProgressCallback());

/// Save @p map to a ply file using the density rate model.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save.
/// @param opt Additional export controls.
/// @param prog Optional function called to report on progress.
/// @return The number of points saved.
uint64_t ohmtools_API saveDensityCloud(const std::string &file_name, const ohm::OccupancyMap &map,
                                       const SaveDensityCloudOptions &opt = SaveDensityCloudOptions(),
                                       const ProgressCallback &prog = ProgressCallback());

/// Similar to @c saveCloud() exporting voxels as a series of cube meshes.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save.
/// @param opt Additional export controls.
/// @param prog Optional function called to report on progress.
/// @return The number of voxels saved.
uint64_t ohmtools_API saveVoxels(const std::string &file_name, const ohm::OccupancyMap &map,
                                 const SaveCloudOptions &opt = SaveCloudOptions(),
                                 const ProgressCallback &prog = ProgressCallback());

/// Save @p map assuming it is a heightmap (contains @c HeightmapVoxel data) to a ply cloud.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save.
/// @param opt Additional export controls.
/// @param prog Optional function called to report on progress.
/// @return The number of points saved.
uint64_t ohmtools_API saveHeightmapCloud(const std::string &file_name, const ohm::OccupancyMap &heightmap,
                                         const SaveHeightmapCloudOptions &opt = SaveHeightmapCloudOptions(),
                                         const ProgressCallback &prog = ProgressCallback());

/// Similar to @c saveHeightmapCloud() exporting voxels as a series of cube meshes.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save.
/// @param opt Additional export controls.
/// @param prog Optional function called to report on progress.
/// @return The number of voxels saved.
uint64_t ohmtools_API saveHeightmapVoxels(const std::string &file_name, const ohm::OccupancyMap &heightmap,
                                          const SaveHeightmapCloudOptions &opt = SaveHeightmapCloudOptions(),
                                          const ProgressCallback &prog = ProgressCallback());

/// Save the results of @p query on @p map to a ply file, exporting all intersected voxels.
/// Voxels are coloured by the reported query range values where voxels at or beyond @p colourRange
/// are green, tending to orange closer to zero.
/// @param file_name File to save to. Please add the .ply extension.
/// @param map The map to save voxels from.
/// @param query The query to save results from.
/// @param colour_range Affects voxel colouring as described above. Green at this range.
/// @param prog Optional function called to report on progress.
void ohmtools_API saveQueryCloud(const std::string &file_name, const ohm::OccupancyMap &map, const ohm::Query &query,
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
size_t ohmtools_API saveClearanceCloud(const std::string &file_name, const ohm::OccupancyMap &map,
                                       const glm::dvec3 &min_extents, const glm::dvec3 &max_extents,
                                       float colour_range = 0.0f, int export_type = 0,
                                       const ProgressCallback &prog = ProgressCallback());
}  // namespace ohmtools

#endif  // OHMTOOLS_OHMCLOUD_H
