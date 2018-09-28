// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMCLOUD_H_
#define OHMCLOUD_H_

#include "ohmtoolsconfig.h"

#include <glm/glm.hpp>

#include <functional>

namespace ohm
{
  class OccupancyMap;
  class Query;
}

namespace ohmtools
{
  /// Callback used by @c saveCloud() etc to report on progress.
  /// The arguments passed are the current progress and the target progress respectively.
  typedef std::function<void (size_t, size_t)> ProgressCallback;

  /// Save @p map to a ply file, exporting only Occupied voxels.
  /// @param fileName File to save to. Please add the .ply extension.
  /// @param map The map to save.
  /// @param prog Optional function called to report on progress.
  void ohmtools_API saveCloud(const char *fileName, const ohm::OccupancyMap &map,
                              const ProgressCallback &prog = ProgressCallback());

  /// Save the results of @p query on @p map to a ply file, exporting all intersected voxels.
  /// Voxels are coloured by the reported query range values where voxels at or beyond @p colourRange
  /// are green, tending to orange closer to zero.
  /// @param fileName File to save to. Please add the .ply extension.
  /// @param map The map to save voxels from.
  /// @param query The query to save results from.
  /// @param colourRange Affects voxel colouring as described above. Green at this range.
  /// @param prog Optional function called to report on progress.
  void ohmtools_API saveQueryCloud(const char *fileName, const ohm::OccupancyMap &map,
                                   const ohm::Query &query, float colourRange = 0.0f,
                                   const ProgressCallback &prog = ProgressCallback());

  /// Save a point cloud representing the @c OccupancyNode::clearance() values for voxels in @p map.
  ///
  /// Voxels are coloured by the reported query range values where voxels at or beyond @p colourRange
  /// are green, tending to orange closer to zero.
  ///
  /// Only saves data from regions overlapping the @p minExtents and @p maxExtents, though all voxels from
  /// such regions are exported, not just those within the extents.
  ///
  /// @param fileName File to save to. Please add the .ply extension.
  /// @param map The map to save voxels from.
  /// @param minExtents Min extents to save overlapping regions from.
  /// @param maxExtents Max extents to save overlapping regions from.
  /// @param colourRange Affects voxel colouring as described above. Green at this range.
  /// @param exportType Type of voxels to export. Voxels of this @c OccupancyType or greater are exported.
  /// @param prog Optional function called to report on progress.
  size_t ohmtools_API saveClearanceCloud(const char *fileName, const ohm::OccupancyMap &map,
                                         const glm::dvec3 &minExtents, const glm::dvec3 &maxExtents,
                                         float colourRange = 0.0f, int exportType = 0,
                                         const ProgressCallback &prog = ProgressCallback());
}

#endif // OHMCLOUD_H_
