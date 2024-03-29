// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMHEIGHTMAP_HEIGHTMAPDETAIL_H
#define OHMHEIGHTMAP_HEIGHTMAPDETAIL_H

#include "OhmHeightmapConfig.h"

#include "ohmheightmap/HeightmapMode.h"
#include "ohmheightmap/UpAxis.h"

#include <ohm/Aabb.h>

#include <glm/glm.hpp>

#include <memory>

namespace ohm
{
class OccupancyMap;
class MapInfo;

/// Pimpl data for @c Heightmap .
struct ohmheightmap_API HeightmapDetail
{
  /// Source occupancy map pointer from which we are building a heightmap.
  const OccupancyMap *occupancy_map = nullptr;
  /// Use a very thin occupancy map for the heightmap representation.
  ///
  /// Within the heightmap the occupancy values are used independently of any of the threshold values.
  /// For each voxel the values have the following meanings:
  /// - @c ohm::unobservedOccupancyValue() => unobserved region (standard semantics)
  /// - <tt>value < 0</tt> => virtual surface
  /// - <tt>value == 0</tt> => vacant due to local cache seeding
  /// - <tt>value > 0</tt> => real surface
  ///
  /// The case where @c value is zero indicates that there are no observations for the column and it has not been
  /// able to be observed.
  std::unique_ptr<OccupancyMap> heightmap;
  /// Similar to @c heightmap , this is an occupancy map into which we build a multilayer heightmap.
  std::unique_ptr<OccupancyMap> multilayer_heightmap;
  /// The direct of up used in heightmap generation. Must be aligned to a specific access.
  glm::dvec3 up = glm::dvec3(0, 0, 1);
  /// Ignore all source voxels which lie higher than this above the seed voxel height.
  /// Enable by setting a positive, non zero value.
  double ceiling = 0;
  /// Ignore all source voxels which lie a distance greater than this below the seed voxel height.
  /// Enable by setting a positive, non zero value.
  double floor = 0;
  /// Minimum clearance above a potential ground/surface voxel required to accept the voxel as a viable surface.
  double min_clearance = 1.0;
  /// Voxel layer containing the @c HeightmapVoxel data in the @c heightmap.
  int heightmap_voxel_layer = -1;
  /// Identifies the up axis: @c UpAxis
  UpAxis up_axis_id = UpAxis::kZ;
  /// Identifies the up axis as aligned to XYZ, [0, 2] but ignores sign/direction.
  /// Same as up_axis_id if that value is >= 0.
  int vertical_axis_index = int(UpAxis::kZ);
  /// Enables post process filtering for layered heightmaps, removing virtual surface voxels with fewer 26-connected
  /// populated neighbours.
  unsigned virtual_surface_filter_threshold = 0;
  /// Level of debugging information provided in generating the heightmap. Only has an effect if
  /// @c OHM_TES_DEBUG is configured in CMake.
  int debug_level = 0;
  /// Specifies the heightmap generation mode.
  HeightmapMode mode = HeightmapMode::kPlanar;
  /// Should heightmap generation ignore the presence of voxel mean positions, forcing voxel centres instead?
  bool ignore_voxel_mean = false;
  /// Allow the generation of a virtual heightmap floor around the transition from unknown to free voxels?
  ///
  /// @see @c Heightmap::setGenerateVirtualFloor()
  bool generate_virtual_surface = false;
  /// Prefer a virtual surface below the reference position to a real surface above.
  /// @see @c Heightmap::setPromoteVirtualBelow()
  bool promote_virtual_below = false;

  ~HeightmapDetail();

  void updateAxis();
  static const glm::dvec3 &upAxisNormal(UpAxis axis_id);
  static int surfaceIndexA(UpAxis up_axis_id);
  static const glm::dvec3 &surfaceNormalA(UpAxis axis_id);
  static int surfaceIndexB(UpAxis up_axis_id);
  static const glm::dvec3 &surfaceNormalB(UpAxis axis_id);

  void fromMapInfo(const MapInfo &info);
  void toMapInfo(MapInfo &info) const;
};


inline void HeightmapDetail::updateAxis()
{
  up = upAxisNormal(up_axis_id);
  vertical_axis_index = (int(up_axis_id) >= 0) ? int(up_axis_id) : -(int(up_axis_id) + 1);
}
}  // namespace ohm

#endif  // OHMHEIGHTMAP_HEIGHTMAPDETAIL_H
