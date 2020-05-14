// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPDETAIL_H
#define HEIGHTMAPDETAIL_H

#include "OhmConfig.h"

#include <ohm/Aabb.h>
#include <ohm/UpAxis.h>

#include <glm/glm.hpp>

#include <memory>

namespace ohm
{
  class OccupancyMap;
  class MapInfo;

  struct ohm_API HeightmapDetail
  {
    OccupancyMap *occupancy_map = nullptr;
    /// Use a very thin occupancy map for the heightmap representation.
    ///
    /// Within the heightmap the occupancy values are used independently of any of the threshold values.
    /// For each voxel the values have the following meanings:
    /// - @c ohm::voxel::invalidMarkerValue() => unobserved region (standard semantics)
    /// - <tt>value < 0</tt> => virtual surface
    /// - <tt>value == 0</tt> => vacant due to local cache seeding
    /// - <tt>value > 0</tt> => real surface
    ///
    /// The case where @c value is zero indicates that there are no observations for the column and it has not been
    /// able to be observed. This only comes from the @c heightmap_local_cache where the cache has been seeded at a
    /// start position and the nearby voxels cannot be observed due to sensor blind spots. Once the sensor moves the
    /// local cache will gradually be updated with real observations and this case will disappear.
    std::unique_ptr<OccupancyMap> heightmap;
    /// A cache of the previous heightmap surface around the reference position.
    /// Used to maintain previous heightmap information around the reference pos and deal with sensor visibility issues
    /// (lack thereof).
    std::unique_ptr<OccupancyMap> heightmap_local_cache;
    /// The direct of up used in heightmap generation. Must be aligned to a specific access.
    glm::dvec3 up = glm::dvec3(0, 0, 1);
    /// Ignore all source voxels which lie higher than this above the reference position height.
    /// Enable by setting a positive value.
    double ceiling = 0;
    /// Minimum clearance above a potential ground/surface voxel required to accept the voxel as a viable surface.
    double min_clearance = 1.0;
    /// Range at which to generate negative obstacle surfaces.
    double negative_obstacle_radius = 0;
    /// Voxel layer containing the @c HeightmapVoxel data.
    int heightmap_layer = -1;
    /// Identifies the up axis: @c UpAxis
    UpAxis up_axis_id = UpAxis::kZ;
    /// Identifies the up axis as aligned to XYZ, [0, 2] but ignores sign/direction.
    /// Same as up_axis_id if that value is >= 0.
    int vertical_axis_index = int(UpAxis::kZ);
    /// Should heightmap generation ignore the presence of voxel mean positions, forcing voxel centres instead?
    bool ignore_voxel_mean = false;
    /// Allow the generation of a virtual heightmap floor around the transition from unknown to free voxels?
    ///
    /// @see @c Heightmap::setGenerateVirtualFloor()
    bool generate_virtual_surface = false;
    /// Prefer a virtual surface below the reference position to a real surface above.
    /// @see @c Heightmap::setPromoteVirtualBelow()
    bool promote_virtual_below = false;
    /// Use the flood fill technique? Slower, but better at following surfaces.
    bool use_flood_fill = false;

    void updateAxis();
    static const glm::dvec3 &upAxisNormal(UpAxis axis_id);
    static int surfaceIndexA(UpAxis up_axis_id);
    static const glm::dvec3 &surfaceNormalA(UpAxis axis_id);
    static int surfaceIndexB(UpAxis up_axis_id);
    static const glm::dvec3 &surfaceNormalB(UpAxis axis_id);

    void fromMapInfo(const MapInfo &info);
    void toMapInfo(MapInfo &info) const;  // NOLINT(google-runtime-references)
  };


  inline void HeightmapDetail::updateAxis()
  {
    up = upAxisNormal(up_axis_id);
    vertical_axis_index = (int(up_axis_id) >= 0) ? int(up_axis_id) : -(int(up_axis_id) + 1);
  }
}  // namespace ohm

#endif  // HEIGHTMAPDETAIL_H
