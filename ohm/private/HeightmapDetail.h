// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPDETAIL_H
#define HEIGHTMAPDETAIL_H

#include "OhmConfig.h"

#include <ohm/UpAxis.h>

#include <glm/glm.hpp>

#include <memory>

namespace ohm
{
  class OccupancyMap;
  class MapInfo;

  struct ohm_API HeightmapDetail
  {
    ohm::OccupancyMap *occupancy_map = nullptr;
    /// Use a very thin occupancy map for the heightmap representation.
    std::unique_ptr<ohm::OccupancyMap> heightmap;
    glm::dvec3 up = glm::dvec3(0, 0, 1);
    /// Ignore all source voxels which lie lower than this below the base height.
    /// Enable by setting a positive value.
    double floor = 0;
    /// Ignore all source voxels which lie higher than this above the base height.
    /// Enable by setting a positive value.
    double ceiling = 0;
    /// Minimum clearance above a potential ground/surface voxel required to accept the voxel as a viable surface.
    double min_clearance = 1.0;
    double negative_obstacle_radius = 3.0;
    /// Voxel layer containing the @c HeightmapVoxel data.
    int heightmap_layer = -1;
    /// Voxel layer used to build the first pass heightmap without blur.
    int heightmap_build_layer = -1;
    /// Identifies the up axis: @c UpAxis
    UpAxis up_axis_id = UpAxis::kZ;
    /// Identifies the up axis as aligned to XYZ, [0, 2] but ignores sign/direction.
    /// Same as up_axis_id if that value is >= 0.
    int vertical_axis_index = int(UpAxis::kZ);
    /// Target number of threads to use. 1 => no threading.
    unsigned thread_count = 1;
    /// Should heightmap generation ignore the presence of sub-voxel positions, forcing voxel centres instead?
    bool ignore_sub_voxel_positioning = false;
    /// Allow the generation of a heightmap floor around the transition from unknown to free voxels?
    ///
    /// @see @c Heightmap::setGenerateFloorFromUnknown()
    bool generate_floor_from_unknown = false;

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
