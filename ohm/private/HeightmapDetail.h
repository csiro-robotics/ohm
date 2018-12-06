// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPDETAIL_H
#define HEIGHTMAPDETAIL_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

#include <memory>

namespace ohm
{
  class OccupancyMap;
  class MapInfo;

  struct HeightmapDetail
  {
    ohm::OccupancyMap *occupancy_map = nullptr;
    /// Use a very thin occupancy map for the heightmap representation.
    std::unique_ptr<ohm::OccupancyMap> heightmap;
    glm::dvec3 up;
    /// Ignore all source voxels which lie lower than this below the base height.
    /// Enable by setting a positive value.
    double floor = 0;
    /// Ignore all source voxels which lie higher than this above the base height.
    /// Enable by setting a positive value.
    double ceiling = 0;
    double min_clearance = 1.0;
    /// Voxel layer containing the @c HeightmapVoxel data.
    unsigned heightmap_layer = 0;
    /// Voxel layer used to build the first pass heightmap without blur.
    unsigned heightmap_build_layer = 0;
    /// Identifies the up axis: @c Heightmap::Axis
    int up_axis_id = 0;
    /// Identifies the up axis as aligned to XYZ, [0, 2] but ignores sign/direction.
    /// Same as up_axis_id if that value is >= 0.
    int vertical_axis_id = 0;
    /// Pseudo blur factor: 2D voxel search range when building heightmap.
    /// I.e., a @c blur_level or 1 makes for an neighbourhood 3 (N3) search, level 2 is N5, 3 is N7, etc.
    unsigned blur_level = 0;

    void updateAxis();
    static const glm::dvec3 &upAxisNormal(int axis_id);

    void fromMapInfo(const MapInfo &info);
    void toMapInfo(MapInfo &info) const;
  };


  inline void HeightmapDetail::updateAxis()
  {
    up = upAxisNormal(up_axis_id);
    vertical_axis_id = (up_axis_id >= 0) ? up_axis_id : -(up_axis_id + 1);
  }
} // namespace ohm

#endif // HEIGHTMAPDETAIL_H
