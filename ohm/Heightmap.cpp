// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Heightmap.h"

#include "private/HeightmapDetail.h"

#include "HeightmapVoxel.h"
#include "Key.h"
#include "MapCache.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"

#include <algorithm>
#include <iostream>

using namespace ohm;

namespace
{
  /// Helper class for walking a plane in the heightmap given any up axis.
  /// Manages walking the correct axis based on the Heightmap::Axis.
  ///
  /// Usage:
  /// - Initialise
  /// - call @c begin().
  /// - do work
  /// - call @c walkNext() and loop if true.
  struct PlaneWalker
  {
    const OccupancyMap &map;
    const Key &min_ext_key, &max_ext_key;
    int axis_indices[3] = { 0, 0, 0 };

    PlaneWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, Heightmap::Axis up_axis)
      : map(map)
      , min_ext_key(min_ext_key)
      , max_ext_key(max_ext_key)
    {
      switch (up_axis)
      {
      case Heightmap::AxisX:
        /* fallthrough */
      case Heightmap::AxisNegX:
        axis_indices[0] = 1;
        axis_indices[1] = 2;
        axis_indices[2] = 0;
        break;
      case Heightmap::AxisY:
        /* fallthrough */
      case Heightmap::AxisNegY:
        axis_indices[0] = 0;
        axis_indices[1] = 2;
        axis_indices[2] = 1;
        break;
      case Heightmap::AxisZ:
        /* fallthrough */
      case Heightmap::AxisNegZ:
        axis_indices[0] = 0;
        axis_indices[1] = 1;
        axis_indices[2] = 2;
        break;
      }
    }

    void begin(Key &key) const
    {
      key = min_ext_key;
      key.setRegionAxis(axis_indices[2], 0);
      key.setLocalAxis(axis_indices[2], 0);
    }

    bool walkNext(Key &key) const
    {
      map.stepKey(key, axis_indices[0], 1);
      if (!key.isBounded(axis_indices[0], min_ext_key, max_ext_key))
      {
        // Finished walking this axis. Reset and walk outer axis.
        key.setLocalAxis(axis_indices[0], min_ext_key.localKey()[axis_indices[0]]);
        key.setRegionAxis(axis_indices[0], min_ext_key.regionKey()[axis_indices[0]]);

        map.stepKey(key, axis_indices[1], 1);
        if (!key.isBounded(axis_indices[1], min_ext_key, max_ext_key))
        {
          return false;
        }
      }

      return true;
    }
  };


  inline bool voxelHeight(double *height, const VoxelConst &voxel, const glm::dvec3 &up)
  {
    if (voxel.isOccupied())
    {
      // Determine the height offset for voxel.
      const glm::dvec3 voxel_centre = voxel.centreGlobal();
      *height = glm::dot(voxel_centre, up);
      return true;
    }
    return false;
  }


  bool calculateHeightAt(double *height, const VoxelConst &voxel, Heightmap::Axis axis_id,
                         const glm::dvec3 &up, int blur_level)
  {
    if (blur_level == 0)
    {
      return voxelHeight(height, voxel, up);
    }

    double voxel_height = 0;
    bool have_height = false;

    *height = std::numeric_limits<double>::max();

    // Use deltas and axis_a, axis_b to resolve walking the plane around the blur level
    // This is setup such that when the primary axis is
    /// - X -> walk Z/Y
    /// - Y -> walk Z/X
    /// - Z -> walk Y/X
    glm::ivec3 deltas(0);
    int axis_a, axis_b;

    if (axis_id == Heightmap::AxisX || axis_id == Heightmap::AxisNegX)
    {
      axis_a = 1;
      axis_b = 2;
    }
    else if (axis_id == Heightmap::AxisY || axis_id == Heightmap::AxisNegY)
    {
      axis_a = 0;
      axis_b = 2;
    }
    else
    {
      axis_a = 0;
      axis_b = 1;
    }

    for (int db = -blur_level; db <= blur_level; ++db)
    {
      deltas[axis_b] = db;
      for (int da = -blur_level; da <= blur_level; ++da)
      {
        deltas[axis_a] = da;
        VoxelConst neighbour = voxel.neighbour(deltas[0], deltas[1], deltas[2]);
        if (voxelHeight(&voxel_height, neighbour, up))
        {
          *height = std::min(*height, voxel_height);
          have_height = true;
        }
      }
    }

    return have_height;
  }
}  // namespace

Heightmap::Heightmap(double grid_resolution, double min_clearance, Axis up_axis, unsigned region_size)
  : imp_(new HeightmapDetail)
{
  region_size = region_size ? region_size : kDefaultRegionSize;

  imp_->min_clearance = min_clearance;

  // Cache the up axis normal.
  imp_->up_axis_id = up_axis;
  switch (up_axis)
  {
  case AxisNegX:
    imp_->up = glm::dvec3(-1, 0, 0);
    imp_->vertical_axis_id = AxisX;
    break;
  case AxisNegY:
    imp_->up = glm::dvec3(0, -1, 0);
    imp_->vertical_axis_id = AxisY;
    break;
  case AxisNegZ:
    imp_->up = glm::dvec3(0, 0, -1);
    imp_->vertical_axis_id = AxisZ;
    break;
  case AxisX:
    imp_->up = glm::dvec3(1, 0, 0);
    imp_->vertical_axis_id = up_axis;
    break;
  case AxisY:
    imp_->up = glm::dvec3(0, 1, 0);
    imp_->vertical_axis_id = up_axis;
    break;
  case AxisZ:
    imp_->up = glm::dvec3(0, 0, 1);
    imp_->vertical_axis_id = up_axis;
    break;
  default:
    std::cerr << "Unknown up axis ID: " << up_axis << std::endl;
    imp_->up_axis_id = imp_->vertical_axis_id = AxisZ;
    imp_->up = glm::dvec3(0, 0, 1);
    break;
  }

  // Use an OccupancyMap to store grid cells. Each region is 1 voxel thick.
  glm::u8vec3 region_dim(region_size);
  region_dim[imp_->vertical_axis_id] = 1;
  imp_->heightmap.reset(new OccupancyMap(grid_resolution, region_dim));

  // Setup the heightmap voxel layout.
  MapLayout &layout = imp_->heightmap->layout();

  MapLayer *layer;
  VoxelLayout voxels;

  const float invalid_marker_value = voxel::invalidMarkerValue();
  const float max_clearance = std::numeric_limits<float>::max();
  int max_clearance_int = 0;
  static_assert(sizeof(max_clearance) == sizeof(max_clearance_int), "size mismatch");

  memcpy(&max_clearance_int, &max_clearance, sizeof(max_clearance));

  size_t clear_value = 0;
  memcpy(&clear_value, &invalid_marker_value, std::min(sizeof(invalid_marker_value), sizeof(clear_value)));
  layer = layout.addLayer("occupancy", 0);
  voxels = layer->voxelLayout();
  voxels.addMember("occupancy", DataType::kFloat, clear_value);

  // Initialise the data structure to have both ranges at float max.
  memset(&clear_value, max_clearance_int, sizeof(clear_value));
  layer = layout.addLayer(HeightmapVoxel::kHeightmapLayer, 0);
  imp_->heightmap_layer = layer->layerIndex();
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);
}


Heightmap::~Heightmap()
{}


void Heightmap::setOccupancyMap(OccupancyMap *map)
{
  imp_->occupancy_map = map;
}


OccupancyMap *Heightmap::occupancyMap() const
{
  return imp_->occupancy_map;
}


OccupancyMap &Heightmap::heightmap() const
{
  return *imp_->heightmap;
}


void Heightmap::setMinClearance(double clearance)
{
  imp_->min_clearance = clearance;
}


double Heightmap::minClearance() const
{
  return imp_->min_clearance;
}


void Heightmap::setBlurLevel(int blur)
{
  imp_->blur_level = blur;
}


int Heightmap::blurLevel() const
{
  return imp_->blur_level;
}


Heightmap::Axis Heightmap::upAxis() const
{
  return Heightmap::Axis(imp_->up_axis_id);
}


int Heightmap::upAxisIndex() const
{
  return imp_->vertical_axis_id;
}


const glm::dvec3 &Heightmap::upAxisNormal() const
{
  return imp_->up;
}


unsigned Heightmap::heightmapVoxelLayer() const
{
  return imp_->heightmap_layer;
}


bool Heightmap::update()
{
  if (!imp_->occupancy_map)
  {
    return false;
  }

  // Brute force initial approach.
  const OccupancyMap &src_map = *imp_->occupancy_map;
  OccupancyMap &heightmap = *imp_->heightmap;

  // Clear previous results.
  heightmap.clear();

  // 1. Calculate the map extents.
  //  a. Calculate occupancy map extents.
  //  b. Project occupancy map extents onto heightmap plane.
  // 2. Populate heightmap voxels

  glm::dvec3 min_ext, max_ext;
  src_map.calculateExtents(min_ext, max_ext);

  // Generate keys for these extents.
  const Key min_ext_key = heightmap.voxelKey(min_ext);
  const Key max_ext_key = heightmap.voxelKey(max_ext);

  // Walk the heightmap voxels which are potentially occupied. We only walk the X/Y plane.
  MapCache heightmap_cache;
  MapCache src_cache;

  // Walk the heightmap plane.
  PlaneWalker walker(heightmap, min_ext_key, max_ext_key, upAxis());
  Key target_key;

  for (walker.begin(target_key); walker.walkNext(target_key); )
  {
    HeightmapVoxel column_details;
    column_details.height = std::numeric_limits<float>::max();
    column_details.clearance = 0;
    // Start walking the voxels in the source map.
    glm::dvec3 column_reference = heightmap.voxelCentreGlobal(target_key);
    // Set to the min Z extents of the source map.
    column_reference[imp_->vertical_axis_id] = min_ext[imp_->vertical_axis_id];
    const Key src_min_key = src_map.voxelKey(column_reference);
    column_reference[imp_->vertical_axis_id] = max_ext[imp_->vertical_axis_id];
    const Key src_max_key = src_map.voxelKey(column_reference);

    // Walk the src column up.
    Key src_key = (upAxis() >= 0) ? src_min_key : src_max_key;
    // Select walking direction based on the up axis being aligned with the primary axis or not.
    const int step_dir = (upAxis() >= 0) ? 1 : -1;
    for (; src_key.isBounded(imp_->vertical_axis_id, src_min_key, src_max_key); src_map.stepKey(src_key, imp_->vertical_axis_id, step_dir))
    {
      VoxelConst src_voxel = src_map.voxel(src_key, &src_cache);

      double height = 0;

      if (calculateHeightAt(&height, src_voxel, upAxis(), imp_->up, imp_->blur_level))
      {
        if (height < column_details.height)
        {
          // First voxel in column.
          column_details.height = float(height);
        }
        else if (column_details.clearance <= 0)
        {
          // No clearance value.
          column_details.clearance = float(height - column_details.height);
          if (column_details.clearance >= imp_->min_clearance)
          {
            // Found our heightmap voxels.
            break;
          }
          else
          {
            // Insufficient clearance. This becomes our new base voxel; keep looking for clearance.
            column_details.height = float(height);
            column_details.clearance = 0;
          }
        }
      }
    }

    // Commit the voxel if required.
    if (column_details.height < std::numeric_limits<float>::max())
    {
      Voxel heightmap_voxel = heightmap.voxel(target_key, true, &heightmap_cache);
      heightmap_voxel.setValue(heightmap.occupancyThresholdValue());
      HeightmapVoxel *voxel_content = heightmap_voxel.layerContent<HeightmapVoxel *>(imp_->heightmap_layer);
      *voxel_content = column_details;
    }
  }

  return true;
}
