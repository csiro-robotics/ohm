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

// Post blur does not give as good results. Leave the slow algorithm for now.
#define POST_BLUR 0

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


  inline float relativeVoxelHeight(double absolute_height, const VoxelConst &voxel, const glm::dvec3 &up)
  {
    const float relative_height = float(absolute_height - glm::dot(voxel.centreGlobal(), up));
    return relative_height;
  }


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


  bool calculateHeightAt(double *height, const VoxelConst &voxel, Heightmap::Axis axis_id, const glm::dvec3 &up,
                         int blur_level)
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

  if (up_axis < AxisNegZ || up_axis > AxisZ)
  {
    std::cerr << "Unknown up axis ID: " << up_axis << std::endl;
    up_axis = AxisZ;
  }

  // Cache the up axis normal.
  imp_->up_axis_id = up_axis;
  imp_->up = upAxisNormal(up_axis);
  imp_->vertical_axis_id = (up_axis >= 0) ? up_axis : -(up_axis + 1);

  // Use an OccupancyMap to store grid cells. Each region is 1 voxel thick.
  glm::u8vec3 region_dim(region_size);
  region_dim[imp_->vertical_axis_id] = 1;
  imp_->heightmap.reset(new OccupancyMap(grid_resolution, region_dim));

  // Setup the heightmap voxel layout.
  MapLayout &layout = imp_->heightmap->layout();

  layout.filterLayers({ defaultLayerName(kDlOccupancy) });

  MapLayer *layer;
  VoxelLayout voxels;

  const float max_clearance = std::numeric_limits<float>::max();
  int max_clearance_int = 0;
  static_assert(sizeof(max_clearance) == sizeof(max_clearance_int), "size mismatch");

  memcpy(&max_clearance_int, &max_clearance, sizeof(max_clearance));

  size_t clear_value = 0;
  // Initialise the data structure to have both ranges at float max.
  memset(&clear_value, max_clearance_int, sizeof(clear_value));
  layer = layout.addLayer(HeightmapVoxel::kHeightmapLayer, 0);
  imp_->heightmap_layer = layer->layerIndex();
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);

  layer = layout.addLayer(HeightmapVoxel::kHeightmapBuildLayer, 0);
  imp_->heightmap_build_layer = layer->layerIndex();
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


const glm::dvec3 &Heightmap::upAxisNormal(int axis_id)
{
  static const glm::dvec3 kAxes[] =  //
    {
      glm::dvec3(0, 0, -1),  // -Z
      glm::dvec3(0, -1, 0),  // -Y
      glm::dvec3(-1, 0, 0),  // -X
      glm::dvec3(1, 0, 0),   // X
      glm::dvec3(0, 1, 0),   // Y
      glm::dvec3(0, 0, 1),   // Z
      glm::dvec3(0, 0, 0),   // Dummy
    };

  unsigned axis_index = unsigned(axis_id - AxisNegZ);
  if (axis_index < 0 || axis_index >= sizeof(kAxes) - sizeof(kAxes[0]))
  {
    // Reference the dummy index.
    axis_index = sizeof(kAxes) - sizeof(kAxes[0]) - 1;
  };

  return kAxes[axis_index];
}


unsigned Heightmap::heightmapVoxelLayer() const
{
  return imp_->heightmap_layer;
}


double Heightmap::baseHeight() const
{
  return glm::dot(upAxisNormal(), imp_->heightmap->origin());
}


bool Heightmap::update(double base_height)
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

  heightmap.setOrigin(upAxisNormal() * base_height);

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

#if POST_BLUR
  unsigned heightmap_build_layer = (imp_->blur_level) ? imp_->heightmap_build_layer : imp_->heightmap_layer;
#else   // POST_BLUR
  const unsigned heightmap_build_layer = imp_->heightmap_layer;
#endif  // POST_BLUR

  for (walker.begin(target_key); walker.walkNext(target_key);)
  {
    double column_height = std::numeric_limits<double>::max();
    double column_clearance_height = column_height;
    // Start walking the voxels in the source map.
    glm::dvec3 column_reference = heightmap.voxelCentreGlobal(target_key);
    // Set to the min Z extents of the source map.
    column_reference[imp_->vertical_axis_id] = min_ext[imp_->vertical_axis_id];
    const Key src_min_key = src_map.voxelKey(column_reference);
    column_reference[imp_->vertical_axis_id] = max_ext[imp_->vertical_axis_id];
    const Key src_max_key = src_map.voxelKey(column_reference);

    // Walk the src column up.
    Key src_key = (upAxis() >= 0) ? src_min_key : src_max_key;
    double height = 0;

    // Select walking direction based on the up axis being aligned with the primary axis or not.
    const int step_dir = (upAxis() >= 0) ? 1 : -1;
    for (; src_key.isBounded(imp_->vertical_axis_id, src_min_key, src_max_key);
         src_map.stepKey(src_key, imp_->vertical_axis_id, step_dir))
    {
      VoxelConst src_voxel = src_map.voxel(src_key, &src_cache);

#if POST_BLUR
      if (voxelHeight(&height, src_voxel, imp_->up))
#else   // POST_BLUR
      if (calculateHeightAt(&height, src_voxel, upAxis(), imp_->up, imp_->blur_level))
#endif  // POST_BLUR
      {
        if (height < column_height)
        {
          // First voxel in column.
          column_height = column_clearance_height = height;
        }
        else if (column_clearance_height == column_height)
        {
          // No clearance value.
          column_clearance_height = height;
          if (column_clearance_height - column_height >= imp_->min_clearance)
          {
            // Found our heightmap voxels.
            break;
          }
          else
          {
            // Insufficient clearance. This becomes our new base voxel; keep looking for clearance.
            column_height = column_clearance_height = height;
          }
        }
      }
    }

    // Commit the voxel.
    Voxel heightmap_voxel = heightmap.voxel(target_key, true, &heightmap_cache);
    HeightmapVoxel *voxel_content = heightmap_voxel.layerContent<HeightmapVoxel *>(heightmap_build_layer);
    voxel_content->height = relativeVoxelHeight(column_height, heightmap_voxel, upAxisNormal());
    voxel_content->clearance = float(column_clearance_height - column_height);
    if (column_height < std::numeric_limits<double>::max())
    {
      heightmap_voxel.setValue(heightmap.occupancyThresholdValue());
    }
  }

#if POST_BLUR
  if (imp_->blur_level)
  {
    // Walk the heightmap applying blur.
    for (auto &&heightmap_voxel : heightmap)
    {
      // First migrate build layer to final layer.
      HeightmapVoxel *voxel_content = heightmap_voxel.layerContent<HeightmapVoxel *>(imp_->heightmap_layer);
      *voxel_content = *heightmap_voxel.layerContent<HeightmapVoxel *>(imp_->heightmap_build_layer);
      double voxel_height = glm::dot(upAxisNormal(), heightmap_voxel.centreGlobal()) + voxel_content->height;
      double clearance_height = voxel_height + voxel_content->clearance;
      for (int j = -imp_->blur_level; j <= int(imp_->blur_level); ++j)
      {
        for (int i = -imp_->blur_level; i <= int(imp_->blur_level); ++i)
        {
          // Ignore the voxel itself.
          if (i || j)
          {
            int dx = 0, dy = 0, dz = 0;
            switch (imp_->up_axis_id)
            {
            case Heightmap::AxisX:
              /* fallthrough */
            case Heightmap::AxisNegX:
              dy = i;
              dz = j;
              break;
            case Heightmap::AxisY:
              /* fallthrough */
            case Heightmap::AxisNegY:
              dx = i;
              dz = j;
              break;
            case Heightmap::AxisZ:
              /* fallthrough */
            case Heightmap::AxisNegZ:
              dx = i;
              dy = j;
              break;
            }

            // Set deltas by the upAxis().
            Voxel neighbour = heightmap_voxel.neighbour(dx, dy, dz);
            if (neighbour.isValid())
            {
              const HeightmapVoxel *neighbour_content =
                neighbour.layerContent<HeightmapVoxel *>(imp_->heightmap_build_layer);
              // Ignore unoccupied neighbours.
              if (neighbour_content->height < std::numeric_limits<float>::max())
              {
                double neighbour_height = glm::dot(upAxisNormal(), neighbour.centreGlobal()) + neighbour_content->height;

                if (!heightmap_voxel.isOccupied())
                {
                  // Bluring into an empty voxel.
                  voxel_height = neighbour_height;
                  clearance_height = neighbour_height + neighbour_content->clearance;
                  // Mark voxel as occupied.
                  heightmap_voxel.setValue(imp_->heightmap->occupancyThresholdValue());
                }
                else
                {
                  // Adjusting existing value.
                  if (neighbour_height > voxel_height)
                  {
                    // No clearance value.
                    voxel_height = neighbour_height;
                  }
                  else if (neighbour_height < clearance_height)
                  {
                    clearance_height = neighbour_height;
                  }
                }
              }
            }
          }
        }
      }

      // Finish transfer and blur.
      voxel_content->height = float(voxel_height - glm::dot(heightmap_voxel.centreGlobal(), upAxisNormal()));
      voxel_content->clearance = clearance_height - voxel_height;
    }
  }
#endif  // POST_BLUR

  return true;
}
