// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Heightmap.h"

#include "private/HeightmapDetail.h"

#include "Aabb.h"
#include "HeightmapVoxel.h"
#include "Key.h"
#include "MapCache.h"
#include "MapChunk.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"

#include <algorithm>
#include <iostream>

#define PROFILING 0
#include <ohmutil/Profile.h>

// #undef OHM_THREADS
#ifdef OHM_THREADS
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#endif // OHM_THREADS

using namespace ohm;

namespace
{
  /// Helper class for walking a plane in the heightmap given any up axis.
  /// Manages walking the correct axis based on the @c UpAxis.
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

    PlaneWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis)
      : map(map)
      , min_ext_key(min_ext_key)
      , max_ext_key(max_ext_key)
    {
      switch (up_axis)
      {
      case UpAxis::X:
        /* fallthrough */
      case UpAxis::NegX:
        axis_indices[0] = 1;
        axis_indices[1] = 2;
        axis_indices[2] = 0;
        break;
      case UpAxis::Y:
        /* fallthrough */
      case UpAxis::NegY:
        axis_indices[0] = 0;
        axis_indices[1] = 2;
        axis_indices[2] = 1;
        break;
      case UpAxis::Z:
        /* fallthrough */
      case UpAxis::NegZ:
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


  inline bool sourceVoxelHeight(glm::dvec3 *voxel_position, double *height, const VoxelConst &voxel,
                                const glm::dvec3 &up, bool force_voxel_centre)
  {
    if (voxel.isOccupied())
    {
      // Determine the height offset for voxel.
      *voxel_position = (force_voxel_centre) ? voxel.centreGlobal() : voxel.position();
      *height = glm::dot(*voxel_position, up);
      return true;
    }
    return false;
  }


  void updateHeightmapForRegion(HeightmapDetail *imp, double base_height, const Key &min_ext_key,
                                const Key &max_ext_key, UpAxis up_axis, const Aabb &src_map_extents)
  {
    // Walk the heightmap voxels which are potentially occupied. We only walk the X/Y plane.
    MapCache heightmap_cache;
    MapCache src_cache;
    const OccupancyMap &src_map = *imp->occupancy_map;
    OccupancyMap &heightmap = *imp->heightmap;
    PlaneWalker walker(heightmap, min_ext_key, max_ext_key, up_axis);
    Key target_key;

    const int heightmap_build_layer = imp->heightmap_layer;

    walker.begin(target_key);
    do
    {
      // PROFILE(walk_voxel)
      double column_height = std::numeric_limits<double>::max();
      double column_clearance_height = column_height;
      // Start walking the voxels in the source map.
      glm::dvec3 column_reference = heightmap.voxelCentreGlobal(target_key);
      // Set to the min Z extents of the source map.
      column_reference[imp->vertical_axis_id] = src_map_extents.minExtents()[imp->vertical_axis_id];
      const Key src_min_key = src_map.voxelKey(column_reference);
      column_reference[imp->vertical_axis_id] = src_map_extents.maxExtents()[imp->vertical_axis_id];
      const Key src_max_key = src_map.voxelKey(column_reference);

      // Walk the src column up.
      Key src_key = (int(up_axis) >= 0) ? src_min_key : src_max_key;
      glm::dvec3 sub_voxel_pos(0);
      glm::dvec3 column_voxel_pos(0);
      double height = 0;

      // Select walking direction based on the up axis being aligned with the primary axis or not.
      const int step_dir = (int(up_axis) >= 0) ? 1 : -1;
      for (; src_key.isBounded(imp->vertical_axis_id, src_min_key, src_max_key);
          src_map.stepKey(src_key, imp->vertical_axis_id, step_dir))
      {
        // PROFILE(column);
        VoxelConst src_voxel = src_map.voxel(src_key, &src_cache);

        if (sourceVoxelHeight(&sub_voxel_pos, &height, src_voxel, imp->up, imp->ignore_sub_voxel_positioning))
        {
          if (imp->floor > 0 && height < base_height - imp->floor)
          {
            // Below floor: don't start heightmap yet.
            continue;
          }

          if (imp->ceiling > 0 && height > base_height + imp->ceiling)
          {
            // Above ceiling: done with this column.
            break;
          }

          if (height < column_height)
          {
            // First voxel in column.
            column_height = column_clearance_height = height;
            column_voxel_pos = sub_voxel_pos;
          }
          else if (column_clearance_height == column_height)
          {
            // No clearance value.
            column_clearance_height = height;
            if (column_clearance_height - column_height >= imp->min_clearance)
            {
              // Found our heightmap voxels.
              break;
            }
            else
            {
              // Insufficient clearance. This becomes our new base voxel; keep looking for clearance.
              column_height = column_clearance_height = height;
              column_voxel_pos = sub_voxel_pos;
            }
          }
        }
      }

      // PROFILE(commit)
      // Commit the voxel.
      Voxel heightmap_voxel = heightmap.voxel(target_key, true, &heightmap_cache);
      HeightmapVoxel *voxel_content = heightmap_voxel.layerContent<HeightmapVoxel *>(heightmap_build_layer);
      voxel_content->height = relativeVoxelHeight(column_height, heightmap_voxel, imp->up);
      voxel_content->clearance = float(column_clearance_height - column_height);
      if (column_height < std::numeric_limits<double>::max())
      {
        heightmap_voxel.setValue(heightmap.occupancyThresholdValue());
        if (!imp->ignore_sub_voxel_positioning)
        {
          // Reset to voxel centre on the primary axis because the height is encoded in the heightmap layer.
          column_voxel_pos[imp->vertical_axis_id] = heightmap_voxel.centreGlobal()[imp->vertical_axis_id];
          heightmap_voxel.setPosition(column_voxel_pos);
        }
      }
    } while (walker.walkNext(target_key));
  }
}  // namespace

Heightmap::Heightmap()
  : Heightmap(0.2, 2.0, UpAxis::Z)
{}


Heightmap::Heightmap(double grid_resolution, double min_clearance, UpAxis up_axis, unsigned region_size)
  : imp_(new HeightmapDetail)
{
  region_size = region_size ? region_size : kDefaultRegionSize;

  imp_->min_clearance = min_clearance;

  if (up_axis < UpAxis::NegZ || up_axis > UpAxis::Z)
  {
    std::cerr << "Unknown up axis ID: " << int(up_axis) << std::endl;
    up_axis = UpAxis::Z;
  }

  // Cache the up axis normal.
  imp_->up_axis_id = up_axis;
  imp_->updateAxis();

  // Use an OccupancyMap to store grid cells. Each region is 1 voxel thick.
  glm::u8vec3 region_dim(region_size);
  region_dim[int(imp_->vertical_axis_id)] = 1;
  imp_->heightmap.reset(new OccupancyMap(grid_resolution, region_dim));

  // Setup the heightmap voxel layout.
  MapLayout &layout = imp_->heightmap->layout();

  layout.filterLayers({ default_layer::occupancyLayerName() });

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
  imp_->heightmap_layer = (int)layer->layerIndex();
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);

  layer = layout.addLayer(HeightmapVoxel::kHeightmapBuildLayer, 0);
  imp_->heightmap_build_layer = (int)layer->layerIndex();
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);

  updateMapInfo(imp_->heightmap->mapInfo());
}


Heightmap::~Heightmap()
{}


bool Heightmap::setThreadCount(unsigned thread_count)
{
#ifdef OHM_THREADS
  imp_->thread_count = thread_count;
  return true;
#else  // OHM_THREADS
  (void)thread_count; // Unused.
  return false;
#endif // OHM_THREADS
}


unsigned Heightmap::threadCount() const
{
  return imp_->thread_count;
}


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


void Heightmap::setFloor(double floor)
{
  imp_->floor = floor;
}


double Heightmap::floor() const
{
  return imp_->floor;
}


void Heightmap::setCeiling(double ceiling)
{
  imp_->ceiling = ceiling;
}


double Heightmap::ceiling() const
{
  return imp_->ceiling;
}


void Heightmap::setMinClearance(double clearance)
{
  imp_->min_clearance = clearance;
}


double Heightmap::minClearance() const
{
  return imp_->min_clearance;
}


void Heightmap::setIgnoreSubVoxelPositioning(bool ignore)
{
  imp_->ignore_sub_voxel_positioning = ignore;
}


bool Heightmap::ignoreSubVoxelPositioning() const
{
  return imp_->ignore_sub_voxel_positioning;
}


UpAxis Heightmap::upAxis() const
{
  return UpAxis(imp_->up_axis_id);
}


int Heightmap::upAxisIndex() const
{
  return int(imp_->vertical_axis_id);
}


const glm::dvec3 &Heightmap::upAxisNormal() const
{
  return imp_->up;
}


const glm::dvec3 &Heightmap::surfaceAxisA() const
{
  return HeightmapDetail::surfaceNormalA(int(imp_->up_axis_id));
}


const glm::dvec3 &Heightmap::surfaceAxisB() const
{
  return HeightmapDetail::surfaceNormalB(int(imp_->up_axis_id));
}


const glm::dvec3 &Heightmap::upAxisNormal(int axis_id)
{
  return HeightmapDetail::upAxisNormal(axis_id);
}


const glm::dvec3 &Heightmap::surfaceAxisA(int axis_id)
{
  return HeightmapDetail::surfaceNormalA(axis_id);
}


const glm::dvec3 &Heightmap::surfaceAxisB(int axis_id)
{
  return HeightmapDetail::surfaceNormalB(axis_id);
}


int Heightmap::heightmapVoxelLayer() const
{
  return imp_->heightmap_layer;
}


int Heightmap::heightmapVoxelBuildLayer() const
{
  return imp_->heightmap_build_layer;
}


double Heightmap::baseHeight() const
{
  return glm::dot(upAxisNormal(), imp_->heightmap->origin());
}


bool Heightmap::update(double base_height, const ohm::Aabb &cull_to)
{
  if (!imp_->occupancy_map)
  {
    return false;
  }

  // Brute force initial approach.
  const OccupancyMap &src_map = *imp_->occupancy_map;
  OccupancyMap &heightmap = *imp_->heightmap;

  updateMapInfo(heightmap.mapInfo());

  // Clear previous results.
  heightmap.clear();

  // Allow sub-voxel positioning.
  const bool sub_voxel_allowed = !imp_->ignore_sub_voxel_positioning;
  heightmap.setSubVoxelsEnabled(src_map.subVoxelsEnabled() && sub_voxel_allowed);

  heightmap.setOrigin(upAxisNormal() * base_height);

  // 1. Calculate the map extents.
  //  a. Calculate occupancy map extents.
  //  b. Project occupancy map extents onto heightmap plane.
  // 2. Populate heightmap voxels

  glm::dvec3 min_ext, max_ext;
  src_map.calculateExtents(min_ext, max_ext);

  // Clip to the cull box.
  for (int i = 0; i < 3; ++i)
  {
    if (cull_to.diagonal()[i] > 0)
    {
      min_ext[i] = std::max(cull_to.minExtents()[i], min_ext[i]);
      max_ext[i] = std::min(cull_to.maxExtents()[i], max_ext[i]);
    }
  }

  // Collapse the vertical axis for the keys.
  glm::dvec3 min_ext_collapsed = min_ext;
  glm::dvec3 max_ext_collapsed = max_ext;

  min_ext_collapsed[upAxisIndex()] = max_ext_collapsed[upAxisIndex()] = 0.0;

  // Generate keys for these extents.
  const Key min_ext_key = heightmap.voxelKey(min_ext_collapsed);
  const Key max_ext_key = heightmap.voxelKey(max_ext_collapsed);

  PROFILE(walk)
#ifdef OHM_THREADS
  if (imp_->thread_count != 1)
  {
    const auto updateHeightmapBlock = [&] (const tbb::blocked_range3d<unsigned, unsigned, unsigned> &range) //
    { //
      Key min_key_local = min_ext_key;
      Key max_key_local = min_ext_key;

      // Move to the target offset.
      imp_->heightmap->moveKey(min_key_local, range.cols().begin(), range.rows().begin(), range.pages().begin());
      imp_->heightmap->moveKey(max_key_local, range.cols().end() - 1, range.rows().end() - 1, range.pages().end() - 1);
      updateHeightmapForRegion(imp_.get(), base_height, min_key_local, max_key_local, upAxis(), Aabb(min_ext, max_ext));
    };

    const glm::ivec3 voxel_range = heightmap.rangeBetween(min_ext_key, max_ext_key);

    const auto parallel_loop = [&]  //
    {
      tbb::parallel_for(tbb::blocked_range3d<unsigned, unsigned, unsigned>(0, voxel_range.z + 1,  //
                                                                           0, voxel_range.y + 1,  //
                                                                           0, voxel_range.x + 1),
                        updateHeightmapBlock);
      // const unsigned grain_size = 8;
      // tbb::parallel_for(tbb::blocked_range3d<unsigned, unsigned, unsigned>(0, voxel_range.z + 1, grain_size,  //
      //                                                                      0, voxel_range.y + 1, grain_size,  //
      //                                                                      0, voxel_range.x + 1, grain_size),
      //                   updateHeightmapBlock);
    };

    if (imp_->thread_count)
    {
      tbb::task_arena limited_arena(imp_->thread_count);
      limited_arena.execute(parallel_loop);
    }
    else
    {
      parallel_loop();
    }
  }
  else
#endif  // OHM_THREADS
  {
    updateHeightmapForRegion(imp_.get(), base_height, min_ext_key, max_ext_key, upAxis(), Aabb(min_ext, max_ext));
  }
  PROFILE_END(walk)

  return true;
}


void Heightmap::updateMapInfo(MapInfo &info) const
{
  imp_->toMapInfo(info);
}
