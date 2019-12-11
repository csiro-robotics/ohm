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
#include "MapCoord.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "OccupancyType.h"
#include "PlaneFillWalker.h"
#include "PlaneWalker.h"
#include "Voxel.h"

#include <algorithm>
#include <cstring>
#include <iostream>

#define PROFILING 0
#include <ohmutil/Profile.h>

// #undef OHM_THREADS
#ifdef OHM_THREADS
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#endif  // OHM_THREADS

using namespace ohm;

namespace
{
  inline float relativeVoxelHeight(double absolute_height, const VoxelConst &voxel, const glm::dvec3 &up)
  {
    const float relative_height = float(absolute_height - glm::dot(voxel.centreGlobal(), up));
    return relative_height;
  }


  inline ohm::OccupancyType sourceVoxelHeight(glm::dvec3 *voxel_position, double *height, const OccupancyMap &map,
                                              const VoxelConst &voxel, const Key &voxel_key, const glm::dvec3 &up,
                                              bool force_voxel_centre)
  {
    ohm::OccupancyType voxel_type = ohm::OccupancyType(map.occupancyType(voxel));
    if (voxel_type == ohm::kOccupied)
    {
      // Determine the height offset for voxel.
      *voxel_position = (force_voxel_centre) ? voxel.centreGlobal() : voxel.position();
    }
    else
    {
      // Return the voxel centre. Voxel may be invalid, so use the map interface on the key.
      *voxel_position = map.voxelCentreGlobal(voxel_key);
    }
    *height = glm::dot(*voxel_position, up);

    return voxel_type;
  }


  unsigned updateHeightmapForRegion(HeightmapDetail *imp, double base_height, const Key &min_ext_key,
                                    const Key &max_ext_key, UpAxis up_axis, const Aabb &src_map_extents)
  {
    // Walk the heightmap voxels which are potentially occupied. We only walk the X/Y plane.
    MapCache heightmap_cache;
    MapCache src_cache;
    const OccupancyMap &src_map = *imp->occupancy_map;
    OccupancyMap &heightmap = *imp->heightmap;
    PlaneWalker walker(heightmap, min_ext_key, max_ext_key, up_axis);
    Key target_key;
    unsigned populated_count = 0;

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
      column_reference[imp->vertical_axis_index] = src_map_extents.minExtents()[imp->vertical_axis_index];
      const Key src_min_key = src_map.voxelKey(column_reference);
      column_reference[imp->vertical_axis_index] = src_map_extents.maxExtents()[imp->vertical_axis_index];
      const Key src_max_key = src_map.voxelKey(column_reference);

      // Walk the src column up.
      Key src_key = (int(up_axis) >= 0) ? src_min_key : src_max_key;
      glm::dvec3 sub_voxel_pos(0);
      glm::dvec3 column_voxel_pos(0);
      double height = 0;
      bool have_transitioned_from_unknown = false;

      // Select walking direction based on the up axis being aligned with the primary axis or not.
      const int step_dir = (int(up_axis) >= 0) ? 1 : -1;
      for (; src_key.isBounded(imp->vertical_axis_index, src_min_key, src_max_key);
           src_map.stepKey(src_key, imp->vertical_axis_index, step_dir))
      {
        // PROFILE(column);
        VoxelConst src_voxel = src_map.voxel(src_key, &src_cache);

        const ohm::OccupancyType voxel_type = sourceVoxelHeight(&sub_voxel_pos, &height, src_map, src_voxel, src_key,
                                                                imp->up, imp->ignore_sub_voxel_positioning);
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

        if (voxel_type == ohm::kOccupied ||
            imp->generate_floor_from_unknown && !have_transitioned_from_unknown && voxel_type == ohm::kFree)
        {
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

            // Insufficient clearance. This becomes our new base voxel; keep looking for clearance.
            column_height = column_clearance_height = height;
            column_voxel_pos = sub_voxel_pos;
          }
        }

        if (voxel_type != ohm::kUncertain)
        {
          have_transitioned_from_unknown = true;
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
        ++populated_count;
        if (!imp->ignore_sub_voxel_positioning)
        {
          // Reset to voxel centre on the primary axis because the height is encoded in the heightmap layer.
          column_voxel_pos[imp->vertical_axis_index] = heightmap_voxel.centreGlobal()[imp->vertical_axis_index];
          heightmap_voxel.setPosition(column_voxel_pos);
        }
      }
    } while (walker.walkNext(target_key));

    return populated_count;
  }

  Key findNearestSupportingVoxel(const OccupancyMap &map, const Key &seed_key, int up_axis_index,  //
                                 const Key &min_key, const Key &max_key, int voxel_ceiling, bool floor_from_unknown)
  {
    // Walk up and down at the same time until we find the nearest occupied voxel.
    int key_offset = 0;
    Key best_unknown_floor = Key::kNull;
    bool in_range = false;  // False when both keys above and below are invalid.
    do
    {
      Key above = seed_key;
      Key below = seed_key;
      in_range = false;

      map.moveKeyAlongAxis(above, up_axis_index, key_offset);
      map.moveKeyAlongAxis(below, up_axis_index, -key_offset);

      // Preference below.
      if (below.isBounded(up_axis_index, min_key, max_key))
      {
        // Key still valid. Ensure we keep walking.
        in_range = true;
        switch (map.occupancyType(map.voxel(below)))
        {
        case ohm::kOccupied:
          // Found a occupied voxel. This is the floor candidate.
          return below;
        case ohm::kFree:
          // Free voxel. Track this voxel as a potential floor voxel for generating floor_from_unknown.
          best_unknown_floor = below;
          break;
        default:
          break;
        }
      }

      // Limit how much we search up by the voxel_ceiling (if set).
      if ((voxel_ceiling <= 0 || key_offset <= voxel_ceiling) && above.isBounded(up_axis_index, min_key, max_key))
      {
        // Key still valid. Ensure we keep walking.
        in_range = true;
        if (map.voxel(above).isOccupied())
        {
          return above;
        }
        switch (map.occupancyType(map.voxel(above)))
        {
        case ohm::kOccupied:
          // Found a occupied voxel. This is the floor candidate.
          return above;
        case ohm::kFree:
          // Free voxel. Track this voxel as a potential floor voxel for generating floor_from_unknown so long as we
          // don't already have one. This ensures we only track the lowest candidate.
          if (best_unknown_floor.isNull())
          {
            best_unknown_floor = above;
          }
          break;
        default:
          break;
        }
      }

      ++key_offset;
    } while (in_range);

    if (floor_from_unknown)
    {
      return best_unknown_floor;
    }

    return Key::kNull;
  }


  Key findGround(double *height_out, double *clearance_out, const OccupancyMap &map, const Key &seed_key,
                 const Key &min_key, const Key &max_key, MapCache *cache, const HeightmapDetail &imp)
  {
    // Start with the seed_key and look for ground. We only walk up from the seed key.
    double column_height = std::numeric_limits<double>::max();
    double column_clearance_height = column_height;

    // Start walking the voxels in the source map.
    // glm::dvec3 column_reference = heightmap.voxelCentreGlobal(target_key);

    // Walk the src column up.
    const int up_axis_index = std::abs(int(imp.up_axis_id));
    const int step_dir = (int(imp.up_axis_id) >= 0) ? 1 : -1;
    glm::dvec3 sub_voxel_pos(0);
    glm::dvec3 column_voxel_pos(0);
    double height = 0;
    bool have_candidate = false;
    bool have_transitioned_from_unknown = false;

    // Select walking direction based on the up axis being aligned with the primary axis or not.

    Key ground_key = Key::kNull;
    for (Key key = seed_key; key.isBounded(up_axis_index, min_key, max_key); map.stepKey(key, up_axis_index, step_dir))
    {
      // PROFILE(column);
      const VoxelConst voxel = map.voxel(key, cache);

      const ohm::OccupancyType voxel_type =
        sourceVoxelHeight(&sub_voxel_pos, &height, map, voxel, key, imp.up, imp.ignore_sub_voxel_positioning);

      if (voxel_type == ohm::kOccupied ||
          imp.generate_floor_from_unknown && !have_transitioned_from_unknown && voxel_type == ohm::kFree)
      {
        if (have_candidate)
        {
          // Branch condition where we have a candidate ground_key, but have yet to check or record its clearance.
          // Clearance height is the height of the current voxel associated with key.
          column_clearance_height = height;
          if (column_clearance_height - column_height >= imp.min_clearance)
          {
            // Found our heightmap voxels.
            // We have sufficient clearance so ground_key is our ground voxel.
            break;
          }

          // Insufficient clearance. The current voxel becomes our new base voxel; keep looking for clearance.
          column_height = column_clearance_height = height;
          column_voxel_pos = sub_voxel_pos;
          // Current voxel becomes our new ground candidate voxel.
          ground_key = key;
        }
        else
        {
          // Branch condition only for the first voxel in column.
          ground_key = key;
          column_height = column_clearance_height = height;
          column_voxel_pos = sub_voxel_pos;
          have_candidate = true;
        }
      }

      if (voxel_type != ohm::kUncertain)
      {
        have_transitioned_from_unknown = true;
      }
    }

    // Did we find a valid candidate?
    if (have_candidate)
    {
      *height_out = height;
      *clearance_out = column_clearance_height - column_height;
      return ground_key;
    }

    return Key::kNull;
  }
}  // namespace

Heightmap::Heightmap()
  : Heightmap(0.2, 2.0, UpAxis::kZ)
{}


Heightmap::Heightmap(double grid_resolution, double min_clearance, UpAxis up_axis, unsigned region_size)
  : imp_(new HeightmapDetail)
{
  region_size = region_size ? region_size : kDefaultRegionSize;

  imp_->min_clearance = min_clearance;

  if (up_axis < UpAxis::kNegZ || up_axis > UpAxis::kZ)
  {
    std::cerr << "Unknown up axis ID: " << int(up_axis) << std::endl;
    up_axis = UpAxis::kZ;
  }

  // Cache the up axis normal.
  imp_->up_axis_id = up_axis;
  imp_->updateAxis();

  // Use an OccupancyMap to store grid cells. Each region is 1 voxel thick.
  glm::u8vec3 region_dim(region_size);
  region_dim[int(imp_->vertical_axis_index)] = 1;
  imp_->heightmap = std::make_unique<OccupancyMap>(grid_resolution, region_dim, ohm::MapFlag::kSubVoxelPosition);

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
  imp_->heightmap_layer = static_cast<int>(layer->layerIndex());
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);

  layer = layout.addLayer(HeightmapVoxel::kHeightmapBuildLayer, 0);
  imp_->heightmap_build_layer = static_cast<int>(layer->layerIndex());
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);

  updateMapInfo(imp_->heightmap->mapInfo());
}


Heightmap::~Heightmap() = default;


bool Heightmap::setThreadCount(unsigned thread_count)
{
#ifdef OHM_THREADS
  imp_->thread_count = thread_count;
  return true;
#else   // OHM_THREADS
  (void)thread_count;  // Unused.
  return false;
#endif  // OHM_THREADS
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


void Heightmap::setGenerateFloorFromUnknown(bool enable)
{
  imp_->generate_floor_from_unknown = enable;
}


bool Heightmap::generateFloorFromUnknown() const
{
  return imp_->generate_floor_from_unknown;
}


UpAxis Heightmap::upAxis() const
{
  return UpAxis(imp_->up_axis_id);
}


int Heightmap::upAxisIndex() const
{
  return int(imp_->vertical_axis_index);
}


const glm::dvec3 &Heightmap::upAxisNormal() const
{
  return imp_->up;
}


int Heightmap::surfaceAxisIndexA() const
{
  return HeightmapDetail::surfaceIndexA(imp_->up_axis_id);
}


const glm::dvec3 &Heightmap::surfaceAxisA() const
{
  return HeightmapDetail::surfaceNormalA(imp_->up_axis_id);
}


int Heightmap::surfaceAxisIndexB() const
{
  return HeightmapDetail::surfaceIndexB(imp_->up_axis_id);
}


const glm::dvec3 &Heightmap::surfaceAxisB() const
{
  return HeightmapDetail::surfaceNormalB(imp_->up_axis_id);
}


const glm::dvec3 &Heightmap::upAxisNormal(UpAxis axis_id)
{
  return HeightmapDetail::upAxisNormal(axis_id);
}


const glm::dvec3 &Heightmap::surfaceAxisA(UpAxis axis_id)
{
  return HeightmapDetail::surfaceNormalA(axis_id);
}


const glm::dvec3 &Heightmap::surfaceAxisB(UpAxis axis_id)
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

  glm::dvec3 min_ext(0.0), max_ext(0.0);
  src_map.calculateExtents(&min_ext, &max_ext);

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
  std::atomic_uint populated_count(0);
  if (imp_->thread_count != 1)
  {
    const auto update_heightmap_block = [&](const tbb::blocked_range3d<unsigned, unsigned, unsigned> &range)  //
    {                                                                                                         //
      Key min_key_local = min_ext_key;
      Key max_key_local = min_ext_key;

      // Move to the target offset.
      imp_->heightmap->moveKey(min_key_local, range.cols().begin(), range.rows().begin(), range.pages().begin());
      imp_->heightmap->moveKey(max_key_local, range.cols().end() - 1, range.rows().end() - 1, range.pages().end() - 1);
      populated_count += updateHeightmapForRegion(imp_.get(), base_height, min_key_local, max_key_local, upAxis(),
                                                  Aabb(min_ext, max_ext));
    };

    const glm::ivec3 voxel_range = heightmap.rangeBetween(min_ext_key, max_ext_key);

    const auto parallel_loop = [&]  //
    {
      tbb::parallel_for(tbb::blocked_range3d<unsigned, unsigned, unsigned>(0, voxel_range.z + 1,  //
                                                                           0, voxel_range.y + 1,  //
                                                                           0, voxel_range.x + 1),
                        update_heightmap_block);
      // const unsigned grain_size = 8;
      // tbb::parallel_for(tbb::blocked_range3d<unsigned, unsigned, unsigned>(0, voxel_range.z + 1, grain_size,  //
      //                                                                      0, voxel_range.y + 1, grain_size,  //
      //                                                                      0, voxel_range.x + 1, grain_size),
      //                   update_heightmap_block);
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
#else   // OHM_THREADS
  unsigned populated_count = 0;
#endif  // OHM_THREADS
  {
    populated_count +=
      updateHeightmapForRegion(imp_.get(), base_height, min_ext_key, max_ext_key, upAxis(), Aabb(min_ext, max_ext));
  }
  PROFILE_END(walk)

  return populated_count != 0;
}


bool Heightmap::update(const glm::dvec3 &reference_pos, const ohm::Aabb &cull_to, const ohm::Aabb &exclude)
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

  heightmap.setOrigin(upAxisNormal() * reference_pos[upAxisIndex()]);

  // 1. Calculate the map extents.
  //  a. Calculate occupancy map extents.
  //  b. Project occupancy map extents onto heightmap plane.
  // 2. Populate heightmap voxels

  ohm::Aabb src_region;
  src_map.calculateExtents(&src_region.minExtentsMutable(), &src_region.maxExtentsMutable());

  // Clip to the cull box.
  for (int i = 0; i < 3; ++i)
  {
    if (cull_to.diagonal()[i] > 0)
    {
      src_region.minExtentsMutable()[i] = std::min(cull_to.minExtents()[i], src_region.minExtentsMutable()[i]);
      src_region.maxExtentsMutable()[i] = std::max(cull_to.maxExtents()[i], src_region.maxExtentsMutable()[i]);
    }
  }

  // Generate keys for these extents.
  const Key min_ext_key = src_map.voxelKey(src_region.minExtents());
  const Key max_ext_key = src_map.voxelKey(src_region.maxExtents());

  PROFILE(walk)

  const int heightmap_build_layer = imp_->heightmap_layer;

  // Set the initial key.
  Key walk_key = src_map.voxelKey(reference_pos);

  // Bound the walk_key to the search bounds.
  if (!walk_key.isBounded(min_ext_key, max_ext_key))
  {
    walk_key.clampToAxis(surfaceAxisIndexA(), min_ext_key, max_ext_key);
    walk_key.clampToAxis(surfaceAxisIndexB(), min_ext_key, max_ext_key);
  }

  PlaneFillWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id, false);

  if (!walker.begin(walk_key))
  {
    return false;
  }

  // Walk the 2D extraction region in a spiral around walk_key.
  MapCache src_map_cache, heightmap_cache;
  unsigned populated_count = 0;
  const int voxel_ceiling = ohm::pointToRegionCoord(imp_->ceiling, src_map.resolution());
  do
  {
    // Find the nearest voxel to the current key which may be a ground candidate.
    Key candidate_key = findNearestSupportingVoxel(src_map, walk_key, upAxisIndex(), min_ext_key, max_ext_key,
                                                   voxel_ceiling, imp_->generate_floor_from_unknown);

    if (candidate_key.isNull())
    {
      // Candidate is invalid. Add walk_key neighbours for walking.
      walker.addNeighbours(walk_key, PlaneFillWalker::Revisit::None);
      continue;
    }

    // Walk up from the candidate to find the best heightmap voxel.
    double height = 0;
    double clearance = 0;
    const Key ground_key =
      findGround(&height, &clearance, src_map, candidate_key, min_ext_key, max_ext_key, &src_map_cache, *imp_);

    // Write into the heightmap.
    if (!ground_key.isNull())
    {
      // We have a ground key. Add it's neighbours (adjusted height) for walking.
      const PlaneFillWalker::Revisit revisit_behaviour =
        int(imp_->up_axis_id) >= 0 ? PlaneFillWalker::Revisit::Lower : PlaneFillWalker::Revisit::Higher;
      walker.addNeighbours(ground_key, revisit_behaviour);

      // Write to the heightmap.
      VoxelConst src_voxel = src_map.voxel(ground_key);
      const ohm::OccupancyType voxel_type = ohm::OccupancyType(src_map.occupancyType(src_voxel));

      bool is_floor_voxel = false;
      bool is_excluded = false;
      glm::dvec3 voxel_pos = src_voxel.position();
      if (voxel_type == ohm::kOccupied || voxel_type == ohm::kFree && imp_->generate_floor_from_unknown)
      {

        if (voxel_type == ohm::kFree && imp_->generate_floor_from_unknown && exclude.contains(voxel_pos))
        {
          is_floor_voxel = false;
          is_excluded = true;

          // // The nominated surface is actually the interface between free and unknown space. Use historic data from the
          // // surface cache if available.
          // if (surface_cache)
          // {
          //   const int up_index = upAxisIndex();
          //   glm::dvec3 coord = src_voxel.centreGlobal();
          //   ohm::Key cache_key = surface_cache->voxelKey(coord);
          //   // Clamp the vertical aspect of the key.
          //   cache_key.setRegionAxis(up_index, 0);
          //   cache_key.setLocalAxis(up_index, 0);
          //   const VoxelConst cache_voxel = surface_cache->voxel(cache_key);
          //   if (cache_voxel.isValid() && cache_voxel.isOccupied())
          //   {
          //     voxel_pos = cache_voxel.position();

          //     const ohm::MapLayer *heightmap_layer = surface_cache->layout().layer(ohm::HeightmapVoxel::kHeightmapLayer);
          //     const int heightmap_layer_index = heightmap_layer->layerIndex();

          //     // Read the height then clear it to reset for next update.
          //     const ohm::HeightmapVoxel *cache_height_info =
          //       cache_voxel.layerContent<const ohm::HeightmapVoxel *>(heightmap_layer_index);
          //     voxel_pos[up_index] += cache_height_info->height;
          //   }
          // }
        }
        else
        {
          is_floor_voxel = true;
          // Cache the height then clear from the position.
          const double src_height = voxel_pos[upAxisIndex()];
          voxel_pos[upAxisIndex()] = 0;

          // Get the heightmap voxel to update.
          Key hm_key = heightmap.voxelKey(voxel_pos);
          project(&hm_key);
          Voxel hm_voxel = heightmap.voxel(hm_key, true, &heightmap_cache);
          hm_voxel.setValue(heightmap.occupancyThresholdValue());
          // Set sub-voxel position as required.
          if (sub_voxel_allowed)
          {
            hm_voxel.setPosition(voxel_pos);
          }

          // Write the height and clearance values.
          HeightmapVoxel *voxel_content = hm_voxel.layerContent<HeightmapVoxel *>(heightmap_build_layer);
          if (voxel_content)
          {
            voxel_content->height = relativeVoxelHeight(src_height, hm_voxel, imp_->up);
            voxel_content->clearance = float(clearance);
          }

          ++populated_count;
        }
      }

      if (!is_floor_voxel)
      {
        // No ground found. Add walk_key neighbours for walking.
        walker.addNeighbours(walk_key, PlaneFillWalker::Revisit::None);

        if (!is_excluded)
        {
          // If the voxel is within the negative obstacle range then we create a fake surface at a low height to create
          // a steep and high cost transition.
          if (glm::dot(voxel_pos - reference_pos, voxel_pos - reference_pos) <=
              imp_->negative_obstacle_radius * imp_->negative_obstacle_radius)
          {
            Key hm_key = heightmap.voxelKey(voxel_pos);
            project(&hm_key);
            Voxel hm_voxel = heightmap.voxel(hm_key, true, &heightmap_cache);
            hm_voxel.setValue(heightmap.occupancyThresholdValue());
            // Set sub-voxel position as required.
            if (sub_voxel_allowed)
            {
              hm_voxel.setPosition(voxel_pos);
            }

            // Write the height and clearance values.
            HeightmapVoxel *voxel_content = hm_voxel.layerContent<HeightmapVoxel *>(heightmap_build_layer);
            if (voxel_content)
            {
              voxel_content->height = reference_pos[upAxisIndex()] - 100.0;
              voxel_content->clearance = 0.01;
            }
          }
        }
      }
    }
  } while (walker.walkNext(walk_key));

  return populated_count != 0;
}


void Heightmap::updateMapInfo(MapInfo &info) const
{
  imp_->toMapInfo(info);
}


Key &Heightmap::project(Key *key)
{
  key->setRegionAxis(upAxisIndex(), 0);
  key->setLocalAxis(upAxisIndex(), 0);
  return *key;
}
