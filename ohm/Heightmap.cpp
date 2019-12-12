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
          imp.generate_virtual_floor && !have_transitioned_from_unknown && voxel_type == ohm::kFree)
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


void Heightmap::setGenerateVirtualFloor(bool enable)
{
  imp_->generate_virtual_floor = enable;
}


bool Heightmap::generateVirtualFloor() const
{
  return imp_->generate_virtual_floor;
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


bool Heightmap::update(const glm::dvec3 &reference_pos, const ohm::Aabb &cull_to)
{
  if (!imp_->occupancy_map)
  {
    return false;
  }

  // static bool allow_update = true;
  // static unsigned update_number = 0;

  // if (!allow_update || update_number == 0)
  // {
  //   return false;
  // }

  // ++update_number;

  // Brute force initial approach.
  const OccupancyMap &src_map = *imp_->occupancy_map;
  OccupancyMap &heightmap = *imp_->heightmap;

  updateMapInfo(heightmap.mapInfo());

  // Clear previous results.
  heightmap.clear();

  // Allow sub-voxel positioning.
  const bool sub_voxel_allowed = !imp_->ignore_sub_voxel_positioning;
  heightmap.setSubVoxelsEnabled(src_map.subVoxelsEnabled() && sub_voxel_allowed);

  heightmap.setOrigin(upAxisNormal() * glm::dot(upAxisNormal(), reference_pos));

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
                                                   voxel_ceiling, imp_->generate_virtual_floor);

    // Walk up from the candidate to find the best heightmap voxel.
    double height = 0;
    double clearance = 0;
    const Key ground_key = (!candidate_key.isNull()) ? findGround(&height, &clearance, src_map, candidate_key,
                                                                  min_ext_key, max_ext_key, &src_map_cache, *imp_) :
                                                       walk_key;

    // Add neighbours for walking.
    PlaneFillWalker::Revisit revisit_behaviour = PlaneFillWalker::Revisit::None;
    if (!candidate_key.isNull())
    {
      revisit_behaviour =
        int(imp_->up_axis_id) >= 0 ? PlaneFillWalker::Revisit::Lower : PlaneFillWalker::Revisit::Higher;
    }
    walker.addNeighbours(ground_key, revisit_behaviour);

    // Write to the heightmap.
    VoxelConst src_voxel = src_map.voxel(ground_key);
    const ohm::OccupancyType voxel_type = ohm::OccupancyType(src_map.occupancyType(src_voxel));

    // May be a void in the map.
    glm::dvec3 voxel_pos = (src_voxel.isValid()) ? src_voxel.position() : src_map.voxelCentreGlobal(ground_key);

    if (voxel_type == ohm::kOccupied || imp_->generate_virtual_floor)
    {
      // Real or virtual surface.
      float surface_value = (voxel_type == kOccupied) ? kHeightmapSurfaceValue : kHeightmapVirtualSurfaceValue;
      bool should_populate = voxel_type != kUncertain && voxel_type != kNull;

      if (voxel_type != kOccupied)
      {
        // For virtual surface cells, first lookup the local heightmap cache. We prefer the cache if it has a real
        // surface stored or it has a kHeightmapVacantValue value.
        // We do not replace a virtual surface with a virtual surface though.
        float local_cache_value = 0;
        glm::dvec3 local_cache_pos(0.0);
        if (lookupLocalCache(voxel_pos, &local_cache_pos, &local_cache_value, &clearance))
        {
          if (local_cache_value >= 0)
          {
            surface_value = local_cache_value;
            voxel_pos = local_cache_pos;
            should_populate = true;
          }
        }
      }

      if (should_populate)
      {
        // Cache the height then clear from the position.
        const double src_height = voxel_pos[upAxisIndex()];
        voxel_pos[upAxisIndex()] = 0;

        // Get the heightmap voxel to update.
        Key hm_key = heightmap.voxelKey(voxel_pos);
        project(&hm_key);
        Voxel hm_voxel = heightmap.voxel(hm_key, true, &heightmap_cache);
        hm_voxel.setValue(surface_value);
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
  } while (walker.walkNext(walk_key));

  // Update the local cache.
  updateLocalCache(reference_pos);

  return populated_count != 0;
}


bool Heightmap::heightmapVoxelPosition(const VoxelConst &heightmap_voxel, glm::dvec3 *pos, float *clearance) const
{
  if (heightmap_voxel.isNull())
  {
    return false;
  }

  *pos = heightmap_voxel.position();
  const bool is_uncertain = heightmap_voxel.isUncertainOrNull();
  if (!is_uncertain)
  {
    // Get height info.
    const HeightmapVoxel *heightmap_info = heightmap_voxel.layerContent<const HeightmapVoxel *>(imp_->heightmap_layer);
    (*pos)[upAxisIndex()] = heightmap_voxel.centreGlobal()[upAxisIndex()] + heightmap_info->height;
    if (clearance)
    {
      *clearance = heightmap_info->clearance;
    }

    if (heightmap_voxel.value() == 0)
    {
      // Vacant
      return false;
    }
  }

  // Check for a virtual surface or unobserved voxel within the fatal obstacle range.
  if (imp_->negative_obstacle_radius > 0 && (is_uncertain || heightmap_voxel.value() < 0))
  {
    const glm::dvec3 reference_pos = imp_->heightmap->origin();
    glm::dvec3 from_reference_pos = *pos - reference_pos;
    from_reference_pos[upAxisIndex()] = 0;
    if (glm::dot(from_reference_pos, from_reference_pos) < imp_->negative_obstacle_radius * imp_->negative_obstacle_radius)
    {
      // Within fatal negative obstacle range. Give it a dramatic position variance from the reference pos.
      (*pos)[upAxisIndex()] = reference_pos[upAxisIndex()] - 2.0;
      if (clearance)
      {
        *clearance = -1.0f;
      }
      return true;
    }
  }

  return !is_uncertain;
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


void Heightmap::seedLocalCache(const glm::dvec3 &reference_pos)
{
  if (!imp_->heightmap_local_cache)
  {
    imp_->heightmap_local_cache =
      std::make_unique<OccupancyMap>(imp_->heightmap->resolution(), imp_->heightmap->regionVoxelDimensions(),
                                     ohm::MapFlag::kSubVoxelPosition, imp_->heightmap->layout());
  }

  // local_surface_map has the same MapLayout as the heightmap.
  OccupancyMap &local_cache = *imp_->heightmap_local_cache;
  MapCache voxel_lookup_cache;
  const ohm::MapLayer *heightmap_layer = local_cache.layout().layer(ohm::HeightmapVoxel::kHeightmapLayer);
  const int heightmap_layer_index = heightmap_layer->layerIndex();
  const int row_axis = surfaceAxisIndexA();
  const int col_axis = surfaceAxisIndexB();

  local_cache.setOrigin(reference_pos);

  const glm::dvec3 min_range = reference_pos + imp_->local_cache_bounds.minExtents();
  const glm::dvec3 max_range = reference_pos + imp_->local_cache_bounds.maxExtents();

  ohm::Key min_key = local_cache.voxelKey(min_range);
  ohm::Key max_key = local_cache.voxelKey(max_range);

  // Flatten on to a plane.
  project(&min_key);
  project(&max_key);

  const glm::ivec3 key_range = local_cache.rangeBetween(min_key, max_key);

  // Initialise the local surface as a flat region around 0, 0, 0.
  ohm::Key key;
  for (int row = 0; row < key_range[row_axis]; ++row)
  {
    key = min_key;
    local_cache.moveKeyAlongAxis(key, row_axis, row);
    for (int col = 0; col < key_range[col_axis]; ++col)
    {
      local_cache.moveKeyAlongAxis(key, col_axis, 1);
      // Add the seed surface.
      ohm::Voxel voxel = local_cache.voxel(key, true, &voxel_lookup_cache);
      voxel.setValue(0.0f);

      ohm::HeightmapVoxel *height_info = voxel.layerContent<ohm::HeightmapVoxel *>(heightmap_layer_index);
      height_info->height = 0.0f;
      height_info->clearance = 0.0f;
    }
  }
}


void Heightmap::updateLocalCache(const glm::dvec3 &reference_pos)
{
  if (!imp_->heightmap_local_cache)
  {
    return;
  }

  // In the cache:
  // - occupied => have cached value
  // - free => unseen
  // - null/uncertain => unknown

  // From the heightmap into the surface cache:
  // - occupied => real ground
  // - free => fake ground
  // - null,uncertain => unseen

  OccupancyMap &heightmap = *imp_->heightmap;
  OccupancyMap &local_cache_map = *imp_->heightmap_local_cache;
  const MapLayer *heightmap_layer = heightmap.layout().layer(HeightmapVoxel::kHeightmapLayer);
  const int heightmap_layer_index = heightmap_layer->layerIndex();
  const int up_index = upAxisIndex();
  const int row_axis = surfaceAxisIndexA();
  const int col_axis = surfaceAxisIndexB();

  MapCache heightmap_lookup_cache;
  MapCache surface_lookup_cache;

  // Up the local surface from the heightmap.
  local_cache_map.clear();
  // Move the local surface cache to match the heightmap. This really only adjusts the height.
  local_cache_map.setOrigin(heightmap.origin());

  // Update the search extents.
  const glm::dvec3 min_range = reference_pos + imp_->local_cache_bounds.minExtents();
  const glm::dvec3 max_range = reference_pos + imp_->local_cache_bounds.maxExtents();

  // Flatten the key range to the heightmap surface.
  Key min_key = local_cache_map.voxelKey(min_range);
  Key max_key = local_cache_map.voxelKey(max_range);
  project(&min_key);
  project(&max_key);

  const glm::ivec3 key_range = local_cache_map.rangeBetween(min_key, max_key);

  // Read back into the surface cache.
  Key key;
  for (int row = 0; row < key_range[row_axis]; ++row)
  {
    key = min_key;
    local_cache_map.moveKeyAlongAxis(key, row_axis, row);
    for (int col = 0; col < key_range[col_axis]; ++col)
    {
      local_cache_map.moveKeyAlongAxis(key, col_axis, 1);

      // Convert to a voxel centre to have a coordinate to lookup in the heightmap.
      glm::dvec3 coord = local_cache_map.voxelCentreGlobal(key);

      // Resolve the key in the heightmap.
      Key hm_key = heightmap.voxelKey(coord);
      // Ensure the hm_key up_axis indexing is all zero so we only work on a single layer (working in 2D)
      hm_key.setRegionAxis(up_index, 0);
      hm_key.setLocalAxis(up_index, 0);
      const VoxelConst hm_voxel = heightmap.voxel(hm_key, false, &heightmap_lookup_cache);

      if (!hm_voxel.isUncertainOrNull())
      {
        Voxel cache_voxel = local_cache_map.voxel(key, true, &surface_lookup_cache);
        const float value = hm_voxel.value();
        cache_voxel.setValue(value);

        const HeightmapVoxel *src_height_info = hm_voxel.layerContent<const HeightmapVoxel *>(heightmap_layer_index);

        HeightmapVoxel *dst_height_info = cache_voxel.layerContent<HeightmapVoxel *>(heightmap_layer_index);
        *dst_height_info = *src_height_info;
      }
    }
  }
}


bool Heightmap::lookupLocalCache(const glm::dvec3 &lookup_pos, glm::dvec3 *cache_pos, float *cache_value,
                                 double *clearance)
{
  if (!imp_->heightmap_local_cache)
  {
    return false;
  }

  Key key = imp_->heightmap_local_cache->voxelKey(lookup_pos);
  if (key.isNull())
  {
    return false;
  }

  project(&key);
  const VoxelConst cache_voxel = imp_->heightmap_local_cache->voxel(key);
  if (cache_voxel.isUncertainOrNull())
  {
    return false;
  }

  *cache_pos = cache_voxel.position();
  const MapLayer *heightmap_layer = imp_->heightmap_local_cache->layout().layer(HeightmapVoxel::kHeightmapLayer);
  const int heightmap_layer_index = heightmap_layer->layerIndex();
  const HeightmapVoxel *height_info = cache_voxel.layerContent<const HeightmapVoxel *>(heightmap_layer_index);
  *cache_value = cache_voxel.value();
  (*cache_pos)[upAxisIndex()] = cache_voxel.centreGlobal()[upAxisIndex()] * double(height_info->height);
  *clearance = height_info->clearance;
  return true;
}
