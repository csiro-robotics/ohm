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


  Key findNearestSupportingVoxel2(const OccupancyMap &map, const Key &from_key, const Key &to_key, int up_axis_index,
                                  int step_limit, bool search_up, bool floor_from_unknown, int *offset,
                                  bool *is_virtual)
  {
    *is_virtual = true;

    int vertical_range = map.rangeBetween(from_key, to_key)[up_axis_index] + 1;
    if (step_limit > 0)
    {
      vertical_range = std::min(vertical_range, step_limit);
    }
    const int step = (vertical_range >= 0) ? 1 : -1;
    vertical_range = (vertical_range >= 0) ? vertical_range : -vertical_range;

    MapCache map_cache;
    Key best_virtual(nullptr);
    int last_voxel_type = kNull;

    Key last_key(nullptr);
    Key current_key = from_key;
    for (int i = 0; i < vertical_range; ++i)
    {
      // We bias the offset to prefer searching down by one voxel.
      *offset = i + !!search_up;
      const VoxelConst voxel = map.voxel(current_key, &map_cache);

      const int voxel_type = map.occupancyType(voxel);
      switch (voxel_type)
      {
      case kOccupied:
        // First occupied. Terminate.
        *is_virtual = false;
        return current_key;
      case kFree:
        // When searching up, we have a virtual surface when we transition from uncertain to free.
        if (floor_from_unknown && (last_voxel_type == kNull || last_voxel_type == kUncertain) && search_up &&
            best_virtual.isNull())
        {
          best_virtual = current_key;
        }
        break;

      default:
        // Transition to uncertain. When searchign down, we have a virtual surface if we transitioned from free.
        if (floor_from_unknown && !search_up && last_voxel_type == kFree)
        {
          best_virtual = last_key;
        }

        if (voxel_type == kNull)
        {
          // Null region. Skip the entire region on the next step.
          Key next_key = current_key;
          if (step > 0)
          {
            next_key.setLocalAxis(up_axis_index, map.regionVoxelDimensions()[up_axis_index] - 1);
            const int delta = next_key.localKey()[up_axis_index] - current_key.localKey()[up_axis_index];
            i += delta;
            current_key = next_key;
          }
          else
          {
            next_key.setLocalAxis(up_axis_index, 0);
            const int delta = current_key.localKey()[up_axis_index] - next_key.localKey()[up_axis_index];
            i += delta;
            current_key = next_key;
          }
        }
        break;
      }

      last_voxel_type = voxel_type;
      last_key = current_key;

      map.moveKeyAlongAxis(current_key, up_axis_index, step);
    }

    if (best_virtual.isNull())
    {
      if (!search_up && last_voxel_type == kFree)
      {
        best_virtual = last_key;
      }
      else
      {
        *offset = -1;
      }
    }

    // We only get here if we haven't found an occupied voxel. Return the best virtual one.
    return best_virtual;
  }

  inline Key findNearestSupportingVoxel(const OccupancyMap &map, const Key &seed_key, UpAxis up_axis,
                                        const Key &min_key, const Key &max_key, int voxel_ceiling,
                                        bool floor_from_unknown)
  {
    PROFILE(findNearestSupportingVoxel);
    Key above, below;
    int offset_below = -1;
    int offset_above = -1;
    bool virtual_below = false;
    bool virtual_above = false;

    const int up_axis_index = (int(up_axis) >= 0) ? int(up_axis) : -int(up_axis) - 1;
    const Key &search_down_to = (int(up_axis) >= 0) ? min_key : max_key;
    const Key &search_up_to = (int(up_axis) >= 0) ? max_key : min_key;
    below = findNearestSupportingVoxel2(map, seed_key, search_down_to, up_axis_index, 0, false, floor_from_unknown,
                                        &offset_below, &virtual_below);
    above = findNearestSupportingVoxel2(map, seed_key, search_up_to, up_axis_index, voxel_ceiling, true,
                                        floor_from_unknown, &offset_above, &virtual_above);

    // Prefer non-virtual over virtual. Prefer the closer result.
    if (virtual_above && !virtual_below)
    {
      return below;
    }

    if (!virtual_above && virtual_below)
    {
      return above;
    }

    // We never allow virtual voxels above as this generates better heightmaps. Virtual surfaces are more interesting
    // when approaching a slope down than any such information above.
    if (virtual_above && virtual_below)
    {
      return below;
    }

    if (offset_below >= 0 && (offset_above < 0 || offset_below <= offset_above))
    {
      return below;
    }

    return above;
  }

  Key findGround(double *height_out, double *clearance_out, const OccupancyMap &map, const Key &seed_key,
                 const Key &min_key, const Key &max_key, MapCache *cache, const HeightmapDetail &imp)
  {
    PROFILE(findGround);
    // Start with the seed_key and look for ground. We only walk up from the seed key.
    double column_height = std::numeric_limits<double>::max();
    double column_clearance_height = column_height;

    // Start walking the voxels in the source map.
    // glm::dvec3 column_reference = heightmap.voxelCentreGlobal(target_key);

    // Walk the src column up.
    const int up_axis_index = imp.vertical_axis_index;
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
          imp.generate_virtual_surface && !have_transitioned_from_unknown && voxel_type == ohm::kFree)
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


  void onVisitPlaneFill(PlaneFillWalker &walker, const HeightmapDetail &imp, const Key &candidate_key,
                        const Key &ground_key)
  {
    // Add neighbours for walking.
    PlaneFillWalker::Revisit revisit_behaviour = PlaneFillWalker::Revisit::None;
    if (!candidate_key.isNull())
    {
      revisit_behaviour = int(imp.up_axis_id) >= 0 ? PlaneFillWalker::Revisit::Lower : PlaneFillWalker::Revisit::Higher;
    }
    walker.addNeighbours(ground_key, revisit_behaviour);
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

  updateMapInfo(imp_->heightmap->mapInfo());
}


Heightmap::~Heightmap() = default;


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


void Heightmap::setGenerateVirtualSurface(bool enable)
{
  imp_->generate_virtual_surface = enable;
}


bool Heightmap::generateVirtualSurface() const
{
  return imp_->generate_virtual_surface;
}


void Heightmap::setUseFloodFill(bool flood_fill)
{
  imp_->use_flood_fill = flood_fill;
}


bool Heightmap::useFloodFill() const
{
  return imp_->use_flood_fill;
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


bool Heightmap::buildHeightmap(const glm::dvec3 &reference_pos, const ohm::Aabb &cull_to)
{
  if (!imp_->occupancy_map)
  {
    return false;
  }

  PROFILE(buildHeightmap);

  // 1. Calculate the map extents.
  //  a. Calculate occupancy map extents.
  //  b. Project occupancy map extents onto heightmap plane.
  // 2. Populate heightmap voxels

  const OccupancyMap &src_map = *imp_->occupancy_map;
  ohm::Aabb src_region;
  src_map.calculateExtents(&src_region.minExtentsMutable(), &src_region.maxExtentsMutable());

  // Clip to the cull box.
  for (int i = 0; i < 3; ++i)
  {
    if (cull_to.diagonal()[i] > 0)
    {
      src_region.minExtentsMutable()[i] = cull_to.minExtents()[i];
      src_region.maxExtentsMutable()[i] = cull_to.maxExtents()[i];
    }
  }

  // Generate keys for these extents.
  const Key min_ext_key = src_map.voxelKey(src_region.minExtents());
  const Key max_ext_key = src_map.voxelKey(src_region.maxExtents());

  unsigned processedCount = 0;
  if (!imp_->use_flood_fill)
  {
    const Key planar_key = src_map.voxelKey(reference_pos);
    PlaneWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id, &planar_key);
    processedCount = buildHeightmapT(walker, reference_pos);
  }
  else
  {
    PlaneFillWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id, false);
    processedCount = buildHeightmapT(walker, reference_pos, &onVisitPlaneFill);
  }

#if PROFILING
  ohm::Profile::instance().report();
#endif  // PROFILING

  return processedCount;
}


HeightmapVoxelType Heightmap::getHeightmapVoxelInfo(const VoxelConst &heightmap_voxel, glm::dvec3 *pos,
                                                    float *clearance) const
{
  if (heightmap_voxel.isNull())
  {
    return HeightmapVoxelType::kUnknown;
  }

  *pos = heightmap_voxel.position();
  const bool is_uncertain = heightmap_voxel.isUncertainOrNull();
  const float heightmap_voxel_value = (!is_uncertain) ? heightmap_voxel.value() : -1.0f;
  if (!is_uncertain)
  {
    // Get height info.
    const HeightmapVoxel *heightmap_info = heightmap_voxel.layerContent<const HeightmapVoxel *>(imp_->heightmap_layer);
    (*pos)[upAxisIndex()] = heightmap_voxel.centreGlobal()[upAxisIndex()] + heightmap_info->height;
    if (clearance)
    {
      *clearance = heightmap_info->clearance;
    }

    if (heightmap_voxel_value == 0)
    {
      // kVacant
      return HeightmapVoxelType::kVacant;
    }

    if (heightmap_voxel_value > 0)
    {
      return HeightmapVoxelType::kSurface;
    }
  }

  return (!is_uncertain) ? HeightmapVoxelType::kVirtualSurface : HeightmapVoxelType::kUnknown;
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


template <typename KeyWalker>
bool Heightmap::buildHeightmapT(KeyWalker &walker, const glm::dvec3 &reference_pos,
                                void (*on_visit)(KeyWalker &, const HeightmapDetail &, const Key &, const Key &))
{
  // Brute force initial approach.
  const OccupancyMap &src_map = *imp_->occupancy_map;
  OccupancyMap &heightmap = *imp_->heightmap;

  updateMapInfo(heightmap.mapInfo());

  // Clear previous results.
  heightmap.clear();

  // Encode the base height of the heightmap in the origin.
  // heightmap.setOrigin(upAxisNormal() * glm::dot(upAxisNormal(), reference_pos));

  // Allow sub-voxel positioning.
  const bool sub_voxel_allowed = !imp_->ignore_sub_voxel_positioning;
  heightmap.setSubVoxelsEnabled(src_map.subVoxelsEnabled() && sub_voxel_allowed);

  PROFILE(walk)

  const int heightmap_layer = imp_->heightmap_layer;

  // Set the initial key.
  Key walk_key = src_map.voxelKey(reference_pos);

  // Bound the walk_key to the search bounds.
  if (!walk_key.isBounded(walker.min_ext_key, walker.max_ext_key))
  {
    walk_key.clampToAxis(surfaceAxisIndexA(), walker.min_ext_key, walker.max_ext_key);
    walk_key.clampToAxis(surfaceAxisIndexB(), walker.min_ext_key, walker.max_ext_key);
  }

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
    // This is key closest to the walk_key which could be ground. This will be either an occupied voxel, or virtual ground
    /// voxel. Virtual ground is where a free is supported by an uncertain or null voxel below it.
    Key candidate_key = findNearestSupportingVoxel(src_map, walk_key, upAxis(), walker.min_ext_key, walker.max_ext_key,
                                                   voxel_ceiling, imp_->generate_virtual_surface);

    // Walk up from the candidate to find the best heightmap voxel.
    double height = 0;
    double clearance = 0;
    // Walk the column of candidate_key to find the first occupied voxel with sufficent clearance. A virtual voxel
    // with sufficient clearance may be given if there is no valid occupied voxel.
    const Key ground_key = (!candidate_key.isNull()) ?
                             findGround(&height, &clearance, src_map, candidate_key, walker.min_ext_key,
                                        walker.max_ext_key, &src_map_cache, *imp_) :
                             walk_key;

    if (on_visit)
    {
      (*on_visit)(walker, *imp_, candidate_key, ground_key);
    }

    // Write to the heightmap.
    VoxelConst src_voxel = src_map.voxel(ground_key);
    const ohm::OccupancyType voxel_type = ohm::OccupancyType(src_map.occupancyType(src_voxel));

    // We use the voxel centre for lookup in the local cache for better consistency. Otherwise lateral drift in
    // subvoxel positioning can result in looking up the wrong cell.
    glm::dvec3 src_voxel_centre =
      (src_voxel.isValid()) ? src_voxel.centreGlobal() : src_map.voxelCentreGlobal(ground_key);
    glm::dvec3 voxel_pos = (src_voxel.isValid()) ? src_voxel.position() : src_voxel_centre;

    if (voxel_type == ohm::kOccupied || imp_->generate_virtual_surface)
    {
      // Real or virtual surface.
      float surface_value = (voxel_type == kOccupied) ? kHeightmapSurfaceValue : kHeightmapVirtualSurfaceValue;

      if (voxel_type != kUncertain && voxel_type != kNull)
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
        HeightmapVoxel *voxel_content = hm_voxel.layerContent<HeightmapVoxel *>(heightmap_layer);
        if (voxel_content)
        {
          voxel_content->height = relativeVoxelHeight(src_height, hm_voxel, imp_->up);
          voxel_content->clearance = float(clearance);
        }

        ++populated_count;
      }
    }
  } while (walker.walkNext(walk_key));

  return populated_count != 0;
}
