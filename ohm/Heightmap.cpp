// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Heightmap.h"

#include "private/HeightmapDetail.h"

#include "Aabb.h"
#include "DefaultLayer.h"
#include "HeightmapVoxel.h"
#include "Key.h"
#include "MapChunk.h"
#include "MapCoord.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "OccupancyType.h"
#include "PlaneFillWalker.h"
#include "PlaneWalker.h"
#include "VoxelData.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "CovarianceVoxel.h"

#include <algorithm>
#include <cstring>
#include <iostream>

#define PROFILING 0
#include <ohmutil/Profile.h>

#include <cassert>

namespace ohm
{
namespace
{
/// Helper structure for managing voxel data access from the source heightmap.
struct SrcVoxel
{
  Voxel<const float> occupancy;             ///< Occupancy value (required)
  Voxel<const VoxelMean> mean;              ///< Voxel mean layer (optional)
  Voxel<const CovarianceVoxel> covariance;  ///< Covariance layer used for surface normal estimation (optional)
  float occupancy_threshold;                ///< Occupancy threshold cached from the source map.

  SrcVoxel(const OccupancyMap &map, bool use_voxel_mean)
    : occupancy(&map, map.layout().occupancyLayer())
    , mean(&map, use_voxel_mean ? map.layout().meanLayer() : -1)
    , covariance(&map, map.layout().covarianceLayer())
    , occupancy_threshold(map.occupancyThresholdValue())
  {}

  /// Set the key, but only for the occupancy layer.
  inline void setKey(const Key &key) { occupancy.setKey(key); }

  /// Sync the key from the occupancy layer to the other layers.
  inline void syncKey()
  {
    // Chain the occupancy values which maximise data caching.
    covariance.setKey(mean.setKey(occupancy));
  }

  /// Query the target map.
  inline const OccupancyMap &map() const { return *occupancy.map(); }

  /// Query the occupancy classification of the current voxel.
  inline OccupancyType occupancyType() const
  {
    float value = unobservedOccupancyValue();
#ifdef __clang_analyzer__
    if (occupancy.voxelMemory())
#else   // __clang_analyzer__
    if (occupancy.isValid())
#endif  // __clang_analyzer__
    {
      occupancy.read(&value);
    }
    OccupancyType type = (value >= occupancy_threshold) ? kOccupied : kFree;
    type = value != unobservedOccupancyValue() ? type : kUnobserved;
    return occupancy.chunk() ? type : kNull;
  }

  /// Query the voxel position. Must call @c syncKey() first if using voxel mean.
  inline glm::dvec3 position() const
  {
    glm::dvec3 pos = occupancy.map()->voxelCentreGlobal(occupancy.key());
#ifdef __clang_analyzer__
    if (mean.voxelMemory())
#else   // __clang_analyzer__
    if (mean.isValid())
#endif  // __clang_analyzer__
    {
      VoxelMean mean_info;
      mean.read(&mean_info);
      pos += subVoxelToLocalCoord<glm::dvec3>(mean_info.coord, occupancy.map()->resolution());
    }
    return pos;
  }

  /// Query the voxel centre for the current voxel.
  inline glm::dvec3 centre() const { return occupancy.map()->voxelCentreGlobal(occupancy.key()); }
};

/// A utility for tracking the voxel being written in the heightmap.
struct DstVoxel
{
  /// Occupancy voxel in the heightmap: writable
  Voxel<float> occupancy;
  /// Heightmap extension data.
  Voxel<HeightmapVoxel> heightmap;
  /// Voxel mean (if being used.)
  Voxel<VoxelMean> mean;

  DstVoxel(OccupancyMap &map, int heightmap_layer, bool use_mean)
    : occupancy(&map, map.layout().occupancyLayer())
    , heightmap(&map, heightmap_layer)
    , mean(&map, use_mean ? map.layout().meanLayer() : -1)
  {}

  inline void setKey(const Key &key) { mean.setKey(heightmap.setKey(occupancy.setKey(key))); }

  /// Get the target (height)map
  inline const OccupancyMap &map() const { return *occupancy.map(); }

  /// Query the position from the heightmap.
  inline glm::dvec3 position() const
  {
    glm::dvec3 pos = occupancy.map()->voxelCentreGlobal(occupancy.key());
    if (mean.isLayerValid())
    {
      VoxelMean mean_info;
      mean.read(&mean_info);
      pos += subVoxelToLocalCoord<glm::dvec3>(mean_info.coord, occupancy.map()->resolution());
    }
    return pos;
  }

  /// Set the position in the heightmap
  inline void setPosition(const glm::dvec3 &pos)
  {
    if (mean.isValid())
    {
      VoxelMean voxel_mean;
      mean.read(&voxel_mean);
      voxel_mean.coord = subVoxelCoord(pos - mean.map()->voxelCentreGlobal(mean.key()), mean.map()->resolution());
      voxel_mean.count = 1;
      mean.write(voxel_mean);
    }
  }

  inline glm::dvec3 centre() const { return occupancy.map()->voxelCentreGlobal(occupancy.key()); }
};

inline float relativeVoxelHeight(double absolute_height, const Key &key, const OccupancyMap &map, const glm::dvec3 &up)
{
  const float relative_height = float(absolute_height - glm::dot(map.voxelCentreGlobal(key), up));
  return relative_height;
}


inline OccupancyType sourceVoxelHeight(glm::dvec3 *voxel_position, double *height, SrcVoxel &voxel,
                                       const glm::dvec3 &up)
{
  OccupancyType voxel_type = voxel.occupancyType();
  if (voxel_type == ohm::kOccupied)
  {
    // Determine the height offset for voxel.
    voxel.syncKey();
    *voxel_position = voxel.position();
  }
  else
  {
    // Return the voxel centre. Voxel may be invalid, so use the map interface on the key.
    *voxel_position = voxel.map().voxelCentreGlobal(voxel.occupancy.key());
  }
  *height = glm::dot(*voxel_position, up);

  return voxel_type;
}


Key findNearestSupportingVoxel2(SrcVoxel &voxel, const Key &from_key, const Key &to_key, int up_axis_index,
                                int step_limit, bool search_up, bool allow_virtual_surface, int *offset,
                                bool *is_virtual)
{
  // Calculate the vertical range we will be searching.
  // Note: the vertical_range sign may not be what you expect. It will match search_up (true === +, false === -)
  // when the up axis is +X, +Y, or +Z. It will not match when the up axis is -X, -Y, or -Z.
  int vertical_range = voxel.map().rangeBetween(from_key, to_key)[up_axis_index] + 1;
  // Step direction is based on the vertical_range sign.
  const int step = (vertical_range >= 0) ? 1 : -1;
  vertical_range = (vertical_range >= 0) ? vertical_range : -vertical_range;
  if (step_limit > 0)
  {
    vertical_range = std::min(vertical_range, step_limit);
  }

  Key best_virtual(nullptr);
  bool last_unknown = true;
  bool last_free = true;

  Key last_key(nullptr);
  Key current_key = from_key;
  for (int i = 0; i < vertical_range; ++i)
  {
    // We bias the offset up one voxel for upward searches. The expectation is that the downward search starts
    // at the seed voxel, while the upward search starts one above that without overlap.
    *offset = i + !!search_up;
    voxel.setKey(current_key);

    // This line yields performance issues likely due to the stochastic memory access.
    // For a true performance gain we'd have to access chunks linearly.
    // Read the occupancy value for the voxel.
    float occupancy = unobservedOccupancyValue();
    if (voxel.occupancy.chunk())
    {
      voxel.occupancy.read(&occupancy);
    }
    // Categorise the voxel.
    const bool occupied = occupancy >= voxel.occupancy_threshold && occupancy != unobservedOccupancyValue();
    const bool free = occupancy < voxel.occupancy_threshold;

    if (occupied)
    {
      // Voxel is occupied. We've found our candidate.
      *is_virtual = false;
      return current_key;
    }

    // No occupied voxel. Update the best (virtual) voxel.
    // We either keep the current best_virtual, or we select the current_voxel as a new best candidate.
    // We split this work into two. The first check is for the upward search where always select the first viable
    // virtual surface voxel and will not overwrite it. The conditions for the upward search are:
    // - virtual surface is allowed
    // - searching up
    // - the current voxel is free
    // - the previous voxel was unknown
    // - we do not already have a virtual voxel
    best_virtual = (allow_virtual_surface && search_up && free && last_unknown && best_virtual.isNull()) ? current_key :
                                                                                                           best_virtual;

    // This is the case for searching down. In this case we are always looking for the lowest virtual voxel.
    // We progressively select the last voxel as the new virtual voxel provided it was considered free and the current
    // voxel is unknown (not free and not occupied). We only need to check free as we will have exited on an occupied
    // voxel. The conditions here are:
    // - virtual surface is allowed
    // - searching down (!search_up)
    // - the last voxel was free
    // - the current voxel is unknown - we only need check !free at this point
    // Otherwise we keep the current candidate
    best_virtual = (allow_virtual_surface && !search_up && last_free && !free) ? last_key : best_virtual;

    // Cache values for the next iteration.
    last_unknown = !occupied && !free;
    last_free = free;
    last_key = current_key;

    // Calculate the next voxel.
    int next_step = step;
    if (!voxel.occupancy.chunk())
    {
      // The current voxel is an empty chunk implying all unknown voxels. We will skip to the last voxel in this
      // chunk. We don't skip the whole chunk to allow the virtual voxel calculation to take effect.
      next_step = (step > 0) ? voxel.occupancy.layerDim()[up_axis_index] - current_key.localKey()[up_axis_index] :
                               -(1 + current_key.localKey()[up_axis_index]);
      i += std::abs(next_step) - 1;
    }

    // Single step in the current region.
    voxel.map().moveKeyAlongAxis(current_key, up_axis_index, next_step);
  }

  if (best_virtual.isNull())
  {
    if (allow_virtual_surface && !search_up && last_free)
    {
      best_virtual = last_key;
    }
    else
    {
      *offset = -1;
    }
  }

  *is_virtual = !best_virtual.isNull();

  // We only get here if we haven't found an occupied voxel. Return the best virtual one.
  return best_virtual;
}

inline Key findNearestSupportingVoxel(SrcVoxel &voxel, const Key &seed_key, UpAxis up_axis, const Key &min_key,
                                      const Key &max_key, int voxel_ceiling, int clearance_voxel_count_permissive,
                                      bool allow_virtual_surface, bool promote_virtual_below)
{
  PROFILE(findNearestSupportingVoxel);
  Key above;
  Key below;
  int offset_below = -1;
  int offset_above = -1;
  bool virtual_below = false;
  bool virtual_above = false;

  const int up_axis_index = (int(up_axis) >= 0) ? int(up_axis) : -int(up_axis) - 1;
  const Key &search_down_to = (int(up_axis) >= 0) ? min_key : max_key;
  const Key &search_up_to = (int(up_axis) >= 0) ? max_key : min_key;
  below = findNearestSupportingVoxel2(voxel, seed_key, search_down_to, up_axis_index, 0, false, allow_virtual_surface,
                                      &offset_below, &virtual_below);
  above = findNearestSupportingVoxel2(voxel, seed_key, search_up_to, up_axis_index, voxel_ceiling, true,
                                      allow_virtual_surface, &offset_above, &virtual_above);

  const bool have_candidate_below = offset_below >= 0;
  const bool have_candidate_above = offset_above >= 0;

  // Ignore the fact that the voxel below is virtual when prefer_virtual_below is set.
  virtual_below = have_candidate_below && virtual_below && !promote_virtual_below;

  // Prefer non-virtual over virtual. Prefer the closer result.
  if (have_candidate_below && virtual_above && !virtual_below)
  {
    return below;
  }

  if (have_candidate_above && !virtual_above && virtual_below)
  {
    return above;
  }

  // We never allow virtual voxels above as this generates better heightmaps. Virtual surfaces are more interesting
  // when approaching a slope down than any such information above.
  if (have_candidate_below && virtual_above && virtual_below)
  {
    return below;
  }

  // When both above and below have valid candidates. We prefer the lower one if there is sufficient clearance from
  // it to the higher one (should be optimistic). Otherwise we prefer the one which has had less searching.
  if (have_candidate_below &&
      (!have_candidate_above || offset_below <= offset_above ||
       have_candidate_below && have_candidate_above && offset_below + offset_above >= clearance_voxel_count_permissive))
  {
    return below;
  }

  return above;
}

Key findGround(double *height_out, double *clearance_out, SrcVoxel &voxel, const Key &seed_key, const Key &min_key,
               const Key &max_key, const HeightmapDetail &imp)
{
  PROFILE(findGround);
  // Start with the seed_key and look for ground. We only walk up from the seed key.
  double column_height = std::numeric_limits<double>::max();
  double column_clearance_height = column_height;

  // Start walking the voxels in the source map.
  // glm::dvec3 column_reference = heightmap.voxelCentreGlobal(target_key);

  // Walk the src column up.
  const int up_axis_index = imp.vertical_axis_index;
  // Select walking direction based on the up axis being aligned with the primary axis or not.
  const int step_dir = (int(imp.up_axis_id) >= 0) ? 1 : -1;
  glm::dvec3 sub_voxel_pos(0);
  glm::dvec3 column_voxel_pos(0);
  double height = 0;
  OccupancyType candidate_voxel_type = ohm::kNull;
  OccupancyType last_voxel_type = ohm::kNull;

  Key ground_key = Key::kNull;
  for (Key key = seed_key; key.isBounded(up_axis_index, min_key, max_key);
       voxel.map().stepKey(key, up_axis_index, step_dir))
  {
    // PROFILE(column);
    voxel.setKey(key);

    const OccupancyType voxel_type = sourceVoxelHeight(&sub_voxel_pos, &height, voxel, imp.up);

    // We check the clearance and consider a new candidate if we have encountered an occupied voxel, or
    // we are considering virtual surfaces. When considering virtual surfaces, we also check clearance where we
    // have transitioned from unobserved to free and we do not already have a candidate voxel. In this way
    // only occupied voxels can obstruct the clearance value and only the lowest virtual voxel will be considered as
    // a surface.
    const bool last_is_unobserved = last_voxel_type == ohm::kUnobserved || last_voxel_type == ohm::kNull;
    if (voxel_type == ohm::kOccupied || imp.generate_virtual_surface && last_is_unobserved &&
                                          voxel_type == ohm::kFree && candidate_voxel_type == ohm::kNull)
    {
      if (candidate_voxel_type != ohm::kNull)
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
        candidate_voxel_type = voxel_type;
      }
      else
      {
        // Branch condition only for the first voxel in column.
        ground_key = key;
        column_height = column_clearance_height = height;
        column_voxel_pos = sub_voxel_pos;
        candidate_voxel_type = voxel_type;
      }
    }

    last_voxel_type = voxel_type;
  }

  // Did we find a valid candidate?
  if (candidate_voxel_type != ohm::kNull)
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
  PlaneFillWalker::Revisit revisit_behaviour = PlaneFillWalker::Revisit::kNone;
  if (!candidate_key.isNull())
  {
    revisit_behaviour = int(imp.up_axis_id) >= 0 ? PlaneFillWalker::Revisit::kLower : PlaneFillWalker::Revisit::kHigher;
  }
  walker.addNeighbours(ground_key, revisit_behaviour);
}
}  // namespace

Heightmap::Heightmap()
  : Heightmap(0.2, 2.0, UpAxis::kZ)  // NOLINT(readability-magic-numbers)
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
  imp_->heightmap = std::make_unique<OccupancyMap>(grid_resolution, region_dim);

  // Setup the heightmap voxel layout.
  MapLayout &layout = imp_->heightmap->layout();

  layout.filterLayers({ default_layer::occupancyLayerName(), default_layer::meanLayerName() });

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
  voxels.addMember("normal_x", DataType::kFloat, 0);
  voxels.addMember("normal_y", DataType::kFloat, 0);
  voxels.addMember("normal_z", DataType::kFloat, 0);
  voxels.addMember("reserved", DataType::kFloat, 0);

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


void Heightmap::setIgnoreVoxelMean(bool ignore)
{
  imp_->ignore_voxel_mean = ignore;
}


bool Heightmap::ignoreVoxelMean() const
{
  return imp_->ignore_voxel_mean;
}


void Heightmap::setGenerateVirtualSurface(bool enable)
{
  imp_->generate_virtual_surface = enable;
}


bool Heightmap::generateVirtualSurface() const
{
  return imp_->generate_virtual_surface;
}


void Heightmap::setPromoteVirtualBelow(bool enable)
{
  imp_->promote_virtual_below = enable;
}


bool Heightmap::promoteVirtualBelow() const
{
  return imp_->promote_virtual_below;
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

  unsigned processed_count = 0;
  if (!imp_->use_flood_fill)
  {
    const Key planar_key = src_map.voxelKey(reference_pos);
    PlaneWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id, &planar_key);
    processed_count = buildHeightmapT(walker, reference_pos);
  }
  else
  {
    PlaneFillWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id, false);
    processed_count = buildHeightmapT(walker, reference_pos, &onVisitPlaneFill);
  }

#if PROFILING
  ohm::Profile::instance().report();
#endif  // PROFILING

  return processed_count;
}


HeightmapVoxelType Heightmap::getHeightmapVoxelInfo(const Key &key, glm::dvec3 *pos, HeightmapVoxel *voxel_info) const
{
  if (!key.isNull())
  {
    Voxel<const float> heightmap_occupancy(imp_->heightmap.get(), imp_->heightmap->layout().occupancyLayer(), key);

    if (heightmap_occupancy.isValid())
    {
      Voxel<const HeightmapVoxel> heightmap_voxel(imp_->heightmap.get(), imp_->heightmap_layer, key);
      Voxel<const VoxelMean> mean_voxel(imp_->heightmap.get(), imp_->heightmap->layout().meanLayer(), key);

      const glm::dvec3 voxel_centre = imp_->heightmap->voxelCentreGlobal(key);
      *pos = mean_voxel.isLayerValid() ? positionSafe(mean_voxel) : voxel_centre;
      float occupancy;
      heightmap_occupancy.read(&occupancy);
      const bool is_uncertain = occupancy == ohm::unobservedOccupancyValue();
      const float heightmap_voxel_value = (!is_uncertain) ? occupancy : -1.0f;
      // isValid() is somewhat redundant, but it silences a clang-tidy check.
      if (!is_uncertain && heightmap_voxel.isValid())
      {
        // Get height info.
        HeightmapVoxel heightmap_info;
        heightmap_voxel.read(&heightmap_info);
        (*pos)[upAxisIndex()] = voxel_centre[upAxisIndex()] + heightmap_info.height;
        if (voxel_info)
        {
          *voxel_info = heightmap_info;
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
    return HeightmapVoxelType::kUnknown;
  }
  return HeightmapVoxelType::kUnknown;
}


void Heightmap::updateMapInfo(MapInfo &info) const
{
  imp_->toMapInfo(info);
}


Key &Heightmap::project(Key *key) const
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

  // Allow voxel mean positioning.
  const bool use_voxel_mean = src_map.voxelMeanEnabled() && !imp_->ignore_voxel_mean;
  if (use_voxel_mean)
  {
    heightmap.addVoxelMeanLayer();
  }

  PROFILE(walk)

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
  const glm::dvec3 up_axis_normal = upAxisNormal();
  unsigned populated_count = 0;
  const int voxel_ceiling = ohm::pointToRegionCoord(imp_->ceiling, src_map.resolution());
  const int clearance_voxel_count_permissive =
    std::max(1, ohm::pointToRegionCoord(imp_->min_clearance, src_map.resolution()) - 1);

  SrcVoxel src_voxel(src_map, use_voxel_mean);
  DstVoxel hm_voxel(heightmap, imp_->heightmap_layer, use_voxel_mean);

  do
  {
    // Find the nearest voxel to the current key which may be a ground candidate.
    // This is key closest to the walk_key which could be ground. This will be either an occupied voxel, or virtual
    // ground voxel.
    // Virtual ground is where a free is supported by an uncertain or null voxel below it.
    Key candidate_key = findNearestSupportingVoxel(src_voxel, walk_key, upAxis(), walker.min_ext_key,
                                                   walker.max_ext_key, voxel_ceiling, clearance_voxel_count_permissive,
                                                   imp_->generate_virtual_surface, imp_->promote_virtual_below);

    // Walk up from the candidate to find the best heightmap voxel.
    double height = 0;
    double clearance = 0;
    // Walk the column of candidate_key to find the first occupied voxel with sufficent clearance. A virtual voxel
    // with sufficient clearance may be given if there is no valid occupied voxel.
    const Key ground_key = (!candidate_key.isNull()) ? findGround(&height, &clearance, src_voxel, candidate_key,
                                                                  walker.min_ext_key, walker.max_ext_key, *imp_) :
                                                       walk_key;

    if (on_visit)
    {
      (*on_visit)(walker, *imp_, candidate_key, ground_key);
    }

    // Write to the heightmap.
    src_voxel.setKey(ground_key);
    src_voxel.syncKey();
    const OccupancyType voxel_type = src_voxel.occupancyType();

    // We use the voxel centre for lookup in the local cache for better consistency. Otherwise lateral drift in
    // subvoxel positioning can result in looking up the wrong cell.
    glm::dvec3 src_voxel_centre =
      (src_voxel.occupancy.isValid()) ? src_voxel.centre() : src_voxel.map().voxelCentreGlobal(ground_key);
    // We only use voxel mean positioning for occupied voxels. The information is unreliable for free voxels.
    glm::dvec3 voxel_pos = (voxel_type == kOccupied) ? src_voxel.position() : src_voxel_centre;

    if (voxel_type == kOccupied || imp_->generate_virtual_surface)
    {
      // Real or virtual surface.
      float surface_value = (voxel_type == kOccupied) ? kHeightmapSurfaceValue : kHeightmapVirtualSurfaceValue;

      if (voxel_type != kUnobserved && voxel_type != kNull)
      {
        // Cache the height then clear from the position.
        const double src_height = voxel_pos[upAxisIndex()];
        voxel_pos[upAxisIndex()] = 0;

        // Get the heightmap voxel to update.
        Key hm_key = heightmap.voxelKey(voxel_pos);
        project(&hm_key);
        // TODO(KS): Using the Voxel interface here is highly suboptimal. This needs to be modified to access the
        // MapChunk directly for efficiency.
        hm_voxel.setKey(hm_key);
        // We can do a direct occupancy value write with no checks for the heightmap. The value is explicit.
        assert(hm_voxel.occupancy.isValid() && hm_voxel.occupancy.voxelMemory());
        hm_voxel.occupancy.write(surface_value);
        // Set voxel mean position as required. Will be skipped if not enabled.
        hm_voxel.setPosition(voxel_pos);

        // Write the height and clearance values.
        HeightmapVoxel height_info;
        hm_voxel.heightmap.read(&height_info);
        height_info.height = relativeVoxelHeight(src_height, hm_key, heightmap, imp_->up);
        height_info.clearance = float(clearance);
        height_info.normal_x = height_info.normal_y = height_info.normal_z = 0;

        if (voxel_type == kOccupied && src_voxel.covariance.isValid())
        {
          CovarianceVoxel cov;
          src_voxel.covariance.read(&cov);
          glm::dvec3 normal;
          covarianceEstimatePrimaryNormal(&cov, &normal);
          const double flip = (glm::dot(normal, up_axis_normal) >= 0) ? 1.0 : -1.0;
          normal *= flip;
          height_info.normal_x = float(normal.x);
          height_info.normal_y = float(normal.y);
          height_info.normal_z = float(normal.z);
        }
        hm_voxel.heightmap.write(height_info);

        ++populated_count;
      }
    }
  } while (walker.walkNext(walk_key));

  return populated_count != 0;
}
}  // namespace ohm
