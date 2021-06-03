// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Heightmap.h"

#include "private/HeightmapDetail.h"
#include "private/HeightmapOperations.h"

#include "Aabb.h"
#include "CovarianceVoxel.h"
#include "DefaultLayer.h"
#include "HeightmapUtil.h"
#include "MapChunk.h"
#include "MapCoord.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "PlaneFillLayeredWalker.h"
#include "PlaneFillWalker.h"
#include "PlaneWalker.h"
#include "Trace.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

// Include after GLM types for glm type streaming operators.
#include "ohmutil/GlmStream.h"

#define PROFILING 0
#include <ohmutil/Profile.h>

#include <3esservermacros.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

// Enable code to support breaking on a specific voxel.
#define HM_DEBUG_VOXEL 0

namespace ohm
{
namespace heightmap
{
/// Helper function for visiting a heightmap node. This expands into the neighbours as required and performs debug
/// rendering.
/// @param walker The class used to walk the heightmap region. Examples; @c PlaneWalker , @c PlanerFillWalker ,
///     @c PlanerFillLayeredWalker
/// @param imp Heightmap implementation.
/// @param walk_key The initial key extracted from the @c walker before calculating the candidate and/or ground keys.
/// @param candidate_key The initial ground candidate key calculated from @p walk_key as the starting point to search
///   for the @p ground_key . May be null.
/// @param ground_key The selected ground key. May be null.
template <typename Walker>
void onVisitWalker(Walker &walker, const HeightmapDetail &imp, const Key &walk_key, const Key &candidate_key,
                   const Key &ground_key)
{
  (void)candidate_key;  // Unused
  (void)imp;            // Unused without TES_ENABLE
  (void)walk_key;       // Unused without TES_ENABLE
  // Add neighbours for walking.
  std::array<Key, 8> neighbours;
  const auto mode = (!candidate_key.isNull()) ? PlaneWalkVisitMode::kAddUnvisitedNeighbours :
                                                PlaneWalkVisitMode::kAddUnvisitedColumnNeighbours;
  size_t added_count = walker.visit(ground_key, mode, neighbours);
  (void)added_count;  // Unused unless TES_ENABLE is defined.
#ifdef TES_ENABLE
  if (g_tes)
  {
    // Add the neighbours for debug visualisation
    KeyRange source_map_range;
    uint32_t voxel_id;
    imp.occupancy_map->calculateExtents(nullptr, nullptr, &source_map_range);
    for (size_t i = 0; i < added_count; ++i)
    {
      const Key &nkey = neighbours[i];
      voxel_id = uint32_t(source_map_range.indexOf(nkey));
      voxel_id |= kNeighbourIdMask;
      const glm::dvec3 pos = imp.occupancy_map->voxelCentreGlobal(nkey);
      tes::Box neighbour(tes::Id(voxel_id, kTcHmVisit), tes::Transform(tes::Vector3d(glm::value_ptr(pos)),
                                                                       tes::Vector3d(imp.heightmap->resolution())));
      neighbour.setColour(tes::Colour::Colours[tes::Colour::CornflowerBlue]).setWireframe(true);
      neighbour.setReplace(true);
      g_tes->create(neighbour);
    }

    // Delete the previous candidate voxel visualisation.
    voxel_id = uint32_t(source_map_range.indexOf(walk_key));
    voxel_id |= kNeighbourIdMask;
    g_tes->destroy(tes::Box(tes::Id(voxel_id, kTcHmVisit)));

    /// Visualise the candiate with a transient box (single frame)
    const glm::dvec3 pos = imp.occupancy_map->voxelCentreGlobal(walk_key);
    tes::Box candidate(tes::Id(0, kTcHmVisit),
                       tes::Transform(tes::Vector3d(glm::value_ptr(pos)), tes::Vector3d(imp.heightmap->resolution())));
    candidate.setColour(tes::Colour::Colours[tes::Colour::LightGoldenrodYellow]).setWireframe(true);
    g_tes->create(candidate);

    std::ostringstream info;
    info << "R" << walk_key.regionKey() << " L" << walk_key.localKey();
    const std::string str = info.str();
    tes::Text3D info_text(str.c_str(), tes::Id(0, kTcHmInfo), tes::Directional(tes::Vector3d(glm::value_ptr(pos))));
    info_text.setScreenFacing(true).setColour(candidate.colour());
    g_tes->create(info_text);
  }
#endif  // neighbours
}
}  // namespace heightmap


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
  region_dim[imp_->vertical_axis_index] = 1;
  // Note: Compression is disabled for the heightmap.
  imp_->heightmap = std::make_unique<OccupancyMap>(grid_resolution, region_dim, MapFlag::kNone);
  // The multilayer heightmap expects more entries. Default to having room for N layers per chunk.
  region_dim[imp_->vertical_axis_index] = 4;
  imp_->multilayer_heightmap = std::make_unique<OccupancyMap>(grid_resolution, region_dim);

  imp_->heightmap_voxel_layer = heightmap::setupHeightmap(*imp_->heightmap, *imp_);
  heightmap::setupHeightmap(*imp_->multilayer_heightmap, *imp_);
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


void Heightmap::setFloor(double floor)
{
  imp_->floor = floor;
}


double Heightmap::floor() const
{
  return imp_->floor;
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


void Heightmap::setMode(HeightmapMode mode)
{
  imp_->mode = mode;
}


HeightmapMode Heightmap::mode() const
{
  return imp_->mode;
}


UpAxis Heightmap::upAxis() const
{
  return UpAxis(imp_->up_axis_id);
}


int Heightmap::upAxisIndex() const
{
  return imp_->vertical_axis_index;
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


void Heightmap::setVirtualSurfaceFilterThreshold(unsigned threshold)
{
  imp_->virtual_surface_filter_threshold = threshold;
}


unsigned Heightmap::virtualSurfaceFilterThreshold() const
{
  return imp_->virtual_surface_filter_threshold;
}


int Heightmap::heightmapVoxelLayer() const
{
  return imp_->heightmap_voxel_layer;
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
  unsigned supporting_voxel_flags = !!imp_->generate_virtual_surface * heightmap::kVirtualSurfaces |
                                    !!imp_->promote_virtual_below * heightmap::kPromoteVirtualBelow;
  switch (imp_->mode)
  {
  case HeightmapMode::kPlanar:  //
  {
    // Pure planar walk must prefer below and does better ignoring virtual surfaces above the plane.
    supporting_voxel_flags |= heightmap::kIgnoreVirtualAbove;
    const Key planar_key = src_map.voxelKey(reference_pos);
    PlaneWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id, &planar_key);
    processed_count = buildHeightmapT(walker, reference_pos, supporting_voxel_flags, supporting_voxel_flags);
    break;
  }
  case HeightmapMode::kSimpleFill:  //
  {
    // We should prefer voxels below for the first iteration, when seeding, after that we prefer the closest candidate
    // supporting voxel to the seed voxel.
    const unsigned initial_supporting_voxel_flags = supporting_voxel_flags;
    supporting_voxel_flags |= heightmap::kBiasAbove;
    PlaneFillWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id);
    processed_count = buildHeightmapT(walker, reference_pos, initial_supporting_voxel_flags, supporting_voxel_flags);
    break;
  }
  case HeightmapMode::kLayeredFillUnordered:  //
  // No break
  case HeightmapMode::kLayeredFill:  //
  {
    // We should prefer voxels below for the first iteration, when seeding, after that we prefer the closest candidate
    // supporting voxel to the seed voxel.
    const unsigned initial_supporting_voxel_flags = supporting_voxel_flags;
    supporting_voxel_flags |= heightmap::kBiasAbove;
    PlaneFillLayeredWalker walker(src_map, min_ext_key, max_ext_key, imp_->up_axis_id);
    processed_count = buildHeightmapT(walker, reference_pos, initial_supporting_voxel_flags, supporting_voxel_flags);
    break;
  }
  default:
    break;
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
      Voxel<const HeightmapVoxel> heightmap_voxel(imp_->heightmap.get(), imp_->heightmap_voxel_layer, key);
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
        *pos = voxel_centre + imp_->up * double(heightmap_info.height);
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


double Heightmap::getVoxelHeight(const Key &key, double height) const
{
  return heightmap::getVoxelHeight(*imp_->heightmap, imp_->up, key, height);
}


double Heightmap::getVoxelHeight(const Key &key, const HeightmapVoxel &info) const
{
  return heightmap::getVoxelHeight(*imp_->heightmap, imp_->up, key, double(info.height));
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


KeyRange Heightmap::buildReferencePlaneSlice(Key min_key, Key max_key, const glm::dvec3 &reference_pos) const
{
  const OccupancyMap &src_map = *imp_->occupancy_map;
  // Calculate the key range which marks the vertical slice through the reference map which covers the vertical range
  //  [reference_pos_height - floor, reference_pos_height + ceiling]
  const double height_sign = (int(imp_->up_axis_id) >= 0) ? 1.0 : -1.0;
  glm::dvec2 planar_slice(reference_pos[imp_->vertical_axis_index] - height_sign * imp_->floor,
                          reference_pos[imp_->vertical_axis_index] + height_sign * imp_->ceiling);
  if (planar_slice[0] > planar_slice[1])
  {
    std::swap(planar_slice[0], planar_slice[1]);
  }

  // Adjust range min/max to match the planar_slice.
  glm::dvec3 ref_pos = reference_pos;
  ref_pos[imp_->vertical_axis_index] = planar_slice[0];
  Key ref_key = src_map.voxelKey(ref_pos);
  min_key.setAxisFrom(imp_->vertical_axis_index, ref_key);

  ref_pos = reference_pos;
  ref_pos[imp_->vertical_axis_index] = planar_slice[1];
  ref_key = src_map.voxelKey(ref_pos);
  max_key.setAxisFrom(imp_->vertical_axis_index, ref_key);

  return KeyRange(min_key, max_key, src_map.regionVoxelDimensions());
}


template <typename KeyWalker>
bool Heightmap::buildHeightmapT(KeyWalker &walker, const glm::dvec3 &reference_pos, unsigned initial_supporting_flags,
                                unsigned iterating_supporting_flags)
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
  if (!walk_key.isBounded(walker.minKey(), walker.maxKey()))
  {
    walk_key.clampToAxis(surfaceAxisIndexA(), walker.minKey(), walker.maxKey());
    walk_key.clampToAxis(surfaceAxisIndexB(), walker.minKey(), walker.maxKey());
  }

  if (!walker.begin(walk_key))
  {
    return false;
  }

  // Walk the 2D extraction region in a spiral around walk_key.
  unsigned populated_count = 0;
  // Convert the search floor and ceiling values to voxel counts.
  const int voxel_floor = ohm::pointToRegionCoord(imp_->floor, src_map.resolution());
  const int voxel_ceiling = ohm::pointToRegionCoord(imp_->ceiling, src_map.resolution());
  const int clearance_voxel_count_permissive =
    std::max(1, ohm::pointToRegionCoord(imp_->min_clearance, src_map.resolution()) - 1);

  heightmap::SrcVoxel src_voxel(src_map, use_voxel_mean);
  heightmap::DstVoxel hm_voxel(heightmap, imp_->heightmap_voxel_layer, use_voxel_mean);

  // Calculate the key range which marks the vertical slice through the reference map which covers the vertical range
  //  [reference_pos_height - floor, reference_pos_height + ceiling]
  const KeyRange reference_planar_slice = buildReferencePlaneSlice(walker.minKey(), walker.maxKey(), reference_pos);

#if HM_DEBUG_VOXEL
  const glm::dvec3 debug_pos(2.05, 0.75, 0);
  const Key debug_src_key(1, -5, 0, 14, 28, 19);
#endif  // HM_DEBUG_VOXEL
  unsigned supporting_voxel_flags = initial_supporting_flags;
  // Tracks voxels which have results at multiple layers for a heightmap support isMultiLayered()
  std::set<ohm::Key> multi_layer_keys;
  // We use this map to collect data for virtual surface filtering step. It maps from source voxel keys to heightmap
  // key and voxel type.
  std::unordered_map<ohm::Key, heightmap::HeightmapKeyType> src_to_heightmap_keys;
  const bool ordered_layers = areLayersSorted();  // True to sort multi-layered configurations.
  bool abort = false;
  do
  {
#if HM_DEBUG_VOXEL
    const glm::dvec3 ref_pos = src_map.voxelCentreGlobal(walk_key);
    if (std::abs(ref_pos.x - debug_pos.x) < 0.5 * src_map.resolution() &&
        std::abs(ref_pos.y - debug_pos.x) < 0.5 * src_map.resolution())
    {
      int stopme = 1;
    }

    if (walk_key == debug_src_key)
    {
      int stopme = 2;
    }
#endif  // HM_DEBUG_VOXEL

    // Mark whether this voxel may be a base layer candidate. This is always true for non-layered heightmaps.
    bool is_initial_key_in_planar_slice =
      !isMultiLayered() || walk_key.isBounded(reference_planar_slice.minKey(), reference_planar_slice.maxKey());

    // Find the nearest voxel to the current key which may be a ground candidate.
    // This is key closest to the walk_key which could be ground. This will be either an occupied voxel, or virtual
    // ground voxel.
    // Virtual ground is where a free is supported by an uncertain or null voxel below it.
    Key candidate_key = heightmap::findNearestSupportingVoxel(src_voxel, walk_key, upAxis(), walker.minKey(),
                                                              walker.maxKey(), voxel_floor, voxel_ceiling,
                                                              clearance_voxel_count_permissive, supporting_voxel_flags);

    is_initial_key_in_planar_slice =
      is_initial_key_in_planar_slice ||
      candidate_key.isBounded(reference_planar_slice.minKey(), reference_planar_slice.maxKey());

    // Walk up from the candidate to find the best heightmap voxel.
    double height = 0;
    double clearance = 0;
    // Walk the column of candidate_key to find the first occupied voxel with sufficent clearance. A virtual voxel
    // with sufficient clearance may be given if there is no valid occupied voxel.
    const Key ground_key = (!candidate_key.isNull()) ? findGround(&height, &clearance, src_voxel, candidate_key,
                                                                  walker.minKey(), walker.maxKey(), *imp_) :
                                                       walk_key;

    is_initial_key_in_planar_slice =
      is_initial_key_in_planar_slice ||
      ground_key.isBounded(reference_planar_slice.minKey(), reference_planar_slice.maxKey());

    heightmap::onVisitWalker(walker, *imp_, walk_key, candidate_key, ground_key);

    // Write to the heightmap.
    src_voxel.setKey(ground_key);
    src_voxel.syncKey();
    // There's a fairly subtle issue we can resolve here. Using say a planar walk, we can end up with a voxel where
    // candidate_key is null so ground_key will be the same as walk_key. If that's of type kFree, then we can
    // incorrectly generate very flat sections of virtual surface. Although this could be fixed above, by ensuring the
    // ground_key is null, some other walk techniques can rely on it no being null. So, we apply a late fix and consider
    // our vxoel to be kNull when candidate_key is null. This actually has a positive impact on layered generation too
    // eliminating some undesirable "streamers" of virtual surface.
    const OccupancyType voxel_type = (!candidate_key.isNull()) ? src_voxel.occupancyType() : OccupancyType::kNull;

    if (voxel_type == kOccupied || (voxel_type == kFree && imp_->generate_virtual_surface))
    {
      // We use the voxel centre for lookup in the local cache for better consistency. Otherwise lateral drift in
      // subvoxel positioning can result in looking up the wrong cell.
      glm::dvec3 src_voxel_centre =
        (src_voxel.occupancy.isValid()) ? src_voxel.centre() : src_voxel.map().voxelCentreGlobal(ground_key);
      // We only use voxel mean positioning for occupied voxels. The information is unreliable for free voxels.
      glm::dvec3 voxel_pos = (voxel_type == kOccupied) ? src_voxel.position() : src_voxel_centre;

      HeightmapVoxelType hm_voxel_type = addSurfaceVoxel(hm_voxel, src_voxel, voxel_type, clearance, voxel_pos,
                                                         multi_layer_keys, is_initial_key_in_planar_slice);
      if (hm_voxel_type != HeightmapVoxelType::kUnknown)
      {
        ++populated_count;
        // Populate src_to_heightmap_keys if we are using it.
        if (ordered_layers && imp_->virtual_surface_filter_threshold > 0)
        {
          src_to_heightmap_keys.emplace(ground_key,
                                        heightmap::HeightmapKeyType{ hm_voxel.occupancy.key(), hm_voxel_type });
        }
      }
    }

    TES_SERVER_UPDATE(g_tes, 0.0f);
    supporting_voxel_flags = iterating_supporting_flags;
  } while (!abort && walker.walkNext(walk_key));

  if (ordered_layers)
  {
    // Mark virtual surface voxels for removal.
    if (imp_->virtual_surface_filter_threshold > 0)
    {
      heightmap::filterVirtualVoxels(*imp_, imp_->virtual_surface_filter_threshold, src_to_heightmap_keys);
    }

    // Sort layers and remove filtered virtual surface voxels.
    heightmap::sortHeightmapLayers(*imp_, multi_layer_keys, use_voxel_mean);
  }

  return populated_count != 0;
}


HeightmapVoxelType Heightmap::addSurfaceVoxel(heightmap::DstVoxel &hm_voxel, const heightmap::SrcVoxel &src_voxel,
                                              const OccupancyType voxel_type, double clearance, glm::dvec3 voxel_pos,
                                              std::set<ohm::Key> &multi_layer_keys, bool is_base_layer_candidate)
{
  if (voxel_type != kUnobserved && voxel_type != kNull)
  {
    OccupancyMap &heightmap = *imp_->heightmap;

    // Real or virtual surface.
    const HeightmapVoxelType add_voxel_type =
      (voxel_type == kOccupied) ? HeightmapVoxelType::kSurface : HeightmapVoxelType::kVirtualSurface;
    float surface_value = (voxel_type == kOccupied) ? kHeightmapSurfaceValue : kHeightmapVirtualSurfaceValue;

    // Cache the height then clear from the position.
    const double src_height = glm::dot(imp_->up, voxel_pos);
    voxel_pos[upAxisIndex()] = 0;

    // Get the heightmap voxel to update.
    Key hm_key = heightmap.voxelKey(voxel_pos);
    project(&hm_key);
    // TODO(KS): Using the Voxel interface here is highly suboptimal. This needs to be modified to access the
    // MapChunk directly for efficiency.
    hm_voxel.setKey(hm_key);

    // We can do a direct occupancy value write with no checks for the heightmap. The value is explicit.
    assert(hm_voxel.occupancy.isValid() && hm_voxel.occupancy.voxelMemory());

    bool should_add = true;
    // For multi-layered heightmaps, we need to check occupancy and not overwrite existing results.
    if (isMultiLayered())
    {
      if (hm_voxel.occupancy.data() != ohm::unobservedOccupancyValue())
      {
        // It's possible to visit the same 2D voxel at different candidate heights, but generate the same result
        // as a previous visitation. We check for this below.
        if (hm_voxel.haveRecordedHeight(src_height, upAxisIndex(), imp_->up))
        {
          // It's a repeat. Don't add.
          should_add = false;
        }
        else
        {
          // Store the base key to ensure we update multi_layer_keys if we are still adding this voxel.
          const auto base_key = hm_key;

          // We need to insert the voxel at the next available voxel in the target column. This results in unsorted
          // insertion with layer sorting optionally occuring as a post process.
          //
          // During this walk up the column we also validate that a virtual surface voxel is not being added too close
          // to another virtual surface voxel.
          double nearest_voxel_below = 0;
          double nearest_voxel_above = 0;
          do
          {
            // Check the distance to the existing voxel.
            const double current_voxel_height =
              hm_voxel.heightmap.data().height + glm::dot(imp_->up, heightmap.voxelCentreGlobal(hm_key));
            const double current_voxel_delta = current_voxel_height - src_height;
            nearest_voxel_below =
              (current_voxel_delta < 0 && (nearest_voxel_below <= 0 || -current_voxel_delta < nearest_voxel_below)) ?
                -current_voxel_delta :
                nearest_voxel_below;
            nearest_voxel_above =
              (current_voxel_delta > 0 && (nearest_voxel_above <= 0 || current_voxel_delta < nearest_voxel_above)) ?
                current_voxel_delta :
                nearest_voxel_above;

            // Walk to the next key up in the heightmap. We always move the key up as the height cells are
            // independent.
            heightmap.moveKeyAlongAxis(hm_key, upAxisIndex(), 1);
            hm_voxel.setKey(hm_key);
            assert(hm_voxel.occupancy.isValid() && hm_voxel.occupancy.voxelMemory());
          } while (hm_voxel.occupancy.data() != ohm::unobservedOccupancyValue());

          // Prevent adding a virtual surface voxel which is too close to an existing voxel.
          // We use the clearance height as the threshold.
          if ((nearest_voxel_below > 0 && nearest_voxel_below <= imp_->min_clearance) ||
              (nearest_voxel_above > 0 && nearest_voxel_above <= imp_->min_clearance))
          {
            should_add = false;
          }

          // We've now found where to insert and validated the insertion. Ensure we have base_key in multi_layer_keys
          // if we are still adding.
          if (should_add && areLayersSorted())
          {
            multi_layer_keys.insert(base_key);
          }
        }
      }
    }

    if (should_add)
    {
      hm_voxel.occupancy.write(surface_value);
      // Set voxel mean position as required. Will be skipped if not enabled.
      hm_voxel.setPosition(voxel_pos);

      // Write the height and clearance values.
      HeightmapVoxel height_info{};
      // Calculate the relative voxel height now that we have a target voxel key which may consider multi-layering.
      // Later sorting may change the HeightmapVoxel::height value as the voxel may be changed.
      height_info.height = heightmap::relativeVoxelHeight(src_height, hm_key, heightmap, imp_->up);
      height_info.clearance = float(clearance);
      height_info.normal_x = height_info.normal_y = height_info.normal_z = 0;
      height_info.layer = (is_base_layer_candidate) ? kHvlBaseLayer : kHvlExtended;
      height_info.contributing_samples =
        uint16_t((src_voxel.mean.isValid()) ? std::min(src_voxel.mean.data().count, 0xffffu) : 0u);

      if (voxel_type == kOccupied && src_voxel.covariance.isValid())
      {
        CovarianceVoxel cov;
        src_voxel.covariance.read(&cov);
        glm::dvec3 normal;
        covarianceEstimatePrimaryNormal(&cov, &normal);
        const double flip = (glm::dot(normal, upAxisNormal()) >= 0) ? 1.0 : -1.0;
        normal *= flip;
        height_info.normal_x = float(normal.x);
        height_info.normal_y = float(normal.y);
        height_info.normal_z = float(normal.z);
      }
      hm_voxel.heightmap.write(height_info);

      hm_voxel.debugDraw(imp_->debug_level, imp_->vertical_axis_index, int(imp_->up_axis_id) >= 0 ? 1.0 : -1.0);
      return add_voxel_type;
    }
  }

  return HeightmapVoxelType::kUnknown;
}
}  // namespace ohm
