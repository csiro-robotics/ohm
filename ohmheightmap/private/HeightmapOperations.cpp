// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapOperations.h"

#include "HeightmapDetail.h"

#include "ohmheightmap/Heightmap.h"  // For TES_ENABLE
#include "ohmheightmap/HeightmapVoxelType.h"

#include <ohm/KeyRange.h>
#include <ohm/Trace.h>  // For TES_ENABLE

#define PROFILING 0
#include <ohmutil/Profile.h>

#include <3esservermacros.h>

#include <glm/gtc/type_ptr.hpp>  // For TES_ENABLE

namespace ohm
{
namespace heightmap
{
#ifdef TES_ENABLE
const uint32_t kNeighbourIdMask = 0x80000000u;
#endif  // TES_ENABLE


bool DstVoxel::haveRecordedHeight(double height, int up_axis_index, const glm::dvec3 &up) const
{
  // Check if the current heightmap voxel has already recorded a result at the given height. Only call for valid
  // voxels from multi-layered heightmaps.

  // Create a voxel for iteration. Seed from this.
  const double epsilon = 1e-3 * occupancy.map()->resolution();
  DstVoxel walker;
  walker.occupancy = occupancy;
  walker.heightmap = heightmap;
  // Not interested in mean

  Key key = occupancy.key();
  double voxel_height;
  while (walker.occupancy.isValid() && walker.occupancy.data() != ohm::unobservedOccupancyValue())
  {
    // Convert from voxel relative to absolute height. This can introduce floating point error.
    voxel_height = walker.heightmap.data().height;
    voxel_height += glm::dot(walker.occupancy.map()->voxelCentreGlobal(walker.occupancy.key()), up);
    if (std::abs(voxel_height - height) < epsilon)
    {
      return true;
    }

    // Next voxel in the column.
    occupancy.map()->moveKeyAlongAxis(key, up_axis_index, 1);
    walker.setKey(key);
  }

  return false;
}


uint32_t DstVoxel::get3esVoxelId(const Key &heightmap_key, const OccupancyMap &heightmap, int up_axis_index)
{
  // Generate an ID for the voxel.
  // For this we take the extents of the occupancy map (in this case the heightmap) and assign an ID based on the
  // voxel position in that range.
  KeyRange heightmap_range;
  heightmap.calculateExtents(nullptr, nullptr, &heightmap_range);
  // Note: the vertical extents of the heightmap can change for a layered map as we add layers. We cater for this
  // here by ensuring the vertical extents of heightmap_range is padded out to allow for 32 layers. This effectively
  // reserves 5 bits for vertical indexing, 1 bits masking open list voxels, leaving 26 for horizontal indexing. This
  // supports a 2D region of approximately 8Kx8K voxels and should support maps which can reasonably be debugged using
  // 3es. Larger maps create excessively large 3es data sets.
  // Anything more really needs to use a tes::MutableMesh object.
  Key adjusted_vertical_key(0, 0, 0, 0, 0, 0);
  heightmap.moveKeyAlongAxis(adjusted_vertical_key, up_axis_index, 32);
  heightmap_range.expand(adjusted_vertical_key);
  return uint32_t(heightmap_range.indexOf(heightmap_key));
}

void DstVoxel::debugDraw(int level, int up_axis_index, double up_scale)
{
  (void)level;
  (void)up_axis_index;
  (void)up_scale;
#ifdef TES_ENABLE
  if (occupancy.isValid() && g_tes)
  {
    uint32_t voxel_id = get3esVoxelId(up_axis_index);
    glm::dvec3 voxel_pos = occupancy.map()->voxelCentreGlobal(occupancy.key());
    voxel_pos[up_axis_index] += up_scale * heightmap.data().height;

    tes::Id box_id(voxel_id, kTcHmSurface);
    tes::Colour box_colour = tes::Colour::Colours[tes::Colour::Green];
    const float voxel_value = occupancy.data();
    if (voxel_value == Heightmap::kHeightmapVirtualSurfaceValue)
    {
      box_id.setCategory(kTcHmVirtualSurface);
      box_colour = tes::Colour::Colours[tes::Colour::Orange];
    }
    else if (voxel_value == Heightmap::kHeightmapVacantValue)
    {
      box_id.setCategory(kTcHmVacant);
      box_colour = tes::Colour::Colours[tes::Colour::LightSlateGrey];
    }

    tes::Box voxel(box_id, tes::Transform(glm::value_ptr(voxel_pos), tes::Vector3d(occupancy.map()->resolution())));
    voxel.setReplace(true);
    // voxel.setTransparent(true);
    // Colour surface green, virtual orange, vacant grey
    voxel.setColour(box_colour);
    g_tes->create(voxel);

    // Create a line for the clearance height.
    const double clearance_height = heightmap.data().clearance;
    if (clearance_height > 0)
    {
      glm::dvec3 clearance_dir(0);
      clearance_dir[up_axis_index] = up_scale;

      tes::Arrow clearance(tes::Id(voxel_id, kTcHmClearance),
                           tes::Directional(tes::Vector3d(glm::value_ptr(voxel_pos)),
                                            tes::Vector3d(glm::value_ptr(clearance_dir)), 0.005, clearance_height));
      clearance.setColour(tes::Colour::Colours[tes::Colour::Yellow]);
      clearance.setReplace(true);
      g_tes->create(clearance);
    }
  }
#endif  // TES_ENABLE
}

bool BaseLayerCandidate::isOtherCandidateBetter(const BaseLayerCandidate &other, const double *seed_height) const
{
  // Check for no currently recorded entry.
  if (key.isNull())
  {
    // This entry is null. Prefer other.
    return true;
  }

  if (other.isValid())
  {
    /// Check for clearance above.
    if (!clearAbove() && other.clearAbove())
    {
      // Other entry has a voxel clearance, but this voxel does not. Prefer other.
      return true;
    }

    // Finally select by distance to seed height, but only if both voxels have the same clearAbove() resolution.
    if (clearAbove() == other.clearAbove() && seed_height)
    {
      if (std::abs(other.height - *seed_height) < std::abs(height - *seed_height))
      {
        return true;
      }
    }
  }

  // This is better.
  return false;
}

OccupancyType sourceVoxelHeight(glm::dvec3 *voxel_position, double *height, SrcVoxel &voxel, const glm::dvec3 &up)
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
                                int step_limit, bool search_up, unsigned flags, int *offset, bool *is_virtual)
{
  const bool allow_virtual_surface = (flags & kVirtualSurfaces) != 0;
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
  bool last_unobserved = false;
  bool last_free = false;

  Key last_key(nullptr);
  Key current_key = from_key;

  if (search_up)
  {
    // In the case of searching up, we need to check the seed_key to see if it is unobserved, and therefore potentially
    // of a virtual surface, but not count it as part of our traversal. This is in part because for a virtual surface
    // we report the supporting unobserved voxel, not the free voxel, for the next phase to detect (final reporting is
    // of the free voxel).
    voxel.setKey(from_key);
    last_unobserved = ohm::isUnobservedOrNull(voxel.occupancy);
    last_key = from_key;

    // Now move the key up one voxel for the next iteration.
    voxel.map().moveKeyAlongAxis(current_key, up_axis_index, step);
  }

  // Note: there is an asymmetry to how we use vertical_range. We would tend to explore up one more voxel than down
  // without expanding the vertical range down by 1.
  if (!search_up)
  {
    ++vertical_range;
  }

  *offset = 0;

  // Confusion over iteration counters.
  // We logically have three iteration counters which all do similar things: i, offset and current_key. While similar
  // they are all slightly different:
  // - i is a very simple loop counter. Not much special here, except that we can make it jump towards the end of the
  //  loop in order to skip null regions.
  // - offset marks how far we've searched and is reported to the caller. It is biased such that on the first iteration
  //  searching up has a lower offset value than searching down. After that they fall in sync.
  // - current_key steps up/down following i, but needs separate management since keys have multiple indexing values per
  //  axis : a region index and a local index.

  for (int i = 0; i < vertical_range; ++i)
  {
    // We want to bias the initial voxel selection such that calling code will to choose the upper voxel over the lower
    // one (where they are both occupied) so that we can follow ramps up, rather than choosing the ground below. This is
    // only relevant when the clearance is zero, but is important.
    //
    // In order to create the bias, we manage the offset value in a non-linear fashion where it may be either 0 or 1 on
    // the first iteration - searching up or down respectively. On subsequent iterations it is linear with a value
    // matching `i + 1`.
    //
    // Note when searching up, first voxel checked is the one above the seed voxel.
    //
    // The offset parameter is used to report this information for the caller.
    //
    // The resulting offset series is:
    //
    //             | i=0 | 1 | 2 | 3 | 4 | 5 |
    // ----------- | --- | - | - | - | - | - |
    // Search up   |   0 | 2 | 3 | 4 | 5 | 6 |
    // Search down |   1 | 2 | 3 | 4 | 5 | 6 |
    *offset = (i > 0) ? i + 1 : !search_up;
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
    const bool unobserved = !occupied && !free;

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
    best_virtual = (allow_virtual_surface && search_up && free && last_unobserved && best_virtual.isNull()) ?
                     last_key :
                     best_virtual;

    // This is the case for searching down. In this case we are always looking for the lowest virtual voxel. More
    // specifically, we are looking for the voxel *below* the virtual voxel. We need to return the unobserved voxel
    // supporting the virtual surface voxel in order for later algorithms to detect it as a virtual voxel because a
    // virtual voxel is characterised by the transition from unobserved to free, but the free voxel is the one of
    // (later) interest.
    //
    // We progressively select the current voxel as the new virtual voxel provided the last voxel was considered free
    // and the current voxel is unknown (not free and not occupied). We only need to check free as we will have exited
    // on an occupied voxel. The conditions here are:
    // - virtual surface is allowed
    // - searching down (!search_up)
    // - the last voxel was free
    // - the current voxel is unknown - we only need check !free at this point
    // Otherwise we keep the current candidate
    best_virtual = (allow_virtual_surface && !search_up && unobserved && last_free) ? current_key : best_virtual;

    // Cache values for the next iteration.
    last_unobserved = unobserved;
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
    *offset = -1;
  }

  *is_virtual = !best_virtual.isNull();

  // We only get here if we haven't found an occupied voxel. Return the best virtual one.
  return best_virtual;
}


Key findNearestSupportingVoxel(SrcVoxel &voxel, const Key &seed_key, UpAxis up_axis, const Key &min_key,
                               const Key &max_key, int voxel_floor_limit, int voxel_ceiling_limit,
                               int clearance_voxel_count_permissive, unsigned flags)
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

  below = findNearestSupportingVoxel2(voxel, seed_key, search_down_to, up_axis_index, voxel_floor_limit, false, flags,
                                      &offset_below, &virtual_below);
  above = findNearestSupportingVoxel2(voxel, seed_key, search_up_to, up_axis_index, voxel_ceiling_limit, true, flags,
                                      &offset_above, &virtual_above);

  const bool have_candidate_below = offset_below >= 0;
  const bool have_candidate_above = offset_above >= 0;

  // Ignore the fact that the voxel below is virtual when prefer_virtual_below is set.
  const bool promote_virtual_below = (flags & heightmap::kPromoteVirtualBelow) != 0;
  virtual_below = have_candidate_below && virtual_below && !promote_virtual_below;

  if (flags & heightmap::kBiasAbove)
  {
    // Prefering the closer voxel.
    if (have_candidate_below && have_candidate_above)
    {
      if (offset_below < offset_above)
      {
        return below;
      }

      return above;
    }
  }

  // Prefer non-virtual over virtual. Prefer the closer result.
  if (have_candidate_below && virtual_above && !virtual_below)
  {
    return below;
  }

  if (have_candidate_above && !virtual_above && virtual_below)
  {
    return above;
  }

  if (flags & heightmap::kIgnoreVirtualAbove)
  {
    // Inore virtual voxels above. This this generates better heightmaps from a fixed reference plane. Virtual surfaces
    // are more interesting when approaching a slope down than any such information above.
    if (have_candidate_below && virtual_above && virtual_below)
    {
      return below;
    }
  }

  // When both above and below have valid candidates. We prefer the lower one if there is sufficient clearance from
  // it to the higher one (should be optimistic). Otherwise we prefer the one which has had less searching.
  if (have_candidate_below && (!have_candidate_above || offset_below <= offset_above ||
                               have_candidate_below && have_candidate_above && !virtual_above &&
                                 offset_below + offset_above >= clearance_voxel_count_permissive))
  {
    return below;
  }

  return above;
}


bool findGround(GroundCandidate &ground, SrcVoxel &voxel, const Key &seed_key, const Key &min_key, const Key &max_key,
                const HeightmapDetail &imp)
{
  PROFILE(findGround);
  ground.invalidate();  // Invalidate the result.
  bool observed_above = false;
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
    // Even if we have insufficient clearnace, we can flag that we've made an observation above the ground voxel
    // so long as we have encountered a non-null/non-unobserved voxel.
    // Note we will set this on the first valid ground candidate we encounter, but it will be immediately cleared.
    observed_above = voxel_type != ohm::kNull && voxel_type != ohm::kUnobserved;
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
        observed_above = false;
      }
      else
      {
        // Branch condition only for the first voxel in column.
        ground_key = key;
        column_height = column_clearance_height = height;
        column_voxel_pos = sub_voxel_pos;
        candidate_voxel_type = voxel_type;
        observed_above = false;
      }
    }

    last_voxel_type = voxel_type;
  }

  // Did we find a valid candidate?
  if (candidate_voxel_type != ohm::kNull)
  {
    ground.key = ground_key;
    ground.height = height;
    ground.clearance = column_clearance_height - column_height;
    ground.observed_above = observed_above;
    return true;
  }

  return false;
}


void filterVirtualVoxels(ohm::HeightmapDetail &detail, unsigned threshold,
                         const std::unordered_map<ohm::Key, HeightmapKeyType> &src_to_heightmap_keys)
{
  PROFILE(filterVirtualVoxels);

  const std::array<glm::ivec3, 26> neighbour_offsets = {
    glm::ivec3(-1, -1, -1), glm::ivec3(0, -1, -1), glm::ivec3(1, -1, -1),  //
    glm::ivec3(-1, 0, -1),  glm::ivec3(0, 0, -1),  glm::ivec3(1, 0, -1),   //
    glm::ivec3(-1, 1, -1),  glm::ivec3(0, 1, -1),  glm::ivec3(1, 1, -1),   //

    glm::ivec3(-1, -1, 0),  glm::ivec3(0, -1, 0),  glm::ivec3(1, -1, 0),  //
    glm::ivec3(-1, 0, 0),   glm::ivec3(1, 0, 0),                          //
    glm::ivec3(-1, 1, 0),   glm::ivec3(0, 1, 0),   glm::ivec3(1, 1, 0),   //

    glm::ivec3(-1, -1, 1),  glm::ivec3(0, -1, 1),  glm::ivec3(1, -1, 1),  //
    glm::ivec3(-1, 0, 1),   glm::ivec3(0, 0, 1),   glm::ivec3(1, 0, 1),   //
    glm::ivec3(-1, 1, 1),   glm::ivec3(0, 1, 1),   glm::ivec3(1, 1, 1)    //
  };

#ifdef TES_ENABLE
  KeyRange heightmap_range;
  detail.occupancy_map->calculateExtents(nullptr, nullptr, &heightmap_range);
#endif  // TES_ENABLE

  ohm::OccupancyMap &heightmap = *detail.heightmap;
  Voxel<float> occupancy_voxel(&heightmap, heightmap.layout().occupancyLayer());
  Voxel<HeightmapVoxel> heightmap_voxel(&heightmap, detail.heightmap_voxel_layer);
  assert(occupancy_voxel.isLayerValid());
  assert(heightmap_voxel.isLayerValid());
  // Walk each key in src_to_heightmap_keys looking for virtual voxels. We'll then check it's neighbour keys for
  // presence in the same map.
  for (const auto &key_mapping : src_to_heightmap_keys)
  {
    const Key &src_key = key_mapping.first;
    const HeightmapKeyType &hm_key_type = key_mapping.second;

    if (hm_key_type.type == HeightmapVoxelType::kVirtualSurface)
    {
      unsigned n_count = 0;  // Populated neighbour count.
      // We have a voxel to check.
      // Lookup it's neighbours counting how many are present in src_to_heightmap_keys.
      for (const auto &n_offset : neighbour_offsets)
      {
        Key n_key = src_key;
        heightmap.moveKey(n_key, n_offset);
        if (src_to_heightmap_keys.find(n_key) != src_to_heightmap_keys.end())
        {
          ++n_count;
        }
      }

      // Mark for removal?
      if (n_count < threshold)
      {
        // Mark the voxel as invalid. To be removed during sorting.
        ohm::setVoxelKey(hm_key_type.key, occupancy_voxel, heightmap_voxel);
        assert(occupancy_voxel.isValid());
        assert(heightmap_voxel.isValid());
        auto hmv = heightmap_voxel.data();
        hmv.layer = kHvlInvalid;
        heightmap_voxel.write(hmv);
        // Invalidate the voxel occupancy value.
        occupancy_voxel.write(ohm::unobservedOccupancyValue());

#ifdef TES_ENABLE
        if (g_tes)
        {
          // Remove from debug visualisation.
          uint32_t voxel_id = DstVoxel::get3esVoxelId(hm_key_type.key, *detail.heightmap, detail.vertical_axis_index);
          const tes::Id box_id(voxel_id, kTcHmVirtualSurface);
          g_tes->destroy(tes::Box(box_id));  // Only the ID matters for destruction/removal.
          // Also remove any clearance ray.
          if (hmv.clearance > 0)
          {
            const tes::Id arrow_id(voxel_id, kTcHmClearance);
            g_tes->destroy(tes::Arrow(arrow_id));
          }
          TES_SERVER_UPDATE(g_tes, 0.0f);
        }
#endif  // TES_ENABLE
      }
    }
  }
}


void sortHeightmapLayers(ohm::HeightmapDetail &detail, const std::set<ohm::Key> &target_keys, const bool use_voxel_mean,
                         const double *seed_height)
{
  PROFILE(sortHeightmapLayers);

  if (!target_keys.empty())
  {
    // We have work to do.

    /// Structure used to extract heightmap data for sorting. Contains all information possible from the heightmap.
    struct SortingVoxel
    {
      double height = 0;
      HeightmapVoxel height_info{};
      float occupancy = 0;
      VoxelMean mean{};
      bool base_layer_candidate = false;

      /// Sorting operator.
      inline bool operator<(const SortingVoxel &other) const { return height < other.height; }
    };

    ohm::OccupancyMap &heightmap = *detail.heightmap;
    DstVoxel voxel(heightmap, detail.heightmap_voxel_layer, use_voxel_mean);
    // Will only ever be small.
    std::vector<SortingVoxel> sorting_list;

    for (const auto &base_key : target_keys)
    {
      Key key = base_key;
      // True if we've found a voxel with a height value within the height_filter range.
      // If not, we'll have to add one with a kHeightmapVacantValue.
      voxel.setKey(key);
      assert(voxel.occupancy.isValid() && voxel.heightmap.isValid() && (!use_voxel_mean || voxel.mean.isValid()));
      sorting_list.clear();

      while (voxel.occupancy.isValid() && voxel.occupancy.data() != ohm::unobservedOccupancyValue())
      {
        // Extract voxel data.
        const HeightmapVoxel hmv = voxel.heightmap.data();
        // Skip kHvlInvalid voxels, thereby removing them.
        if (hmv.layer != kHvlInvalid)
        {
          sorting_list.emplace_back();
          SortingVoxel &sorting_info = sorting_list.back();

          sorting_info.occupancy = voxel.occupancy.data();
          sorting_info.height_info = hmv;
          sorting_info.mean = use_voxel_mean ? voxel.mean.data() : VoxelMean{};
          sorting_info.base_layer_candidate = (sorting_info.height_info.layer == kHvlBaseLayer);
          // HACK: testing some new base layer conditions.
          sorting_info.base_layer_candidate = hmv.clearance > 0;// || (hmv.flags & kHvfObservedAbove) != 0;

          // The height value is stored as a relative to the centre of the voxel in which it resides. We need to
          // convert this to an absolute height.
          sorting_info.height =
            heightmap::getVoxelHeight(heightmap, detail.up, key, double(sorting_info.height_info.height));
        }

        heightmap.moveKeyAlongAxis(key, detail.vertical_axis_index, 1);
        voxel.setKey(key);
      }

      // Enter sort/rebuild block if we have a list to sort or we have just the vacant voxel we've artificially added.
      if (sorting_list.size() > 1)
      {
        // Sort the voxels
        key = base_key;
        std::sort(sorting_list.begin(), sorting_list.end());

        /// Current best base layer candidate.
        BaseLayerCandidate best_base_candidate;
        // Key base_layer_key(nullptr);  // Key identifying the base layer.
        // double base_layer_height = 0;
        // Now write back the sorted results.
        // At the same time we finalise the base layer.
        for (SortingVoxel voxel_info : sorting_list)
        {
          voxel.setKey(key);

          assert(voxel.occupancy.isValid() && voxel.heightmap.isValid() && (!use_voxel_mean || voxel.mean.isValid()));
          // Now that we have a new voxel key, we need to convert the HeightmapVoxel heigth value to be relative to
          // the new voxel centre.
          voxel_info.height_info.height =
            float(voxel_info.height - glm::dot(detail.up, detail.heightmap->voxelCentreGlobal(key)));

          // Only one item can be in the base layer. Track the best candidate.
          if (voxel_info.base_layer_candidate)
          {
            const BaseLayerCandidate current_base_candidate(key, voxel_info.height_info, voxel_info.height);

            // Check if this is the best candidate.
            if (best_base_candidate.isOtherCandidateBetter(current_base_candidate, seed_height))
            {
              // We have found either the first candidate, or a better candidate.
              best_base_candidate = current_base_candidate;
            }
          }

          // We always write kHvlExtended here. The base layer is finalised after the loop.
          voxel_info.height_info.layer = kHvlExtended;

          voxel.occupancy.write(voxel_info.occupancy);
          voxel.heightmap.write(voxel_info.height_info);
          if (use_voxel_mean)
          {
            voxel.mean.write(voxel_info.mean);
          }
          heightmap.moveKeyAlongAxis(key, detail.vertical_axis_index, 1);
        }

        // Finalise the base layer.
        if (best_base_candidate.isValid())
        {
          voxel.setKey(best_base_candidate.key);
          auto height_info = voxel.heightmap.data();
          height_info.layer = kHvlBaseLayer;
          voxel.heightmap.write(height_info);
        }
      }
    }
  }
}
}  // namespace heightmap
}  // namespace ohm
