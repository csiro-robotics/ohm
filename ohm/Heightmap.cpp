// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Heightmap.h"

#include "private/HeightmapDetail.h"

#include "Aabb.h"
#include "CovarianceVoxel.h"
#include "DefaultLayer.h"
#include "HeightmapUtil.h"
#include "HeightmapVoxel.h"
#include "Key.h"
#include "MapChunk.h"
#include "MapCoord.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "OccupancyType.h"
#include "PlaneFillLayeredWalker.h"
#include "PlaneFillWalker.h"
#include "PlaneWalker.h"
#include "Trace.h"
#include "VoxelData.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

// Include after GLM types for glm type streaming operators.
#include "ohmutil/GlmStream.h"

#define PROFILING 1
#include <ohmutil/Profile.h>

#include <3esservermacros.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

// Enable code to support breaking on a specific voxel.
#define HM_DEBUG_VOXEL 1

namespace ohm
{
namespace heightmap
{
#ifdef TES_ENABLE
const uint32_t neighbour_id_mask = 0x80000000u;
#endif  // TES_ENABLE

/// Flags for @c findNearestSupportingVoxel() calls
enum SupportingVoxelFlag : unsigned
{
  /// Allow virtual surface candidate voxels to be reported as potential ground candidates.
  ///
  /// A virtual surface candidate is a free voxel supported by an unobserved voxel.
  kVirtualSurfaces = (1u << 0u),
  /// When selecting a candidate voxel from one above and one below and two candidates are found at the same range,
  /// then bias the selection to the one above.
  ///
  /// This flag is turned when selecting the first voxel in the heightmap, then subsequentyly is turned on for all but
  /// the planar heightmap generation (@c PlaneWalker used). This helps guide fill expansions up sloped surfaces.
  kBiasAbove = (1u << 1u),
  /// Flag used when finding a virtual surface candidate in the lower voxel and an occupied voxel above it. This mode
  /// prefers the virtual voxel candidate. Virtual surfaces must be enabled.
  kPromoteVirtualBelow = (1u << 2u),
  /// Ignore virtual surface candidates above the seed voxel. This generates better heightmaps based on a fixed plane -
  /// using @c PlaneWalker
  kIgnoreVirtualAbove = (1u << 3u),
};

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

  inline DstVoxel() = default;

  DstVoxel(OccupancyMap &map, int heightmap_voxel_layer, bool use_mean)
    : occupancy(&map, map.layout().occupancyLayer())
    , heightmap(&map, heightmap_voxel_layer)
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
      const VoxelMean mean_info = mean.data();
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

  inline bool haveRecordedHeight(double height, int up_axis_index, const glm::dvec3 &up) const
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

  /// Generate a debug voxel ID for a key in the @p heightmap for use with 3es.
  /// @param heightmap_key The voxel key in the @p heightmap .
  /// @param heightmap The heightmap object.
  /// @param up_axis_index The index of the up axis {0, 1, 2} mapping to {X, Y, Z}.
  static uint32_t get3esVoxelId(const Key &heightmap_key, const OccupancyMap &heightmap, int up_axis_index)
  {
    // Generate an ID for the voxel.
    // For this we take the extents fo teh occupancy map (in this case the heightmap) and assign an ID based on the
    // voxel position in that range.
    KeyRange heightmap_range;
    heightmap.calculateExtents(nullptr, nullptr, &heightmap_range);
    // Note: the vertical extents of the heightmap can change for a layered map as we add layers. We cater for this
    // here by ensuring the vertical extents of heightmap_range is padded out to allow for 32 layers. This effectively
    // reserves 5 bits for vertical indexing, 1 bits masking open list voxels, leaving 26 for horizontal indexing. This
    // supports a 2D region of approximately 8Kx8K voxels and should support maps which can reasonably be debugged using
    // 3es. Larger maps create excessively large 3es data sets.
    // Anything more really needs to use the a tes::MutableMesh object.
    Key adjusted_vertical_key(0, 0, 0, 0, 0, 0);
    heightmap.moveKeyAlongAxis(adjusted_vertical_key, up_axis_index, 32);
    heightmap_range.expand(adjusted_vertical_key);
    return uint32_t(heightmap_range.indexOf(heightmap_key));
  }

  /// @overload
  uint32_t get3esVoxelId(int up_axis_index) { return get3esVoxelId(occupancy.key(), *occupancy.map(), up_axis_index); }

  inline void debugDraw(int level, int up_axis_index, double up_scale = 1.0)
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
};

inline float relativeVoxelHeight(double absolute_height, const Key &heightmap_key, const OccupancyMap &heightmap,
                                 const glm::dvec3 &up)
{
  const glm::dvec3 voxel_centre = heightmap.voxelCentreGlobal(heightmap_key);
  const float relative_height = float(absolute_height - glm::dot(voxel_centre, up));
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


/// Calculate the height of the voxel at @p key with matching local @p height value.
///
/// This calculates the centre of the voxel at @p key then calculates `dot(up, centre) + height`.
///
/// @param heightmap The heightmap which @p key references.
/// @param up The up axis for @p heightmap - must be a unit vector aligned with X, Y or Z axes.
/// @param key The heightmap voxel key of interest. The up axis should always be (0, 0) for non-layered heightmaps.
/// @param height The local voxel height delta.
/// @return The global voxel height value.
inline double getVoxelHeight(const OccupancyMap &heightmap, const glm::dvec3 &up, const Key &key, double height)
{
  const glm::dvec3 voxel_centre = heightmap.voxelCentreGlobal(key);
  return glm::dot(up, voxel_centre) + height;
}


/// A secondary operation for @c findNearestSupportingVoxel() which finds the first occupied or virtual voxel in the
/// column of @p from_key . This function can search either up or down from @p from_key until a candidate is found, the
/// @p step_limit number of voxels have been considered or after @p to_key has been considered - whichever condition is
/// met first.
///
/// A valid candidate voxel is one which is occupied or a virtual surface voxel. See the virtual surfaces section of the
/// @c Heightmap class documentation.
///
/// @param voxel Configured to allow access to various aspects of the source map voxels.
/// @param from_key Voxel key identifying where to start the search from. Only voxels in the same column are considered
///   up to @p to_key .
/// @param to_key The final key to consider. This must be in the same column as @p from_key .
/// @param up_axis_index Index of the up axis: 0=>x, 1=>y, 2=>z.
/// @param step_limit Limits the number of voxels to be visited. Zero to visit all voxels in the range
///   `[from_key, to_key]`
/// @param search_up A flag used to indicate if we are searching up a column or not. The sign of the difference between
///   @p from_key and @p to_key cannot be used to infer this as it is defined by the heightmap
///   primary axis, for which up may be alinged with an increasing, negative magnitude.
/// @param flags Flags affecting reporting. Only @c SupportingVoxelFlag::kVirtualSurfaces is considered here. When
///   set, consider virtual surface voxels - free voxels "supported" by unobserved voxels representing the best possible
///   surface estimate.
/// @param[out] offset On success, set the number of voxels between @p from_key and the selected voxel. Undefined on
///   failure.
/// @param[out] is_virtual On success, set to true if the selected voxel is a virtual surface voxel. False when the
///   selected voxel is an occupied voxel. Undefined on failure.
/// @return The first voxel found which is either occupied or a virtual surface voxel when @c kVirtualSurfaces is
///   set. The result is a null key on failure to find such a voxel within the search limits.
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

/// Search the column containing @p seed_key in the source occupancy map for a potential supporting voxel.
///
/// A supporting voxel is one which is either occupied or a virtual surface voxel (if enabled). The source map details
/// are contained in the @c SrcVoxel structure passed via @p voxel . That structure is configured to reference the
/// relevant voxel layers. The actual voxels referenced by @c voxel will be modified by this function, starting at
/// @c seed_key .
///
/// The search process searches above and below @c seed_key - with up defined by @c up_axis - for an occupied or
/// virtual surface voxel. The final voxel selection is guided by several factors:
///
/// - Prefer occupied voxels over virtual surface voxels
///   - Except where @c promote_virtual_below is true
/// - Prefer below to above.
///   - Except where the distance between the candidates below and above is less than
///     @p clearance_voxel_count_permissive .
/// - Limit the search expansion to search up @c voxel_ceiling voxels (this is a voxel count value).
/// - Limit the search down to the map extents.
///
/// The resulting key can be used to identify the voxel from which to start searching for an actual ground candidate
/// with consideration given to clearance above.
///
/// The selected key is expected to be used as the seed for @c findGround() .
///
/// @param voxel Configured to allow access to various aspects of the source map voxels.
/// @param seed_key Voxel key identifying where to start the search from. Only voxels in the same column are considered.
/// @param up_axis Identifies the up axis - XYZ - and direction.
/// @param min_key Min extents limits. Searches are bounded by this value.
/// @param max_key Max extents limits. Searches are bounded by this value.
/// @param voxel_ceiling The number of voxels the function is allowed to consider above the @p seed_key .
/// @param clearance_voxel_count_permissive The number of voxels required to pass the clearance value. Used to
///   discriminate the voxel below when the candidate above is within this range, but non-virtual.
/// @param flags Control flags from @c SupportingVoxelFlag .
/// @return A new seed key from which to start searching for a valid ground voxel (@c findGround()).
inline Key findNearestSupportingVoxel(SrcVoxel &voxel, const Key &seed_key, UpAxis up_axis, const Key &min_key,
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

/// Search for the best ground voxel for the column containing @p seed_key . The search begins at @p seed_key , normally
/// generated by @c findNearestSupportingVoxel() . This function considers the configured
/// @c HeightmapDetail::min_clearance from @p imp and may also consider virtual surface voxels if configured to do so.
///
/// @param[out] height_out Set to the height of the selected ground voxel. This will be based on the voxel mean position
/// (if enabled) for occupied voxels where available, otherwise using the centre of the selected voxel. The value is
/// undefined if the return value is a null key.
/// @param[out] clearance_out Set to the available clearance above the selected voxel if available. A -1 value indicates
/// the height cannot be calculated such as at the limit of the map extents or at the limit of the search extents.
/// Undefined if the result is a null key.
/// @param voxel Configured to allow access to various aspects of the source map voxels.
/// @param seed_key Voxel key identifying where to start the search from. Only voxels in the same column are considered.
/// @param min_key Min extents limits. Searches are bounded by this value.
/// @param max_key Max extents limits. Searches are bounded by this value.
/// @param imp Implementation details of the heightmap object being operated on.
/// @return The first viable ground candidate found from @c seed_key or a null key if no such voxel can be bound.
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
      voxel_id |= neighbour_id_mask;
      const glm::dvec3 pos = imp.occupancy_map->voxelCentreGlobal(nkey);
      tes::Box neighbour(tes::Id(voxel_id, kTcHmVisit), tes::Transform(tes::Vector3d(glm::value_ptr(pos)),
                                                                       tes::Vector3d(imp.heightmap->resolution())));
      neighbour.setColour(tes::Colour::Colours[tes::Colour::CornflowerBlue]).setWireframe(true);
      neighbour.setReplace(true);
      g_tes->create(neighbour);
    }

    // Delete the previous candidate voxel visualisation.
    voxel_id = uint32_t(source_map_range.indexOf(walk_key));
    voxel_id |= neighbour_id_mask;
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

/// A pairing of heightmap voxel key and heightmap voxel type.
struct HeightmapKeyType
{
  Key key;
  HeightmapVoxelType type;
};

/// Filter a layered heightmap removing virtual voxels which have fewer occupied 6-connected neighbours than
/// @p threshold . The call assumes that the check for layered heightmap mode has already been made.
///
/// @param detail Target @c Heightmap private detail.
/// @param threshold The number of non-empty neighbours required to retain a virtual voxel.
/// @param multi_layer_keys The set of heightmap keys which contain multiple voxels in the column.
/// @param src_to_heightmap_keys A mapping of source map key to heightmap voxel key and type.
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


/// A utility function which sorts voxels in a layered heightmap such that lower heights appear lower in the heightmap.
///
/// In a multi layered heightmap, there may be multiple surfaced heights at any 2D voxel coordinate. This function
/// ensures that each 2D block is sorted such that the height value increases. The heightmap generates unsorted results.
/// The @p target_keys identify which voxels need to be sorted. The keys are assumed to address the current bottom voxel
/// of each column only.
///
/// @param detail Target @c Heightmap private detail.
/// @param target_keys keys which contain multiple voxels.
/// @param use_voxel_mean Does the map support voxel mean positioning?
void sortHeightmapLayers(ohm::HeightmapDetail &detail, const std::set<ohm::Key> &target_keys, const bool use_voxel_mean)
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

          // The height value is stored as a relative to the centre of the voxel in which it resides. We need to convert
          // this to an absolute height.
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

        bool have_base_layer = false;  // Track having located the first base layer.
        // Now write back the sorted results.
        for (SortingVoxel voxel_info : sorting_list)
        {
          voxel.setKey(key);

          assert(voxel.occupancy.isValid() && voxel.heightmap.isValid() && (!use_voxel_mean || voxel.mean.isValid()));
          // Now that we have a new voxel key, we need to convert the HeightmapVoxel heigth value to be relative to
          // the new voxel centre.
          voxel_info.height_info.height =
            float(voxel_info.height - glm::dot(detail.up, detail.heightmap->voxelCentreGlobal(key)));

          // Only one item can be in the base layer. Ensure only the first is marked as the base layer item.
          voxel_info.height_info.layer =
            (!have_base_layer && voxel_info.base_layer_candidate) ? kHvlBaseLayer : kHvlExtended;
          have_base_layer = have_base_layer || voxel_info.base_layer_candidate;

          voxel.occupancy.write(voxel_info.occupancy);
          voxel.heightmap.write(voxel_info.height_info);
          if (use_voxel_mean)
          {
            voxel.mean.write(voxel_info.mean);
          }
          heightmap.moveKeyAlongAxis(key, detail.vertical_axis_index, 1);
        }
      }
    }
  }
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
        src_to_heightmap_keys.emplace(ground_key,
                                      heightmap::HeightmapKeyType{ hm_voxel.occupancy.key(), hm_voxel_type });
      }
    }

    TES_SERVER_UPDATE(g_tes, 0.0f);
    supporting_voxel_flags = iterating_supporting_flags;
  } while (!abort && walker.walkNext(walk_key));

  if (ordered_layers)
  {
    if (imp_->virtual_surface_filter_threshold > 0)
    {
      heightmap::filterVirtualVoxels(*imp_, imp_->virtual_surface_filter_threshold, src_to_heightmap_keys);
    }

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
