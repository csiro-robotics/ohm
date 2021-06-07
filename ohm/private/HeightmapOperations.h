// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_HEIGHTMAPOPERATIONS_H_
#define OHM_HEIGHTMAPOPERATIONS_H_

#include "OhmConfig.h"

#include <ohm/HeightmapVoxel.h>
#include <ohm/HeightmapVoxelType.h>
#include <ohm/Key.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/UpAxis.h>
#include <ohm/VoxelData.h>

#include <glm/vec3.hpp>

#include <set>
#include <unordered_map>

namespace ohm
{
struct HeightmapDetail;

namespace heightmap
{
#ifdef TES_ENABLE
extern const uint32_t kNeighbourIdMask;
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

  bool haveRecordedHeight(double height, int up_axis_index, const glm::dvec3 &up) const;

  /// Generate a debug voxel ID for a key in the @p heightmap for use with 3es.
  /// @param heightmap_key The voxel key in the @p heightmap .
  /// @param heightmap The heightmap object.
  /// @param up_axis_index The index of the up axis {0, 1, 2} mapping to {X, Y, Z}.
  static uint32_t get3esVoxelId(const Key &heightmap_key, const OccupancyMap &heightmap, int up_axis_index);

  /// @overload
  uint32_t get3esVoxelId(int up_axis_index) { return get3esVoxelId(occupancy.key(), *occupancy.map(), up_axis_index); }

  void debugDraw(int level, int up_axis_index, double up_scale = 1.0);
};

/// Calculate the height of voxel @p heightmap_key in the @p heightmap relative to the centre of the voxel.
/// @param absolute_height The absolute height value.
/// @param heightmap_key Identifies the voxel of interest.
/// @param heightmap The heightmap.
/// @param up The up vector in the heightmap.
/// @return The @c absolute_height converted to be relative to the centre of the voxel at @p heightmap_key .
inline float relativeVoxelHeight(double absolute_height, const Key &heightmap_key, const OccupancyMap &heightmap,
                                 const glm::dvec3 &up)
{
  const glm::dvec3 voxel_centre = heightmap.voxelCentreGlobal(heightmap_key);
  const float relative_height = float(absolute_height - glm::dot(voxel_centre, up));
  return relative_height;
}

/// Calculate the absolute height of a voxel in a source occupancy map - before heightmap coversion. This essentially
/// takes the 3D position of @p voxel and extracts the height along the @p up axis.
/// @param[out] voxel_position The 3D position calculated for @p voxel . May include @c VoxelMean if available.
/// @param[out] height The height of @p voxel_position along the @p up axis.
/// @param voxel The voxel details in the source occupancy map from which we are generating a heightmap.
/// @param up The heightmap up axis.
/// @return The @c OccupancyType of the source @p voxel .
OccupancyType sourceVoxelHeight(glm::dvec3 *voxel_position, double *height, SrcVoxel &voxel, const glm::dvec3 &up);

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
                                int step_limit, bool search_up, unsigned flags, int *offset, bool *is_virtual);

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
Key findNearestSupportingVoxel(SrcVoxel &voxel, const Key &seed_key, UpAxis up_axis, const Key &min_key,
                               const Key &max_key, int voxel_floor_limit, int voxel_ceiling_limit,
                               int clearance_voxel_count_permissive, unsigned flags);

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
               const Key &max_key, const HeightmapDetail &imp);


/// A pairing of heightmap voxel key and heightmap voxel type.
struct HeightmapKeyType
{
  Key key;
  HeightmapVoxelType type;
};

/// Filter a layered heightmap removing virtual voxels which have fewer occupied 26-connected neighbours than
/// @p threshold . The call assumes that the check for layered heightmap mode has already been made.
///
/// @param detail Target @c Heightmap private detail.
/// @param threshold The number of non-empty neighbours required to retain a virtual voxel.
/// @param multi_layer_keys The set of heightmap keys which contain multiple voxels in the column.
/// @param src_to_heightmap_keys A mapping of source map key to heightmap voxel key and type.
void filterVirtualVoxels(ohm::HeightmapDetail &detail, unsigned threshold,
                         const std::unordered_map<ohm::Key, HeightmapKeyType> &src_to_heightmap_keys);

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
void sortHeightmapLayers(ohm::HeightmapDetail &detail, const std::set<ohm::Key> &target_keys,
                         const bool use_voxel_mean);

}  // namespace heightmap
}  // namespace ohm

#endif  // OHM_HEIGHTMAPOPERATIONS_H_
