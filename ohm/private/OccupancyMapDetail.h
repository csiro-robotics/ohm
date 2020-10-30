//
// Author: Kazys Stepanas
//
#ifndef OHM_MAPDETAIL_H
#define OHM_MAPDETAIL_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

#include "ohm/MapChunk.h"
#include "ohm/MapFlag.h"
#include "ohm/MapInfo.h"
#include "ohm/MapLayout.h"
#include "ohm/MapRegion.h"
#include "ohm/Mutex.h"
#include "ohm/RayFilter.h"

#include <ohmutil/VectorHash.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#endif  // __GNUC__
#include <ska/bytell_hash_map.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif  // __GNUC__

#include <mutex>
#include <unordered_map>
#include <vector>

namespace ohm
{
using ChunkMap = ska::bytell_hash_map<glm::i16vec3, MapChunk *, Vector3Hash<glm::i16vec3>>;

class MapRegionCache;
class OccupancyMap;

/// Internal details associated with an @c OccupancyMap .
struct ohm_API OccupancyMapDetail
{
  /// Mutex type to use to protect map access.
  using Mutex = ohm::Mutex;

  /// A global origin offset for data in the map. All data read from the map has this origin added.
  glm::dvec3 origin = glm::dvec3(0);
  /// The spatial dimensions of each region. Calculated as `region_voxel_dimensions * resolution`.
  /// Each axis may have a different spatial length.
  glm::dvec3 region_spatial_dimensions = glm::dvec3(0);
  /// The voxel dimensions of each region - i.e., the number of voxels along each axis for a map region.
  /// Each axis may have a different voxel length.
  glm::u8vec3 region_voxel_dimensions = glm::u8vec3(0);
  /// The size of a voxel cube edge. All voxels are uniform cubes.
  double resolution = 0.0;
  /// Used to mark changes in the map. This is a monotonic value which is modified when the map is changed and is
  /// copied into associated @c MapChunk objects. This can be used to detect the recency of changes.
  uint64_t stamp = 0;
  /// The value threshold used to consider a voxel as occupied. Occupied voxels have a value equal to or greater than
  /// this value, but not equal to @c ohm::unobservedOccupancyValue() (infinity).
  /// @see @c ohm::valueToProbability()
  float occupancy_threshold_value = 0.0f;
  /// The value adjustment made to voxels in which samples fall (hits). Must be > 0
  float hit_value = 0.0f;
  /// The value adjustment made to voxels along rays leading up to sample points (misses). Must be < 0
  float miss_value = 0.0f;
  /// The maximum value clamp for a voxel value (excluding @c ohm::unobservedOccupancyValue() (infinity)).
  /// Zero to disable.
  float min_voxel_value = 0.0f;
  /// The minimum value clamp for a voxel value.
  /// Zero to disable.
  float max_voxel_value = 0.0f;
  /// Flag indicating voxels become locked and cannot be changed when they reach @c min_voxel_value .
  bool saturate_at_min_value = false;
  /// Flag indicating voxels become locked and cannot be changed when they reach @c max_voxel_value .
  bool saturate_at_max_value = false;
  /// Map control flags.
  MapFlag flags = MapFlag::kNone;
  /// The voxel memory layout information for the map.
  MapLayout layout;
  /// The hash map of @c MapChunk objects contained in this map.
  ChunkMap chunks;
  /// Data access mutex. Used to protect @c chunks .
  mutable Mutex mutex;
  // Region count at load time. Useful when only the header is loaded.
  size_t loaded_region_count = 0;

  /// GPU cache pointer. Note: this is declared here, but implemented in a dependent library. We simply ensure that
  /// the map detail supports a GPU cache.
  ///
  /// @todo Use @c std::unique_ptr
  MapRegionCache *gpu_cache = nullptr;

  /// Optional function to be called for each input ray before processing. See @c RayFilterFunction documentation.
  RayFilterFunction ray_filter;

  /// Meta information storage about the map.
  /// The data stored are arbitratry key/value pairs. Generally it is expected that this may hold data about how
  /// the map was generated or has been modified.
  MapInfo info;

  /// Default constructor.
  OccupancyMapDetail() = default;
  /// Destructor ensures @c gpu_cache is destroyed.
  ~OccupancyMapDetail();

  /// Move an @c Key along a selected axis.
  /// This is the implementation to @c OccupancyMap::moveKeyAlongAxis(). See that function for details.
  /// @param key The key to adjust.
  /// @param axis Axis ID to move along [0, 2].
  /// @param step How far to move/step.
  void moveKeyAlongAxis(Key &key, int axis, int step) const;  // NOLINT(google-runtime-references)

  /// Setup the default @c MapLayout: occupancy layer and clearance layer.
  /// @param enable_voxel_mean Enable voxel mean positioning?
  void setDefaultLayout(bool enable_voxel_mean = false);

  /// Copy internal details from @p other. For cloning.
  /// @param other The map detail to copy from.
  void copyFrom(const OccupancyMapDetail &other);
};
}  // namespace ohm

#endif  // OHM_MAPDETAIL_H
