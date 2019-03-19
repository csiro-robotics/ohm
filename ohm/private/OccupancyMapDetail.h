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
#include "ohm/RayFilter.h"

#include <ohmutil/VectorHash.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#endif // __GNUC__
#include <ohmutil/ska/bytell_hash_map.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif // __GNUC__

#include <mutex>
#include <unordered_map>
#include <vector>

namespace ohm
{
  using ChunkMap = ska::bytell_hash_map<glm::i16vec3, MapChunk *, Vector3Hash<glm::i16vec3>>;

  class MapRegionCache;
  class OccupancyMap;

  struct ohm_API OccupancyMapDetail
  {
    // "sub_voxel"
    static const char *kSubVoxelLayerName;

    glm::dvec3 origin = glm::dvec3(0);
    glm::dvec3 region_spatial_dimensions = glm::dvec3(0);
    glm::u8vec3 region_voxel_dimensions = glm::u8vec3(0);
    double resolution = 0.0;
    double sub_voxel_weighting = 0.3;
    uint64_t stamp = 0;
    float occupancy_threshold_value = 0.0f;
    float occupancy_threshold_probability = 0.0f;
    float hit_value = 0.0f;
    float hit_probability = 0.0f;
    float miss_value = 0.0f;
    float miss_probability = 0.0f;
    float min_voxel_value = 0.0f;
    float max_voxel_value = 0.0f;
    float sub_voxel_filter_scale = 1.0f;
    bool saturate_at_min_value = false;
    bool saturate_at_max_value = false;
    MapLayout layout;
    ChunkMap chunks;
    mutable std::mutex mutex;
    // Region count at load time. Useful when only the header is loaded.
    size_t loaded_region_count = 0;
    MapFlag flags = MapFlag::None;

    /// GPU cache pointer. Note: this is declared here, but implemented in a dependent library. We simply ensure that
    /// the map detail supports a GPU cache.
    MapRegionCache *gpu_cache = nullptr;

    RayFilterFunction ray_filter;

    MapInfo info;  ///< Meta information storage about the map.

    ~OccupancyMapDetail();

    /// Move an @c Key along a selected axis.
    /// This is the implementation to @c OccupancyMap::moveKeyAlongAxis(). See that function for details.
    /// @param key The key to adjust.
    /// @param axis Axis ID to move along [0, 2].
    /// @param step How far to move/step.
    void moveKeyAlongAxis(Key &key, int axis, int step) const;

    /// Setup the default @c MapLayout: occupancy layer and clearance layer.
    /// @param enable_sub_voxel_positioning Enable the sub_voxel positioning information?
    void setDefaultLayout(bool enable_sub_voxel_positioning = false);

    /// Copy internal details from @p other. For cloning.
    /// @param other The map detail to copy from.
    void copyFrom(const OccupancyMapDetail &other);
  };
}  // namespace ohm

#endif  // OHM_MAPDETAIL_H
