// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELUTIL_H
#define VOXELUTIL_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

#include <limits>
#include <type_traits>

namespace ohm
{
  class Key;
  struct OccupancyMapDetail;
  struct MapChunk;

  /// Occupancy layer voxel layout when using sub-voxel patterns.
  /// Without, it is just the @c float occupancy value.
  struct OccupancyVoxel
  {
    /// Voxel occupancy.
    float occupancy;
    /// Sub-voxel pattern.
    uint32_t sub_voxel;
  };

  /// Contains functions which help manipulate common voxel data.
  namespace voxel
  {
    /// Get the address of the voxel referenced by @p key in @p layer_index.
    ///
    /// This function function decodes accesses the voxel memory for @p layer_index in @p chunk and returns a pointer
    /// to the voxel referenced by @p key. There are several expectations on this method, or it will fail, returning
    /// null:
    /// - @p chunk and @p map are non null.
    /// - @p key references the @p chunk (see @c Key::regionCoord() and @c MapRegion::coord)
    /// - @p expected_size is either zero (no validation) or exactly matches the voxel size.
    ///
    /// Some additional assumptions are made which the caller *must* ensure or results are undefined.
    /// - @p layer_index is valid in the map's @c MapLayout.
    /// - @p key.localKey() is in range of the map's region dimensions.
    ///
    /// This function operated on a mutable @c chunk and returns a non-const pointer.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @param expected_size Optional voxel size validation. Fails if voxels do not match this size. Zero to skip
    ///     validation.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    uint8_t *voxelPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map, int layer_index,
                      size_t expected_size);

    /// Get the address of the voxel referenced by @p key in @p layer_index.
    ///
    /// This function overload deals with non-mutable @p chunk pointers.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @param expected_size Optional voxel size validation. Fails if voxels do not match this size. Zero to skip
    ///     validation.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    const uint8_t *voxelPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map, int layer_index,
                            size_t expected_size);

    /// The address of the voxel referenced by @p key as type @c T.
    ///
    /// This function has the same requirements as @c voxelPtr(), however the voxel size is validated against the
    /// @p sizeof(T).
    ///
    /// This overload requires @c T is a pointer type.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    template <typename T>
    T voxelPtrAs(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map, int layer_index)
    {
      return reinterpret_cast<T>(voxelPtr(key, chunk, map, layer_index, sizeof(typename std::remove_pointer<T>::type)));
    }

    /// The address of the voxel referenced by @p key as type @c T.
    ///
    /// This function has the same requirements as @c voxelPtr(), however the voxel size is validated against the
    /// @p sizeof(T).
    ///
    /// This overload requires @c T is a constant pointer type.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    template <typename T>
    T voxelPtrAs(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map, int layer_index)
    {
      return reinterpret_cast<T>(voxelPtr(key, chunk, map, layer_index, sizeof(typename std::remove_pointer<T>::type)));
    }

    const float *voxelOccupancyPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map);
    float *voxelOccupancyPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map);

    const uint32_t *subVoxelPatternPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map);
    uint32_t *subVoxelPatternPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map);

    bool subVoxelOccupancyFilter(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map);

    /// A helper function for accessing the occupancy threshold of @p map.
    /// @return The equivalent to @c OccupancyMap::occupancyThreshold().
    float occupancyThreshold(const OccupancyMapDetail &map);

    /// A helper function for accessing the voxel dimensions of each region in @p map.
    /// @return The equivalent to @c OccupancyMap::regionVoxelDimensions().
    glm::u8vec3 regionVoxelDimensions(const OccupancyMapDetail &map);

    /// Retrieve the coordinates for the centre of the voxel identified by @p key, local to the map origin.
    /// @param key The voxel of interest.
    /// @param map Internal details of the @c OccupancyMap of interest.
    /// @return The voxel coordinates, relative to the @c OccupancyMap::origin().
    glm::dvec3 centreLocal(const Key &key, const OccupancyMapDetail &map);

    /// Retrieve the global coordinates for the centre of the voxel identified by @p key. This includes the
    /// @c OccupancyMap::origin().
    /// @param key The voxel of interest.
    /// @param map Internal details of the @c OccupancyMap of interest.
    /// @return The global voxel coordinates.
    glm::dvec3 centreGlobal(const Key &key, const OccupancyMapDetail &map);

    /// Retrieves the (global) position of a voxel with consideration to sub-voxel positioning.
    /// This is equivalent to @c centreGlobal() if sub-voxel positioning is not enabled, or not resolved for the voxel.
    /// @param key The voxel of interest.
    /// @param map Internal details of the @c OccupancyMap of interest.
    /// @return The global voxel coordinates with sub-voxel positioning.
    glm::dvec3 position(const Key &key, const MapChunk &chunk, const OccupancyMapDetail &map);

    /// Checks if the given @c map has sub-voxel occupancy filtering enabled. Voxel reference must be valid.
    /// @return True if sub-voxel occupancy filtering is enabled.
    /// @see @c subVoxelOccupancyFilter()
    bool subVoxelOccupancyFilterEnabled(const OccupancyMapDetail &map);

    /// Value used to identify invalid or uninitialised voxel voxels.
    /// @return A numeric value considered invalid for a voxel value.
    constexpr float invalidMarkerValue() { return std::numeric_limits<float>::infinity(); }
  }  // namespace voxel
}  // namespace ohm

#endif  // VOXELUTIL_H
