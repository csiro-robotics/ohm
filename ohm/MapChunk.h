// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPCHUNK_H
#define OHM_MAPCHUNK_H

#include "OhmConfig.h"

#include "MapRegion.h"
#include "Key.h"

#include <atomic>

namespace ohm
{
  class MapLayout;

  /// Convert a 3D index into a @c MapChunk into a linear index into the chunk's voxels.
  /// @param x The index into the chunk along the X axis.
  /// @param y The index into the chunk along the Y axis.
  /// @param z The index into the chunk along the Z axis.
  /// @param dimx The number of voxels in a @c MapChunk along the X axis.
  /// @param dimy The number of voxels in a @c MapChunk along the Y axis.
  /// @param dimz The number of voxels in a @c MapChunk along the Z axis.
  inline unsigned voxelIndex(unsigned x, unsigned y, unsigned z, unsigned dimx, unsigned dimy, unsigned /*dimz*/)
  {
    return x + y * dimx + z * dimx * dimy;
  }


  /// @overload
  inline unsigned voxelIndex(const glm::u8vec3 &key, const glm::ivec3 &dim)
  {
    return key.x + key.y * dim.x + key.z * dim.x * dim.y;
  }


  /// @overload
  inline unsigned voxelIndex(const OccupancyKey &key, const glm::ivec3 &dim)
  {
    return key.localKey().x + key.localKey().y * dim.x + key.localKey().z * dim.x * dim.y;
  }


  /// Internal representation of a section of the map.
  ///
  /// A covers a contiguous, voxel region within the map. This structure associated
  /// a @c MapRegion with a set voxel occupancy and clearance values (one each per voxel).
  /// A coarse clearance array is also maintained, which is essentially a set of lower resolution
  /// voxel representation of voxel @c clearance values.
  ///
  /// Check chunk contains an array of @c voxel_maps, containing at least one element. The first element
  /// is always the occupancy probability map, forming a contiguous block of memory to densely represent the
  /// voxels in the chunk. Other elements are user defined, but the common set is:
  /// - occupancy
  /// - clearance values
  /// - coarse clearance
  ///
  /// The clearance values are 1-1 with the occupancy, while the coarse clearance map is a downsampled version of the
  /// clearance map. Details of the available maps is stored in a @c MapLayout.
  struct MapChunk
  {
    /// Defines the spatial region covered by the chunk.
    MapRegion region;
    /// Describes the layers and voxel layout of the chunk (from the map as a whole).
    const MapLayout *layout = nullptr;
    /// Index of the first voxel with valid data: occupied or free, but not uncertain.s
    glm::u8vec3 first_valid_index = glm::u8vec3(255, 255, 255);
    /// Last timestamp the occupancy layer of this chunk was modified.
    double touched_time = 0;

    /// A monotonic stamp value occupancy layer, used to indicate when this layer or a neighbour was last updated.
    /// The map maintains the most up to date stamp: @c OccupancyMap::stamp().
    uint64_t dirty_stamp = 0;

    /// A monotonic stamp value for each @c voxelMap, used to indicate when the layer was last updated.
    /// The map maintains the most up to date stamp: @c OccupancyMap::stamp().
    std::atomic_uint64_t *touched_stamps = nullptr;

    /// Array of voxel maps. Semantics are defined in the owning @c OccupancyMap.
    /// Use @c layout to access specific maps.
    uint8_t **voxel_maps = nullptr;

    /// Chunk flags set from @c MapChunkFlag.
    unsigned flags;

    MapChunk() = default;
    MapChunk(const MapLayout &layout, const glm::uvec3 &region_dim);
    MapChunk(const MapRegion &region, const MapLayout &layout, const glm::uvec3 &region_dim);
    MapChunk(const MapChunk &other) = default;
    MapChunk(MapChunk &&other) noexcept;
    ~MapChunk();

    /// Given a @p voxelIndex into voxels, get the associated @c OccupancyKey.
    /// @param voxel_index An index into voxels. Must be in range
    ///   <tt>[0, regionVoxelDimensions.x * regionVoxelDimensions.y * regionVoxelDimensions.z)</tt>
    ///   or a null key is returned.
    /// @param region_voxel_dimensions The dimensions of each chunk/region along each axis.
    /// @param region_coord The coordinate of the containing region.
    /// @return An @c OccupancyKey to reference the requested voxel.
    static OccupancyKey keyForIndex(size_t voxel_index, const glm::ivec3 &region_voxel_dimensions,
                                    const glm::i16vec3 &region_coord);

    /// @overload
    inline OccupancyKey keyForIndex(size_t voxel_index, const glm::ivec3 &region_voxel_dimensions) const
    {
      return keyForIndex(voxel_index, region_voxel_dimensions, region.coord);
    }

    /// Returns true if the chunk contains any valid voxels. A valid voxel is one who's value has
    /// been set.
    ///
    /// This is a quick test based on the state of @c first_valid_index being valid and not
    /// (255, 255, 255). Thus the result is only corect insofar as @c first_valid_index is correctly
    /// maintained.
    ///
    /// @return True if this chunk contains at least one voxel with a valid value.
    bool hasValidNodes() const;

    /// Update the @c first_valid_index based on adding @p localIndex as a valid index.
    /// This simply chooses whichever occurs first from @c first_valid_index and
    /// @p localIndex.
    /// @param local_index The voxel index within this chunk. Equivalent to Key::localKey().
    /// @param region_voxel_dimensions The dimensions of each chunk/region along each axis.
    void updateFirstValid(const glm::u8vec3 &local_index, const glm::ivec3 &region_voxel_dimensions);

    /// Update the @c first_valid_index by brute force, searching for the first valid voxel.
    /// @param region_voxel_dimensions The dimensions of each chunk/region along each axis.
    /// @param search_from Start searching from this voxel index (must be a valid index).
    void searchAndUpdateFirstValid(const glm::ivec3 &region_voxel_dimensions,
                                   const glm::u8vec3 &search_from = glm::u8vec3(0, 0, 0));

    /// Recalculates what the @c first_valid_index should be (brute force) and validates against its current value.
    /// @return True when the @c first_valid_index value matches what it should be.
    bool validateFirstValid(const glm::ivec3 &region_voxel_dimensions) const;

    bool overlapsExtents(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext,
                         const glm::dvec3 &region_spatial_dimensions) const;

    void extents(glm::dvec3 &min_ext, glm::dvec3 &max_ext, const glm::dvec3 &region_spatial_dimensions) const;
  };
}

#endif  // OHM_MAPCHUNK_H
