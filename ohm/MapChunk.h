// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPCHUNK_H
#define OHM_MAPCHUNK_H

#include "OhmConfig.h"

#include "Key.h"
#include "MapRegion.h"
#include "VoxelBlock.h"

#include <algorithm>
#include <atomic>
#include <vector>
#include <utility>

namespace ohm
{
  class MapLayer;
  class MapLayout;
  struct OccupancyMapDetail;

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
  inline unsigned voxelIndex(const Key &key, const glm::ivec3 &dim)
  {
    return key.localKey().x + key.localKey().y * dim.x + key.localKey().z * dim.x * dim.y;
  }


  inline glm::u8vec3 voxelLocalKey(unsigned index, const glm::ivec3 &dim)
  {
    return glm::u8vec3(index % dim.x, (index % (dim.x * dim.y)) / dim.x, index / (dim.x * dim.y));
  }


  /// Move a region local key to the next coordinate in that region. The operation is constrained by the region
  /// dimensions @p dim.
  ///
  /// The key movement is along the full extents of X, then X/Y, then X/Y/Z.
  ///
  /// @param local_key The local_key to adjust.
  /// @param dim The region voxel dimensions.
  /// @return False if the key is out of range or at the limit of the region.
  inline bool nextLocalKey(glm::u8vec3 &local_key, const glm::ivec3 &dim)  // NOLINT(google-runtime-references)
  {
    if (local_key.x + 1 < dim.x)
    {
      ++local_key.x;
      return true;
    }
    if (local_key.y + 1 < dim.y)
    {
      local_key.x = 0;
      ++local_key.y;
      return true;
    }
    if (local_key.z + 1 < dim.z)
    {
      local_key.x = local_key.y = 0;
      ++local_key.z;
      return true;
    }

    return false;
  }


  /// Move a key's local key to reference the next coordinate in that region. The operation is constrained by the
  /// region dimensions @p dim.
  ///
  /// The key movement is along the full extents of X, then X/Y, then X/Y/Z.
  ///
  /// @param key The key to adjust the local coordinates of.
  /// @param dim The region voxel dimensions.
  /// @return False if the key is out of range or at the limit of the region.
  inline bool nextLocalKey(Key &key, const glm::ivec3 &dim)  // NOLINT(google-runtime-references)
  {
    glm::u8vec3 local_key = key.localKey();
    if (nextLocalKey(local_key, dim))
    {
      key.setLocalKey(local_key);
      return true;
    }
    return false;
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
  ///
  /// @par Voxel access
  /// Direct access to the voxel data in @c voxel_maps is by far the fastest option for inner loops (as opposed to the
  /// @c Voxel interface). However, it is a much more manual process and requires more care and process. In general
  /// update logic for writing voxel data should be as follows:
  /// - Resolve the desired layer indices via @c MapLayout and cache these values.
  /// - Validate @c MapLayer size vs the expected size.
  /// - Cache the @c MapLayer::dimensions() for use with @c voxelIndex() calls (below)
  /// - Ensure GPU memory is synched if required - @c GpuMap::syncVoxels()
  /// - Touch the map and cache the value @c OccupancyMap::touch()
  /// - Generate the key of interest
  /// - Request the @c MapChunk using @c Key::regionKey() and @c OccupancyMap::region() - see note below
  /// - Get the memory for the required layer(s) - @c MapChunk::voxel_maps[layer_index]
  /// - Cast the voxel memory to the expected type - e.g., @c float for occupancy, @c VoxelMean for the voxel mean layer
  /// - Resolve the @c Key::localKey() into a one dimensional index using @c voxelIndex()
  /// - Read/write to the indexed voxel as required
  /// - Update the @c MapChunk::dirty_stamp to the cached @c OccupancyMap::touch() value
  /// - Update the @c MapChunk::touched_stamps for the affected layer(s) to the same touch value.
  ///     - Recommend using @c std::atomic_uint64_t::.store() with @c std::memory_order_relaxed if permitted
  ///
  /// This logic avoids constantly querying and validating @c MapLayer details, while ensuring that the
  /// @c OccupancyMap::stamp() , @c MapChunk::dirty_stamp and are @c MapChunk::touched_stamps are correctly managed.
  ///
  /// An additional optimisation can be made by avoiding calls to @c OccupancyMap::region() . Iterative and line walk
  /// style updates of a map have a strong spatial coherency from one voxel to update to the next. In this case, the
  /// @c MapChunk will often be the same as the previous one. In this case, an easy peformance gain comes by keeping
  /// checking if the new @c Key::regionKey() is the same as the last one, and using the previous @c MapChunk when the
  /// @c regionKey() is unchanged.
  struct MapChunk
  {
    /// Defines the spatial region covered by the chunk.
    MapRegion region = MapRegion{};
    /// Details of the map to which this chunk belongs.
    const OccupancyMapDetail *map = nullptr;
    /// Index of the first voxel with valid data: occupied or free, but not unobserved.
    unsigned first_valid_index = ~0u;
    /// Last timestamp the occupancy layer of this chunk was modified.
    double touched_time = 0;

    /// A monotonic stamp value occupancy layer, used to indicate when this layer or a neighbour was last updated.
    /// The map maintains the most up to date stamp: @c OccupancyMap::stamp().
    uint64_t dirty_stamp = 0;

    /// A monotonic stamp value for each @c voxelMap, used to indicate when the layer was last updated.
    /// The map maintains the most up to date stamp: @c OccupancyMap::stamp().
    /// @note It is not possible to have a @c std::vector of atomic types. We use a unique pointer to an arrray
    /// instead.
    std::unique_ptr<std::atomic_uint64_t[]> touched_stamps;

    /// Array of voxel blocks. Layer semantics are defined in the owning @c OccupancyMap.
    /// Use @c layout to access specific maps.
    std::vector<VoxelBlock::Ptr> voxel_blocks;

    /// Chunk flags set from @c MapChunkFlag.
    unsigned flags = 0;

    MapChunk() = default;
    MapChunk(const OccupancyMapDetail &map);
    MapChunk(const MapRegion &region, const OccupancyMapDetail &map);
    MapChunk(const MapChunk &other) = default;
    MapChunk(MapChunk &&other) noexcept;
    ~MapChunk();

    /// Access details of the voxel layers and layouts for this map.
    const MapLayout &layout() const;

    /// Given a @p voxelIndex into voxels, get the associated @c Key.
    /// @param voxel_index An index into voxels. Must be in range
    ///   <tt>[0, regionVoxelDimensions.x * regionVoxelDimensions.y * regionVoxelDimensions.z)</tt>
    ///   or a null key is returned.
    /// @param region_voxel_dimensions The dimensions of each chunk/region along each axis.
    /// @param region_coord The coordinate of the containing region.
    /// @return An @c Key to reference the requested voxel.
    static Key keyForIndex(size_t voxel_index, const glm::ivec3 &region_voxel_dimensions,
                           const glm::i16vec3 &region_coord);

    /// @overload
    inline Key keyForIndex(size_t voxel_index, const glm::ivec3 &region_voxel_dimensions) const
    {
      return keyForIndex(voxel_index, region_voxel_dimensions, region.coord);
    }

    /// Update the @p layout for the chunk, preserving current layers which have an equivalent in @p new_layout.
    ///
    /// Note: this does not change the @p layout pointer as it is assumed the object at that location is about to be
    /// updated with the information from @p new_layout.
    ///
    /// @param new_layout The new memory layout for voxel chunks.
    /// @param region_dim The dimensions of each region (voxels).
    /// @param preserve_layer_mapping Indicates which layers from @p layout are mapped to new layers in @p new_layout.
    void updateLayout(const MapLayout *new_layout, const glm::uvec3 &region_dim,
                      const std::vector<std::pair<const MapLayer *, const MapLayer *>> &preserve_layer_mapping);

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
    inline void updateFirstValid(unsigned index) { first_valid_index = std::min(index, first_valid_index); }

    /// Update the @c first_valid_index by brute force, searching for the first valid voxel.
    /// @param region_voxel_dimensions The dimensions of each chunk/region along each axis.
    /// @param search_from Start searching from this voxel index (must be a valid index).
    void searchAndUpdateFirstValid(const glm::ivec3 &region_voxel_dimensions,
                                   const glm::u8vec3 &search_from = glm::u8vec3(0, 0, 0));

    inline glm::u8vec3 firstValidKey(const glm::ivec3 &region_voxel_dimensions) const
    {
      return voxelLocalKey(first_valid_index, region_voxel_dimensions);
    }

    /// Recalculates what the @c first_valid_index should be (brute force) and validates against its current value.
    /// @return True when the @c first_valid_index value matches what it should be.
    bool validateFirstValid(const glm::ivec3 &region_voxel_dimensions) const;

    bool overlapsExtents(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext,
                         const glm::dvec3 &region_spatial_dimensions) const;

    void extents(glm::dvec3 &min_ext, glm::dvec3 &max_ext,  // NOLINT(google-runtime-references)
                 const glm::dvec3 &region_spatial_dimensions) const;
  };


  inline bool MapChunk::hasValidNodes() const
  {
    return first_valid_index != ~0u;
    // return first_valid_index.x != 255 && first_valid_index.y != 255 && first_valid_index.z != 255;
  }


  inline void MapChunk::updateFirstValid(const glm::u8vec3 &local_index, const glm::ivec3 &region_voxel_dimensions)
  {
    first_valid_index = std::min(voxelIndex(local_index.x, local_index.y, local_index.z, region_voxel_dimensions.x,
                                            region_voxel_dimensions.y, region_voxel_dimensions.z),
                                 first_valid_index);
    // const unsigned current_first =
    //   voxelIndex(first_valid_index.x, first_valid_index.y, first_valid_index.z, region_voxel_dimensions.x,
    //              region_voxel_dimensions.y, region_voxel_dimensions.z);
    // const unsigned new_first = voxelIndex(local_index.x, local_index.y, local_index.z, region_voxel_dimensions.x,
    //                                       region_voxel_dimensions.y, region_voxel_dimensions.z);
    // first_valid_index = (new_first < current_first) ? local_index : first_valid_index;
#ifdef OHM_VALIDATION
    if (test_first < current_first)
    {
      validateFirstValid(regionVoxelDimensions);
    }
#endif  // OHM_VALIDATION
  }


}  // namespace ohm

#endif  // OHM_MAPCHUNK_H
