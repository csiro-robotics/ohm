// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPREGIONCACHE_H
#define OHM_MAPREGIONCACHE_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

namespace ohm
{
struct MapRegionCacheDetail;
struct MapChunk;

/// Base class for any memory cache responsible for ensuring map regions are available in the correct memory.
///
/// Initially this class is simply a placeholder base class for the GPU cache. In future it map be extended to
/// maintain an in-core, uncompressed region cache with compressed data attached to @c MapRegion objects.
class ohm_API MapRegionCache
{
public:
  /// Constructor.
  MapRegionCache();
  /// Virtual destructor.
  virtual ~MapRegionCache();

  /// Reinitialise the cache. Called after restructuring region layout.
  virtual void reinitialise() = 0;

  /// Flush any outstanding operations. For example, the GPU cache will ensure host memory is up to date.
  virtual void flush() = 0;

  /// Clear the cache. May block on in flight operations. Called before restructuring region layout.
  virtual void clear() = 0;

  /// Remove/flush the region matching @p region_coord from the cache.
  /// @param region_coord The region to flush from the cache.
  virtual void remove(const glm::i16vec3 &region_coord) = 0;

  /// Sync from @p src_chunk at @p src_layer to @p dst_chunk at @p dst_layer provided the source chunk and layer are
  /// currently cached.
  ///
  /// While the @p src_chunk and @p src_layer must be valid in the same map the @p MapRegionCache operates on, the
  /// @p dst_chunk and @p dst_layer may be in another map. For example, this function can be used to sync from GPU
  /// memory of one layer to CPU memory of another layer, even in a different map, without having to sync the original
  /// chunk back to CPU first.
  ///
  /// Implementations may make the following assumptions on the arguments given:
  ///
  /// - The source and destination layers are not the same.
  /// - The source chunk belongs to the same map as that which this cache object targets.
  /// - The layer indices are valid.
  ///
  /// The call should do return false if the @p src_chunk and @p src_layer pair are not currently cached by this object.
  /// In this case, no other operation should be performed.
  ///
  /// @param dst_chunk The chunk object to sync to.
  /// @param dst_layer The index to sync to in @c MapChunk::voxel_blocks .
  /// @param src_chunk The chunk to sync from.
  /// @param src_layer The layer to sync from.
  /// @return True if the source chunk/layer pairing are cached by this object and have been copied to the destination
  ///   chunk/layer pairing.
  virtual bool syncLayerTo(MapChunk &dst_chunk, unsigned dst_layer, const MapChunk &src_chunk, unsigned src_layer) = 0;

  /// Find the @c MapRegionCache which specifically targets the specified voxel @p layer . This supports nested caching
  /// as used by the @c GpuLayerCache . May return itself. May return null there is no cache for @p layer .
  /// @param layer The specific layer of interest.
  /// @return The subcache for @p layer - possibly @c this - on success, null on failure.
  virtual MapRegionCache *findLayerCache(unsigned layer) = 0;

private:
  std::unique_ptr<MapRegionCacheDetail> imp_;
};
}  // namespace ohm

#endif  // OHM_MAPREGIONCACHE_H
