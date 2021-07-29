// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuLayerCache.h"

#include "GpuCacheStats.h"
#include "GpuLayerCacheParams.h"

#include <ohm/MapChunk.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapRegion.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelBlock.h>
#include <ohm/VoxelBuffer.h>

#include <gputil/gpuDevice.h>

#include <ohmutil/VectorHash.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#endif  // __GNUC__
#include <ska/bytell_hash_map.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif  // __GNUC__

#include <algorithm>
#include <cassert>
#include <memory>

namespace ohm
{
/// Data required for a single cache entry.
struct GpuCacheEntry  // NOLINT
{
  /// The cached chunk. May be null when the chunk does not exist in the map.
  MapChunk *chunk = nullptr;
  /// Region key for @c chunk.
  glm::i16vec3 region_key = glm::i16vec3(0);
  /// Offset into the GPU buffer at which this chunk's voxels have been uploaded (bytes).
  size_t mem_offset = 0;
  /// Event associated with the most recent operation on @c gpuMem.
  /// This may be an upload, download or kernel execution using the buffer.
  gputil::Event sync_event;
  /// Stamp value used to assess the oldest cache entry.
  uint64_t age_stamp = 0;
  /// Retains uncompressed voxel memory while the chunk remains in the cache.
  VoxelBuffer<VoxelBlock> voxel_buffer;
  // FIXME: (KS) Would be nice to resolve how chunk stamping is managed to sync between GPU and CPU.
  // Currently we must clear the GpuCache when updating on CPU.
  /// Tracks the @c MapChunk::touched_stamps value for the voxel layer. We know the layer has been modified in CPU
  /// this does not match and we must ignore the cached entry.
  uint64_t chunk_touch_stamp = 0;
  /// Most recent @c  batch_marker from @c upload().
  unsigned batch_marker = 0;
  /// Can/should download of this item be skipped?
  bool skip_download = true;
};

struct GpuLayerCacheDetail
{
  using CacheMap = ska::bytell_hash_map<glm::i16vec3, GpuCacheEntry, Vector3Hash<glm::i16vec3>>;

  /// Cache hit/miss stats.
  GpuCacheStats stats;

  // Not part of the public API. We can put whatever we want here.
  std::unique_ptr<gputil::Buffer> buffer;
  unsigned cache_size = 0;
  unsigned batch_marker = 1;
  CacheMap cache;
  /// List of memory offsets available for re-use. Populated when we remove entries from the cache rather than
  /// replacing them.
  std::vector<size_t> mem_offset_free_list;
  glm::u8vec3 region_size = glm::u8vec3(0);
  uint64_t age_stamp = 0;
  gputil::Queue gpu_queue;
  gputil::Device gpu;
  size_t chunk_mem_size = 0;
  /// Initial target allocation size.
  size_t target_gpu_mem_size = 0;
  /// Map layer from which we read or write data
  unsigned layer_index = 0;
  unsigned flags = 0;
  uint8_t *dummy_chunk = nullptr;
  OccupancyMap *map = nullptr;
  GpuCachePostSyncHandler on_sync;

  ~GpuLayerCacheDetail()
  {
    delete[] dummy_chunk;
    // We must clean up the cache explicitly. Otherwise it may be cleaned up after the _gpu device, in which case
    // the events will no longer be valid.
    cache.clear();
  }
};

GpuLayerCache::GpuLayerCache(const gputil::Device &gpu, const gputil::Queue &gpu_queue, OccupancyMap &map,
                             unsigned layer_index, size_t target_gpu_mem_size, unsigned flags,
                             GpuCachePostSyncHandler on_sync)
  : imp_(new GpuLayerCacheDetail)
{
  assert(layer_index < map.layout().layerCount());

  imp_->gpu = gpu;
  imp_->gpu_queue = gpu_queue;
  imp_->layer_index = layer_index;
  imp_->flags = flags;
  imp_->on_sync = std::move(on_sync);
  imp_->map = &map;

  allocateBuffers(map, map.layout().layer(layer_index), target_gpu_mem_size);
}


GpuLayerCache::~GpuLayerCache()
{
  delete imp_;
}


void GpuLayerCache::reinitialise()
{
  clear();
}


void GpuLayerCache::flush()
{
  syncToMainMemory();
}


unsigned GpuLayerCache::beginBatch()
{
  imp_->batch_marker += 2;
  return imp_->batch_marker;
}


void GpuLayerCache::beginBatch(unsigned batch_marker)
{
  imp_->batch_marker = batch_marker;
}


unsigned GpuLayerCache::layerIndex() const
{
  return imp_->layer_index;
}


size_t GpuLayerCache::allocate(OccupancyMap &map, const glm::i16vec3 &region_key, MapChunk *&chunk,
                               gputil::Event *event, CacheStatus *status, unsigned batch_marker, unsigned flags)
{
  GpuCacheEntry *entry = resolveCacheEntry(map, region_key, chunk, event, status, batch_marker, flags, false);
  if (entry)
  {
    return entry->mem_offset;
  }

  return ~size_t{ 0u };
}


size_t GpuLayerCache::upload(OccupancyMap &map, const glm::i16vec3 &region_key, MapChunk *&chunk, gputil::Event *event,
                             CacheStatus *status, unsigned batch_marker, unsigned flags)
{
  GpuCacheEntry *entry = resolveCacheEntry(map, region_key, chunk, event, status, batch_marker, flags, true);
  if (entry)
  {
    return entry->mem_offset;
  }

  return ~size_t{ 0u };
}


bool GpuLayerCache::lookup(OccupancyMap & /*map*/, const glm::i16vec3 &region_key, size_t *offset,
                           gputil::Event *current_event)
{
  // const MapLayer &layer = map.layout().layer(_layerIndex);
  GpuCacheEntry *entry = findCacheEntry(region_key);
  if (entry)
  {
    if (offset)
    {
      *offset = entry->mem_offset;
    }

    if (current_event)
    {
      *current_event = entry->sync_event;
    }
    return true;
  }

  return false;
}


gputil::Buffer *GpuLayerCache::buffer() const
{
  return imp_->buffer.get();
}


void GpuLayerCache::updateEvent(MapChunk &chunk, gputil::Event &event)
{
  GpuCacheEntry *entry = findCacheEntry(chunk);
  if (!entry)
  {
    // This is a logical error.
    return;
  }

  entry->sync_event = event;
  // Touch the chunk entry.
  entry->age_stamp = imp_->age_stamp++;
}


void GpuLayerCache::updateEvents(unsigned batch_marker, gputil::Event &event)
{
  for (auto &iter : imp_->cache)
  {
    if (iter.second.batch_marker == batch_marker)
    {
      iter.second.sync_event = event;
      // Touch the chunk entry.
      iter.second.age_stamp = imp_->age_stamp;
    }
  }
  ++imp_->age_stamp;
}


void GpuLayerCache::remove(const glm::i16vec3 &region_key)
{
  auto search_iter = imp_->cache.find(region_key);

  if (search_iter != imp_->cache.end())
  {
    GpuCacheEntry &entry = search_iter->second;
    // Wait for oustanding operations, but don't sync.
    entry.sync_event.wait();
    // Push the memory offset onto the free list for re-use.
    imp_->mem_offset_free_list.push_back(entry.mem_offset);
    imp_->cache.erase(search_iter);
  }
}


bool GpuLayerCache::syncLayerTo(MapChunk &dst_chunk, unsigned dst_layer, const MapChunk &src_chunk, unsigned src_layer)
{
  if (src_layer == layerIndex())
  {
    VoxelBuffer<VoxelBlock> dst(dst_chunk.voxel_blocks[dst_layer]);
    return syncToExternal(dst, src_chunk.region.coord) > 0;
  }

  return false;
}


MapRegionCache *GpuLayerCache::findLayerCache(unsigned layer)
{
  // This is part of making GpuLayerCache derive MapRegionCache. This type of MapRegionCache can have no children
  // (vs GpuCache which contains GpuLayerCache objects), so we caonly only return this if we match the layer index.
  return (layerIndex() == layer) ? this : nullptr;
}


void GpuLayerCache::syncToMainMemory(const MapChunk &chunk)
{
  GpuCacheEntry *entry = findCacheEntry(chunk);
  if (entry)
  {
    syncToMainMemory(*entry, true);
  }
}


void GpuLayerCache::syncToMainMemory(const glm::i16vec3 &region_key)
{
  GpuCacheEntry *entry = findCacheEntry(region_key);
  if (entry)
  {
    syncToMainMemory(*entry, true);
  }
}


void GpuLayerCache::syncToMainMemory()
{
  // Queue up memory transfers.
  for (auto &iter : imp_->cache)
  {
    GpuCacheEntry &entry = iter.second;
    syncToMainMemory(entry, false);
  }

  // Wait on the queued events.
  for (auto &iter : imp_->cache)
  {
    GpuCacheEntry &entry = iter.second;
    entry.sync_event.wait();
    if (entry.chunk && imp_->on_sync)
    {
      imp_->on_sync(entry.chunk, imp_->region_size);
    }
    // Up to date.
    entry.skip_download = true;
  }
}


size_t GpuLayerCache::syncToExternal(uint8_t *dst, size_t dst_size, const glm::i16vec3 &src_region_key)
{
  if (dst)
  {
    GpuCacheEntry *entry = findCacheEntry(src_region_key);
    if (entry)
    {
      if (entry->chunk && entry->voxel_buffer.isValid() && entry->voxel_buffer.voxelMemorySize() >= dst_size &&
          !entry->skip_download)
      {
        // Found a cached entry to sync.
        // Read voxel data, waiting on the chunk event to ensure it's up to date.
        // We use a synchronous copy to the destination location.
        return imp_->buffer->read(dst, entry->voxel_buffer.voxelMemorySize(), entry->mem_offset, nullptr,
                                  &entry->sync_event);
      }
    }
  }

  return 0;
}


size_t GpuLayerCache::syncToExternal(VoxelBuffer<VoxelBlock> &dst, const glm::i16vec3 &src_region_key)
{
  if (dst.isValid())
  {
    return syncToExternal(dst.voxelMemory(), dst.voxelMemorySize(), src_region_key);
  }
  return 0;
}


gputil::Device &GpuLayerCache::gpu()
{
  return imp_->gpu;
}


const gputil::Device &GpuLayerCache::gpu() const
{
  return imp_->gpu;
}


gputil::Queue &GpuLayerCache::gpuQueue()
{
  return imp_->gpu_queue;
}


const gputil::Queue &GpuLayerCache::gpuQueue() const
{
  return imp_->gpu_queue;
}


unsigned GpuLayerCache::cachedCount() const
{
  return unsigned(imp_->cache.size());
}


unsigned GpuLayerCache::cacheSize() const
{
  return imp_->cache_size;
}


unsigned GpuLayerCache::bufferSize() const
{
  return (imp_->buffer) ? unsigned(imp_->buffer->actualSize()) : 0;
}


unsigned GpuLayerCache::chunkSize() const
{
  return unsigned(imp_->chunk_mem_size);
}


void GpuLayerCache::reallocate(const OccupancyMap &map)
{
  clear();
  imp_->buffer.reset(nullptr);
  allocateBuffers(map, map.layout().layer(imp_->layer_index), imp_->target_gpu_mem_size);
}


void GpuLayerCache::clear()
{
  // Ensure all outstanding GPU transactions are complete, but do not sync.
  for (auto &&entry : imp_->cache)
  {
    entry.second.sync_event.wait();
  }
  imp_->cache.clear();
  imp_->stats.hits = imp_->stats.misses = imp_->stats.full;
}

void GpuLayerCache::queryStats(GpuCacheStats *stats)
{
  *stats = imp_->stats;
}

GpuCacheEntry *GpuLayerCache::resolveCacheEntry(OccupancyMap &map, const glm::i16vec3 &region_key, MapChunk *&chunk,
                                                gputil::Event *event, CacheStatus *status, unsigned batch_marker,
                                                unsigned flags, bool upload)
{
  const MapLayer &layer = map.layout().layer(imp_->layer_index);
  GpuCacheEntry *entry = findCacheEntry(region_key);
  if (entry)
  {
    ++imp_->stats.hits;
    // Already uploaded.
    chunk = entry->chunk;
    // Needs update?
    bool update_required = upload && (flags & kForceUpload) != 0;

    // Check if it was previously added, but without allowing creation.
    if (!entry->chunk)
    {
      // First check if it's been created on CPU.
      entry->chunk = chunk = map.region(region_key, false);
      if (!entry->chunk && (flags & kAllowRegionCreate))
      {
        // Now allowed to create. Do so.
        entry->chunk = chunk = map.region(region_key, true);
        update_required = upload;
      }
      else if (entry->chunk)
      {
        // Has been created on CPU. Require upload.
        update_required = true;
      }
    }

    // FIXME: (KS) resolve detecting CPU changes. Currently must clear the cache.
    if (upload && !update_required && chunk)
    {
      // Check if the chunk has changed in CPU.
      update_required = chunk->touched_stamps[imp_->layer_index] > entry->chunk_touch_stamp;
    }

    entry->skip_download = entry->skip_download && ((flags & kSkipDownload) != 0);

    if (update_required)
    {
      // Upload the chunk in case it has been created while it's been in the cache.
      gputil::Event wait_for_previous = entry->sync_event;
      gputil::Event *wait_for_ptr = (wait_for_previous.isValid()) ? &wait_for_previous : nullptr;

      // Need to hold the voxel buffer until we have written to the GPU buffer.
      if (entry->chunk && !entry->voxel_buffer.isValid())
      {
        entry->voxel_buffer = VoxelBuffer<VoxelBlock>(entry->chunk->voxel_blocks[imp_->layer_index]);
      }
      const uint8_t *voxel_mem =
        (entry->voxel_buffer.isValid()) ? entry->voxel_buffer.voxelMemory() : imp_->dummy_chunk;
      imp_->buffer->write(voxel_mem, layer.layerByteSize(map.regionVoxelDimensions()), entry->mem_offset,
                          &imp_->gpu_queue, wait_for_ptr, &entry->sync_event);
    }
    // We update the touched stamping even though the entry is already present and we may not need to upload anything.
    // We make the assumption that the request for a upload caching is being made because we are about to modify it.
    // Note we also check the skip_download flag in which case there's no need to update the touched_stamps entry.
    if (upload && chunk)
    {
      if (!entry->skip_download)
      {
        // Keeping in sync between GPU and CPU has been an ongoing issue. It probably needs a stamping system which
        // separates CPU and GPU changes, but we don't have that yet. As an interim solution to recognising GPU changes,
        // we update the dirty stamp for a chunk on both upload and download.
        chunk->dirty_stamp = chunk->touched_stamps[imp_->layer_index] = entry->chunk_touch_stamp = imp_->map->stamp();
      }
      else
      {
        entry->chunk_touch_stamp = chunk->touched_stamps[imp_->layer_index];
      }
    }

    if (event)
    {
      *event = entry->sync_event;
    }
    if (status)
    {
      *status = kCacheExisting;
    }
    entry->age_stamp = imp_->age_stamp++;
    if (batch_marker)
    {
      // Update the batch marker.
      entry->batch_marker = batch_marker;
    }
    return entry;
  }

  ++imp_->stats.misses;

  // Not in the cache yet.
  // Ensure the map chunk exists in the map if kAllowRegionCreate is set.
  // Otherwise chunk may be null.
  chunk = map.region(region_key, (flags & kAllowRegionCreate));

  // Now add the chunk to the cache.
  // Check if there are unallocated buffers.
  if (cachedCount() < cacheSize())
  {
    // Use the next buffer.
    GpuCacheEntry new_entry{};
    // First we try poping an entry off the free list.
    if (!imp_->mem_offset_free_list.empty())
    {
      new_entry.mem_offset = imp_->mem_offset_free_list.front();
      imp_->mem_offset_free_list.erase(imp_->mem_offset_free_list.begin());
    }
    else
    {
      new_entry.mem_offset = imp_->chunk_mem_size * cachedCount();
    }
    auto inserted = imp_->cache.insert(std::make_pair(region_key, new_entry));
    entry = &inserted.first->second;
  }
  else
  {
    ++imp_->stats.full;
    // Cache is full. Look for the oldest entry to sync back to main memory.
    auto oldest_entry = imp_->cache.begin();
    for (auto iter = imp_->cache.begin(); iter != imp_->cache.end(); ++iter)
    {
      // Check the age_stamp and the batch marker.
      if (iter->second.age_stamp < oldest_entry->second.age_stamp &&
          (batch_marker == 0 || iter->second.batch_marker != batch_marker))
      {
        oldest_entry = iter;
      }
    }

    if (batch_marker && oldest_entry != imp_->cache.end() && oldest_entry->second.batch_marker == batch_marker)
    {
      // All entries in the cache share the batch_marker. We cannot upload.
      if (status)
      {
        // Cache is full and we cannot release any old entries.
        *status = kCacheFull;
      }
      return nullptr;
    }

    // Synchronise the oldest entry back to main memory.
    syncToMainMemory(oldest_entry->second, true);

    GpuCacheEntry new_entry{};
    new_entry.mem_offset = oldest_entry->second.mem_offset;
    // Remove oldest entry from the cache
    imp_->cache.erase(oldest_entry);

    // Insert the new entry.
    auto inserted = imp_->cache.insert(std::make_pair(region_key, new_entry));
    entry = &inserted.first->second;
  }

  // Complete the cache entry.
  entry->chunk = chunk;  // May be null.
  // Lock chunk memory for the relevant layer. This will be retained while the chunk is in this cache.
  entry->voxel_buffer =
    (chunk) ? VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[imp_->layer_index]) : VoxelBuffer<VoxelBlock>();
  entry->region_key = region_key;
  entry->age_stamp = imp_->age_stamp++;
  if (batch_marker)
  {
    // Update the batch marker.
    entry->batch_marker = batch_marker;
  }
  entry->skip_download = (flags & kSkipDownload);

  if (upload)
  {
    const uint8_t *voxel_mem = (entry->voxel_buffer.isValid()) ? entry->voxel_buffer.voxelMemory() : imp_->dummy_chunk;
    imp_->buffer->write(voxel_mem, imp_->chunk_mem_size, entry->mem_offset, &imp_->gpu_queue, nullptr,
                        &entry->sync_event);
    if (chunk)
    {
      if (!entry->skip_download)
      {
        // As above where we upload to update, we change the stamp for the chunk on both upload and download.
        chunk->dirty_stamp = chunk->touched_stamps[imp_->layer_index] = entry->chunk_touch_stamp = imp_->map->stamp();
      }
      else
      {
        entry->chunk_touch_stamp = chunk->touched_stamps[imp_->layer_index];
      }
    }
    else
    {
      entry->chunk_touch_stamp = imp_->map->stamp();
    }
  }

  if (event)
  {
    *event = entry->sync_event;
  }
  if (status)
  {
    *status = kCacheNew;
  }

  return entry;
}


void GpuLayerCache::allocateBuffers(const OccupancyMap &map, const MapLayer &layer, size_t target_gpu_mem_size)
{
  // Query the available device memory.
  auto mem_limit = imp_->gpu.maxAllocationSize();
  // Limit to using 1/2 of the device memory. This is a bit of a left over from when there was only one layer to cache.
  mem_limit = (mem_limit * 1) / 2;
  target_gpu_mem_size = (target_gpu_mem_size <= mem_limit) ? target_gpu_mem_size : mem_limit;

  imp_->target_gpu_mem_size = target_gpu_mem_size;
  imp_->region_size = layer.dimensions(map.regionVoxelDimensions());
  imp_->chunk_mem_size = layer.layerByteSize(map.regionVoxelDimensions());

  size_t allocated = 0;

  // Do loop to ensure we allocate at least one buffer.
  unsigned buffer_flags = gputil::kBfReadWrite;
  if (imp_->flags & kGcfMappable)
  {
    buffer_flags |= gputil::kBfHostAccess;
  }

  imp_->cache_size = 0;

  do
  {
    allocated += imp_->chunk_mem_size;
    ++imp_->cache_size;
  } while (allocated + imp_->chunk_mem_size <= target_gpu_mem_size);

  imp_->buffer = std::make_unique<gputil::Buffer>(imp_->gpu, allocated, buffer_flags);

  imp_->dummy_chunk = new uint8_t[layer.layerByteSize(map.regionVoxelDimensions())];
  layer.clear(imp_->dummy_chunk, map.regionVoxelDimensions());
}


void GpuLayerCache::syncToMainMemory(GpuCacheEntry &entry, bool wait_on_sync)
{
  if (entry.chunk && !entry.skip_download)
  {
    // Cache the current sync event. We will make the memory copy depend on this event.
    gputil::Event last_event = entry.sync_event;
    // Release the entry's sync event. We will git it a new one.
    entry.sync_event.release();
    // This should technically always be true if chunk is not null.
    if (entry.voxel_buffer.isValid())
    {
      // Queue memory read blocking on the last event and tracking a new one in entry.syncEvent
      uint8_t *voxel_mem = entry.voxel_buffer.voxelMemory();
      imp_->buffer->read(voxel_mem, imp_->chunk_mem_size, entry.mem_offset, &imp_->gpu_queue, &last_event,
                         &entry.sync_event);
      // Update the dirty stamp for the region
      entry.chunk->dirty_stamp = entry.chunk->touched_stamps[imp_->layer_index] = entry.chunk_touch_stamp =
        imp_->map->touch();
      // Also need to invalidate the MapChunk::first_valid_index as we don't know what it will be coming off the GPU.
      // We only apply this change for the occupancy layer
      if (imp_->layer_index == unsigned(imp_->map->layout().occupancyLayer()))
      {
        entry.chunk->invalidateFirstValidIndex();
      }
    }
  }

  // Do we block now on the sync? This could be changed to execute only when we don't skip download.
  // Must wait if we have an on_sync handler (to call it). Eventually we may be able to use a GPU post event hook, but
  /// there are thread-safety issues with that.
  if (wait_on_sync)
  {
    // Wait for operations to complete.
    entry.sync_event.wait();
    // Up to date.
    entry.skip_download = true;

    if (imp_->on_sync)
    {
      imp_->on_sync(entry.chunk, imp_->region_size);
    }
  }
}


namespace
{
template <typename ENTRY, typename T>
inline ENTRY *findCacheEntry(T &cache, const glm::i16vec3 &region_key)
{
  auto search_iter = cache.find(region_key);

  if (search_iter != cache.end())
  {
    return &search_iter->second;
  }

  // Not in the GPU cache.
  return nullptr;
}
}  // namespace


GpuCacheEntry *GpuLayerCache::findCacheEntry(const glm::i16vec3 &region_key)
{
  return ohm::findCacheEntry<GpuCacheEntry>(imp_->cache, region_key);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const glm::i16vec3 &region_key) const
{
  return ohm::findCacheEntry<const GpuCacheEntry>(imp_->cache, region_key);
}


GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk)
{
  return ohm::findCacheEntry<GpuCacheEntry>(imp_->cache, chunk.region.coord);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk) const
{
  return ohm::findCacheEntry<const GpuCacheEntry>(imp_->cache, chunk.region.coord);
}

}  // namespace ohm
