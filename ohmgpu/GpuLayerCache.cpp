// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuLayerCache.h"

#include "GpuCacheParams.h"

#include <ohm/MapChunk.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapRegion.h>
#include <ohm/OccupancyMap.h>

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

#include <cassert>
#include <memory>

using namespace ohm;

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
    // FIXME: (KS) Would be nice to resolve how chunk stamping is managed to sync between GPU and CPU.
    // Currently we must clear the GpuCache when updating on CPU.
    // /// Tracks the @c MapChunk::touched_stamps value for the voxel layer. We know the layer has been modified in CPU if
    // /// this does not match and we must ignore the cached entry.
    // uint64_t chunk_touch_stamp = 0;
    /// Most recent @c  batch_marker from @c upload().
    unsigned batch_marker = 0;
    /// Can/should download of this item be skipped?
    bool skip_download = true;
  };

  struct GpuLayerCacheDetail
  {
    using CacheMap = ska::bytell_hash_map<glm::i16vec3, GpuCacheEntry, Vector3Hash<glm::i16vec3>>;

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
    unsigned layer_index = 0;
    unsigned flags = 0;
    uint8_t *dummy_chunk = nullptr;
    GpuCachePostSyncHandler on_sync;

    ~GpuLayerCacheDetail()
    {
      delete[] dummy_chunk;
      // We must clean up the cache explicitly. Otherwise it may be cleaned up after the _gpu device, in which case
      // the events will no longer be valid.
      cache.clear();
    }
  };
}  // namespace ohm

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
  imp_->on_sync = on_sync;

  allocateBuffers(map, map.layout().layer(layer_index), target_gpu_mem_size);
}


GpuLayerCache::~GpuLayerCache()
{
  delete imp_;
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
}


GpuCacheEntry *GpuLayerCache::resolveCacheEntry(OccupancyMap &map, const glm::i16vec3 &region_key, MapChunk *&chunk,
                                                gputil::Event *event, CacheStatus *status, unsigned batch_marker,
                                                unsigned flags, bool upload)
{
  const MapLayer &layer = map.layout().layer(imp_->layer_index);
  GpuCacheEntry *entry = findCacheEntry(region_key);
  if (entry)
  {
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
    // if (upload && !update_required && entry->chunk)
    // {
    //   // Check if the chunk has changed in CPU.
    //   update_required = chunk->touched_stamps[imp_->layer_index] != entry->chunk_touch_stamp;
    // }

    if (update_required)
    {
      // Upload the chunk in case it has been created while it's been in the cache.
      gputil::Event wait_for_previous = entry->sync_event;
      gputil::Event *wait_for_ptr = (wait_for_previous.isValid()) ? &wait_for_previous : nullptr;
      const uint8_t *voxel_mem = (entry->chunk) ? layer.voxels(*entry->chunk) : imp_->dummy_chunk;
      imp_->buffer->write(voxel_mem, layer.layerByteSize(map.regionVoxelDimensions()), entry->mem_offset,
                          &imp_->gpu_queue, wait_for_ptr, &entry->sync_event);
    }

    entry->skip_download = entry->skip_download && ((flags & kSkipDownload) != 0);

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

  // Ensure the map chunk exists in the map.
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
    const uint8_t *voxel_mem = (chunk) ? layer.voxels(*chunk) : imp_->dummy_chunk;
    imp_->buffer->write(voxel_mem, imp_->chunk_mem_size, entry->mem_offset, &imp_->gpu_queue, nullptr,
                        &entry->sync_event);
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
  auto mem_limit = imp_->gpu.deviceMemory();
  // Limit to using 1/2 of the device memory.
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
    // Queue memory read blocking on the last event and tracking a new one in entry.syncEvent
    uint8_t *voxel_mem = entry.chunk->layout->layer(imp_->layer_index).voxels(*entry.chunk);
    imp_->buffer->read(voxel_mem, imp_->chunk_mem_size, entry.mem_offset, &imp_->gpu_queue, &last_event,
                       &entry.sync_event);
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
  return ::findCacheEntry<GpuCacheEntry>(imp_->cache, region_key);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const glm::i16vec3 &region_key) const
{
  return ::findCacheEntry<const GpuCacheEntry>(imp_->cache, region_key);
}


GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk)
{
  return ::findCacheEntry<GpuCacheEntry>(imp_->cache, chunk.region.coord);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk) const
{
  return ::findCacheEntry<const GpuCacheEntry>(imp_->cache, chunk.region.coord);
}
