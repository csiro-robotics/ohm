// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuLayerCache.h"

#include "GpuCacheParams.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "MapRegion.h"
#include "OccupancyMap.h"

#include <gputil/gpuDevice.h>

#include <unordered_map>

#include <cassert>
#include <iostream>

using namespace ohm;

namespace ohm
{
  /// Data required for a single cache entry.
  struct GpuCacheEntry
  {
    /// The cached chunk. May be null when the chunk does not exist in the map.
    MapChunk *chunk;
    /// Region key for @c chunk.
    glm::i16vec3 region_key;
    /// Offset into the GPU buffer at which this chunk's voxels have been uploaded (bytes).
    size_t mem_offset;
    /// Event associated with the most recent operation on @c gpuMem.
    /// This may be an upload, download or kernel execution using the buffer.
    gputil::Event sync_event;
    /// Stamp value used to assess the oldest cache entry.
    uint64_t stamp;
    /// Most recent @c  batch_marker from @c upload().
    unsigned batch_marker;
    /// Can/should download of this item be skipped?
    bool skip_download;

    void init()
    {
      chunk = nullptr;
      mem_offset = 0;
      stamp = 0;
      batch_marker = 0;
      skip_download = true;
    }
  };

  struct GpuLayerCacheDetail
  {
    // Not part of the public API. We can put whatever we want here.
    gputil::Buffer *buffer = nullptr;
    unsigned cache_size = 0;
    unsigned batch_marker = 1;
    std::unordered_multimap<unsigned, GpuCacheEntry> cache;
    glm::u8vec3 region_size;
    uint64_t stamp = 0;
    gputil::Queue gpu_queue;
    gputil::Device gpu;
    size_t chunk_mem_size = 0;
    unsigned layer_index = 0;
    unsigned flags = 0;
    uint8_t *dummy_chunk = nullptr;

    ~GpuLayerCacheDetail()
    {
      delete buffer;
      delete [] dummy_chunk;
      // We must clean up the cache explicitly. Otherwise it may be cleaned up after the _gpu device, in which case
      // the events will no longer be valid.
      cache.clear();
    }
  };
}

GpuLayerCache::GpuLayerCache(const gputil::Device &gpu, const gputil::Queue &gpu_queue,
                             OccupancyMap &map, unsigned layer_index, size_t target_gpu_mem_size, unsigned flags)
  : imp_(new GpuLayerCacheDetail)
{
  assert(layer_index < map.layout().layerCount());

  imp_->gpu = gpu;
  imp_->gpu_queue = gpu_queue;
  imp_->layer_index = layer_index;
  imp_->flags = flags;

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


unsigned GpuLayerCache::layerIndex() const
{
  return imp_->layer_index;
}


size_t GpuLayerCache::allocate(OccupancyMap &map, const glm::i16vec3 &region_key, MapChunk *&chunk, gputil::Event *event,
                               CacheStatus *status, unsigned batch_marker, unsigned flags)
{
  GpuCacheEntry *entry = resolveCacheEntry(map, region_key, chunk, event, status, batch_marker, flags, false);
  if (entry)
  {
    return entry->mem_offset;
  }

  return size_t(~0u);
}


size_t GpuLayerCache::upload(OccupancyMap &map, const glm::i16vec3 &region_key, MapChunk *&chunk, gputil::Event *event,
                             CacheStatus *status, unsigned batch_marker, unsigned flags)
{
  GpuCacheEntry *entry = resolveCacheEntry(map, region_key, chunk, event, status, batch_marker, flags, true);
  if (entry)
  {
    return entry->mem_offset;
  }

  return size_t(~0u);
}


bool GpuLayerCache::lookup(OccupancyMap &/*map*/, const glm::i16vec3 &region_key, size_t *offset, gputil::Event *current_event)
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
  return imp_->buffer;
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
  entry->stamp = imp_->stamp++;
}


void GpuLayerCache::updateEvents(unsigned batch_marker, gputil::Event &event)
{
  for (auto &iter : imp_->cache)
  {
    if (iter.second.batch_marker == batch_marker)
    {
      iter.second.sync_event = event;
      // Touch the chunk entry.
      iter.second.stamp = imp_->stamp;
    }
  }
  ++imp_->stamp;
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
    if (entry.chunk)
    {
      entry.chunk->searchAndUpdateFirstValid(imp_->region_size);
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


void GpuLayerCache::clear()
{
  imp_->cache.clear();
}


GpuCacheEntry *GpuLayerCache::resolveCacheEntry(OccupancyMap &map, const glm::i16vec3 &region_key,
                                                MapChunk *&chunk, gputil::Event *event,
                                                CacheStatus *status, unsigned batch_marker,
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
    if (!chunk && (flags & kAllowRegionCreate))
    {
      // Now allowed to create. Do so.
      entry->chunk = chunk = map.region(region_key, true);
      update_required = upload;
    }

    if (update_required)
    {
      // Upload the chunk in case it has been created while it's been in the cache.
      gputil::Event wait_for_previous = entry->sync_event;
      gputil::Event *wait_for_ptr = wait_for_previous.isValid() ? &wait_for_previous : nullptr;
      const uint8_t *voxel_mem = layer.voxels(*chunk);
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
    entry->stamp = imp_->stamp++;
    if (batch_marker)
    {
      // Update the batch marker.
      entry->batch_marker = batch_marker;
    }
    return entry;
  }

  // Ensure the map chunk exists in the map.
  chunk = map.region(region_key, (flags & kAllowRegionCreate));
  const unsigned region_hash = (chunk) ? chunk->region.hash : MapRegion::Hash::calculate(region_key);

  // Now add the chunk to the cache.
  // Check if there are unallocated buffers.
  if (cachedCount() < cacheSize())
  {
    // Use the next buffer.
    GpuCacheEntry new_entry;
    new_entry.init();
    new_entry.mem_offset = imp_->chunk_mem_size * cachedCount();
    auto inserted = imp_->cache.insert(std::make_pair(region_hash, new_entry));
    entry = &inserted->second;
  }
  else
  {
    // Cache is full. Look for the oldest entry to sync back to main memory.
    auto oldest_entry = imp_->cache.begin();
    for (auto iter = imp_->cache.begin(); iter != imp_->cache.end(); ++iter)
    {
      // Check the stamp and the batch marker.
      if (iter->second.stamp < oldest_entry->second.stamp &&
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

    GpuCacheEntry new_entry;
    new_entry.init();
    new_entry.mem_offset = oldest_entry->second.mem_offset;
    // Remove oldest entry from the cache
    imp_->cache.erase(oldest_entry);

    // Insert the new entry.
    auto inserted = imp_->cache.insert(std::make_pair(region_hash, new_entry));
    entry = &inserted->second;
  }

  // Complete the cache entry.
  entry->chunk = chunk;  // May be null.
  entry->region_key = region_key;
  entry->stamp = imp_->stamp++;
  if (batch_marker)
  {
    // Update the batch marker.
    entry->batch_marker = batch_marker;
  }
  entry->skip_download = (flags & kSkipDownload);

  if (upload)
  {
    // std::cout << "upload " << chunk.region.coord << '\n';
    const uint8_t *voxel_mem = (chunk) ? layer.voxels(*chunk) : imp_->dummy_chunk;
    imp_->buffer->write(voxel_mem, imp_->chunk_mem_size, entry->mem_offset, &imp_->gpu_queue, nullptr, &entry->sync_event);
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

  imp_->buffer = new gputil::Buffer(imp_->gpu, allocated, buffer_flags);

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
    // Queue memory read blocking on the last event and traking a new one in entry.syncEvent
    uint8_t *voxel_mem = entry.chunk->layout->layer(imp_->layer_index).voxels(*entry.chunk);
    imp_->buffer->read(voxel_mem, imp_->chunk_mem_size, entry.mem_offset, &imp_->gpu_queue, &last_event, &entry.sync_event);
  }

  // Do we block now on the sync? This could be changed to execute only when we don't skip download.
  if (wait_on_sync)
  {
    // Wait for operations to complete.
    entry.sync_event.wait();
    // Up to date.
    entry.skip_download = true;
  }
}


namespace
{
  template <typename ENTRY, typename T>
  inline ENTRY *findCacheEntry(T &cache, unsigned region_hash, const glm::i16vec3 &region_key)
  {
    auto search_iter = cache.find(region_hash);

    while (search_iter != cache.end() && search_iter->first == region_hash &&
           search_iter->second.region_key != region_key)
    {
      ++search_iter;
    }

    if (search_iter != cache.end() && search_iter->first == region_hash &&
        search_iter->second.region_key == region_key)
    {
      return &search_iter->second;
    }

    // Not in the GPU cache.
    return nullptr;
  }
}


GpuCacheEntry *GpuLayerCache::findCacheEntry(const glm::i16vec3 &region_key)
{
  const unsigned region_hash = MapRegion::Hash::calculate(region_key);
  return ::findCacheEntry<GpuCacheEntry>(imp_->cache, region_hash, region_key);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const glm::i16vec3 &region_key) const
{
  const unsigned region_hash = MapRegion::Hash::calculate(region_key);
  return ::findCacheEntry<const GpuCacheEntry>(imp_->cache, region_hash, region_key);
}


GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk)
{
  return ::findCacheEntry<GpuCacheEntry>(imp_->cache, chunk.region.hash, chunk.region.coord);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk) const
{
  return ::findCacheEntry<const GpuCacheEntry>(imp_->cache, chunk.region.hash, chunk.region.coord);
}
