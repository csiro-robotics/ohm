// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpulayercache.h"

#include "gpucacheparams.h"
#include "mapchunk.h"
#include "maplayer.h"
#include "maplayout.h"
#include "mapregion.h"
#include "occupancymap.h"

#include <gputil/gpudevice.h>

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
    glm::i16vec3 regionKey;
    /// Offset into the GPU buffer at which this chunk's voxels have been uploaded (bytes).
    size_t memOffset;
    /// Event associated with the most recent operation on @c gpuMem.
    /// This may be an upload, download or kernel execution using the buffer.
    gputil::Event syncEvent;
    /// Stamp value used to assess the oldest cache entry.
    uint64_t stamp;
    /// Most recent @c  batchMarker from @c upload().
    unsigned batchMarker;
    /// Can/should download of this item be skipped?
    bool skipDownload;

    void init()
    {
      chunk = nullptr;
      memOffset = 0;
      stamp = 0;
      batchMarker = 0;
      skipDownload = true;
    }
  };

  struct GpuLayerCacheDetail
  {
    // Not part of the public API. We can put whatever we want here.
    gputil::Buffer *buffer = nullptr;
    unsigned cacheSize = 0;
    unsigned batchMarker = 1;
    std::unordered_multimap<unsigned, GpuCacheEntry> cache;
    glm::u8vec3 regionSize;
    uint64_t stamp = 0;
    gputil::Queue gpuQueue;
    gputil::Device gpu;
    size_t chunkMemSize = 0;
    unsigned layerIndex = 0;
    unsigned flags = 0;
    uint8_t *dummyChunk = nullptr;

    ~GpuLayerCacheDetail()
    {
      delete buffer;
      delete [] dummyChunk;
      // We must clean up the cache explicitly. Otherwise it may be cleaned up after the _gpu device, in which case
      // the events will no longer be valid.
      cache.clear();
    }
  };
}

GpuLayerCache::GpuLayerCache(const gputil::Device &gpu, const gputil::Queue &gpuQueue,
                             OccupancyMap &map, unsigned layerIndex, size_t targetGpuMemSize, unsigned flags)
  : _imp(new GpuLayerCacheDetail)
{
  assert(layerIndex < map.layout().layerCount());

  _imp->gpu = gpu;
  _imp->gpuQueue = gpuQueue;
  _imp->layerIndex = layerIndex;
  _imp->flags = flags;

  allocateBuffers(map, map.layout().layer(layerIndex), targetGpuMemSize);
}


GpuLayerCache::~GpuLayerCache()
{
  delete _imp;
}


unsigned GpuLayerCache::beginBatch()
{
  _imp->batchMarker += 2;
  return _imp->batchMarker;
}


unsigned GpuLayerCache::layerIndex() const
{
  return _imp->layerIndex;
}


size_t GpuLayerCache::allocate(OccupancyMap &map, const glm::i16vec3 &regionKey, MapChunk *&chunk, gputil::Event *event,
                               CacheStatus *status, unsigned batchMarker, unsigned flags)
{
  GpuCacheEntry *entry = resolveCacheEntry(map, regionKey, chunk, event, status, batchMarker, flags, false);
  if (entry)
  {
    return entry->memOffset;
  }

  return size_t(~0u);
}


size_t GpuLayerCache::upload(OccupancyMap &map, const glm::i16vec3 &regionKey, MapChunk *&chunk, gputil::Event *event,
                             CacheStatus *status, unsigned batchMarker, unsigned flags)
{
  GpuCacheEntry *entry = resolveCacheEntry(map, regionKey, chunk, event, status, batchMarker, flags, true);
  if (entry)
  {
    return entry->memOffset;
  }

  return size_t(~0u);
}


bool GpuLayerCache::lookup(OccupancyMap &map, const glm::i16vec3 &regionKey, size_t *offset, gputil::Event *currentEvent)
{
  // const MapLayer &layer = map.layout().layer(_layerIndex);
  GpuCacheEntry *entry = findCacheEntry(regionKey);
  if (entry)
  {
    if (offset)
    {
      *offset = entry->memOffset;
    }

    if (currentEvent)
    {
      *currentEvent = entry->syncEvent;
    }
    return true;
  }

  return false;
}


gputil::Buffer *GpuLayerCache::buffer() const
{
  return _imp->buffer;
}


void GpuLayerCache::updateEvent(MapChunk &chunk, gputil::Event &event)
{
  GpuCacheEntry *entry = findCacheEntry(chunk);
  if (!entry)
  {
    // This is a logical error.
    return;
  }

  entry->syncEvent = event;
  // Touch the chunk entry.
  entry->stamp = _imp->stamp++;
}


void GpuLayerCache::updateEvents(unsigned batchMarker, gputil::Event &event)
{
  for (auto iter = _imp->cache.begin(); iter != _imp->cache.end(); ++iter)
  {
    if (iter->second.batchMarker == batchMarker)
    {
      iter->second.syncEvent = event;
      // Touch the chunk entry.
      iter->second.stamp = _imp->stamp;
    }
  }
  ++_imp->stamp;
}


void GpuLayerCache::syncToMainMemory(const MapChunk &chunk)
{
  GpuCacheEntry *entry = findCacheEntry(chunk);
  if (entry)
  {
    syncToMainMemory(*entry, true);
  }
}


void GpuLayerCache::syncToMainMemory(const glm::i16vec3 &regionKey)
{
  GpuCacheEntry *entry = findCacheEntry(regionKey);
  if (entry)
  {
    syncToMainMemory(*entry, true);
  }
}


void GpuLayerCache::syncToMainMemory()
{
  // Queue up memory transfers.
  for (auto iter = _imp->cache.begin(); iter != _imp->cache.end(); ++iter)
  {
    GpuCacheEntry &entry = iter->second;
    syncToMainMemory(entry, false);
  }

  // Wait on the queued events.
  for (auto iter = _imp->cache.begin(); iter != _imp->cache.end(); ++iter)
  {
    GpuCacheEntry &entry = iter->second;
    entry.syncEvent.wait();
    if (entry.chunk)
    {
      entry.chunk->searchAndUpdateFirstValid(_imp->regionSize);
    }
    // Up to date.
    entry.skipDownload = true;
  }
}


gputil::Device &GpuLayerCache::gpu()
{
  return _imp->gpu;
}


const gputil::Device &GpuLayerCache::gpu() const
{
  return _imp->gpu;
}


gputil::Queue &GpuLayerCache::gpuQueue()
{
  return _imp->gpuQueue;
}


const gputil::Queue &GpuLayerCache::gpuQueue() const
{
  return _imp->gpuQueue;
}


unsigned GpuLayerCache::cachedCount() const
{
  return unsigned(_imp->cache.size());
}


unsigned GpuLayerCache::cacheSize() const
{
  return _imp->cacheSize;
}


void GpuLayerCache::clear()
{
  _imp->cache.clear();
}


GpuCacheEntry *GpuLayerCache::resolveCacheEntry(OccupancyMap &map, const glm::i16vec3 &regionKey,
                                                MapChunk *&chunk, gputil::Event *event,
                                                CacheStatus *status, unsigned batchMarker,
                                                unsigned flags, bool upload)
{
  const MapLayer &layer = map.layout().layer(_imp->layerIndex);
  GpuCacheEntry *entry = findCacheEntry(regionKey);
  if (entry)
  {
    // Already uploaded.
    chunk = entry->chunk;
    // Needs update?
    bool updateRequired = upload && (flags & ForceUpload) != 0;

    // Check if it was previously added, but without allowing creation.
    if (!chunk && (flags & AllowRegionCreate))
    {
      // Now allowed to create. Do so.
      entry->chunk = chunk = map.region(regionKey, true);
      updateRequired = upload;
    }

    if (updateRequired)
    {
      // Upload the chunk in case it has been created while it's been in the cache.
      gputil::Event waitForPrevious = entry->syncEvent;
      gputil::Event *waitForPtr = waitForPrevious.isValid() ? &waitForPrevious : nullptr;
      const uint8_t *voxelMem = layer.voxels(*chunk);
      _imp->buffer->write(voxelMem, layer.layerByteSize(map.regionVoxelDimensions()), entry->memOffset,
                          &_imp->gpuQueue, waitForPtr, &entry->syncEvent);
    }

    entry->skipDownload = entry->skipDownload && ((flags & SkipDownload) != 0);

    if (event)
    {
      *event = entry->syncEvent;
    }
    if (status)
    {
      *status = CacheExisting;
    }
    entry->stamp = _imp->stamp++;
    if (batchMarker)
    {
      // Update the batch marker.
      entry->batchMarker = batchMarker;
    }
    return entry;
  }

  // Ensure the map chunk exists in the map.
  chunk = map.region(regionKey, (flags & AllowRegionCreate));
  const unsigned regionHash = (chunk) ? chunk->region.hash : MapRegion::Hash::calculate(regionKey);

  // Now add the chunk to the cache.
  // Check if there are unallocated buffers.
  if (cachedCount() < cacheSize())
  {
    // Use the next buffer.
    GpuCacheEntry newEntry;
    newEntry.init();
    newEntry.memOffset = _imp->chunkMemSize * cachedCount();
    auto inserted = _imp->cache.insert(std::make_pair(regionHash, newEntry));
    entry = &inserted->second;
  }
  else
  {
    // Cache is full. Look for the oldest entry to sync back to main memory.
    auto oldestEntry = _imp->cache.begin();
    for (auto iter = _imp->cache.begin(); iter != _imp->cache.end(); ++iter)
    {
      // Check the stamp and the batch marker.
      if (iter->second.stamp < oldestEntry->second.stamp &&
          (batchMarker == 0 || iter->second.batchMarker != batchMarker))
      {
        oldestEntry = iter;
      }
    }

    if (batchMarker && oldestEntry != _imp->cache.end() && oldestEntry->second.batchMarker == batchMarker)
    {
      // All entries in the cache share the batchMarker. We cannot upload.
      if (status)
      {
        // Cache is full and we cannot release any old entries.
        *status = CacheFull;
      }
      return nullptr;
    }

    // Synchronise the oldest entry back to main memory.
    syncToMainMemory(oldestEntry->second, true);

    GpuCacheEntry newEntry;
    newEntry.init();
    newEntry.memOffset = oldestEntry->second.memOffset;
    // Remove oldest entry from the cache
    _imp->cache.erase(oldestEntry);

    // Insert the new entry.
    auto inserted = _imp->cache.insert(std::make_pair(regionHash, newEntry));
    entry = &inserted->second;
  }

  // Complete the cache entry.
  entry->chunk = chunk;  // May be null.
  entry->regionKey = regionKey;
  entry->stamp = _imp->stamp++;
  if (batchMarker)
  {
    // Update the batch marker.
    entry->batchMarker = batchMarker;
  }
  entry->skipDownload = (flags & SkipDownload);

  if (upload)
  {
    // std::cout << "upload " << chunk.region.coord << '\n';
    const uint8_t *voxelMem = (chunk) ? layer.voxels(*chunk) : _imp->dummyChunk;
    _imp->buffer->write(voxelMem, _imp->chunkMemSize, entry->memOffset, &_imp->gpuQueue, nullptr, &entry->syncEvent);
  }

  if (event)
  {
    *event = entry->syncEvent;
  }
  if (status)
  {
    *status = CacheNew;
  }

  return entry;
}


void GpuLayerCache::allocateBuffers(const OccupancyMap &map, const MapLayer &layer, size_t targetGpuMemSize)
{
  // Query the available device memory.
  auto memLimit = _imp->gpu.deviceMemory();
  // Limit to using 1/2 of the device memory.
  memLimit = (memLimit * 1) / 2;
  targetGpuMemSize = (targetGpuMemSize <= memLimit) ? targetGpuMemSize : memLimit;

  _imp->regionSize = layer.dimensions(map.regionVoxelDimensions());
  _imp->chunkMemSize = layer.layerByteSize(map.regionVoxelDimensions());
  size_t allocated = 0;

  // Do loop to ensure we allocate at least one buffer.
  unsigned bufferFlags = gputil::BF_ReadWrite;
  if (_imp->flags & GCFMappable)
  {
    bufferFlags |= gputil::BF_HostAccess;
  }

  _imp->cacheSize = 0;

  do
  {
    allocated += _imp->chunkMemSize;
    ++_imp->cacheSize;
  } while (allocated + _imp->chunkMemSize <= targetGpuMemSize);

  _imp->buffer = new gputil::Buffer(_imp->gpu, allocated, bufferFlags);

  const size_t NodeCount = layer.volume(map.regionVoxelDimensions());
  _imp->dummyChunk = new uint8_t[layer.layerByteSize(map.regionVoxelDimensions())];
  layer.clear(_imp->dummyChunk, map.regionVoxelDimensions());
}


void GpuLayerCache::syncToMainMemory(GpuCacheEntry &entry, bool waitOnSync)
{
  if (entry.chunk && !entry.skipDownload)
  {
    // Cache the current sync event. We will make the memory copy depend on this event.
    gputil::Event lastEvent = entry.syncEvent;
    // Release the entry's sync event. We will git it a new one.
    entry.syncEvent.release();
    // Queue memory read blocking on the last event and traking a new one in entry.syncEvent
    uint8_t *voxelMem = entry.chunk->layout->layer(_imp->layerIndex).voxels(*entry.chunk);
    _imp->buffer->read(voxelMem, _imp->chunkMemSize, entry.memOffset, &_imp->gpuQueue, &lastEvent, &entry.syncEvent);
  }

  // Do we block now on the sync? This could be changed to execute only when we don't skip download.
  if (waitOnSync)
  {
    // Wait for operations to complete.
    entry.syncEvent.wait();
    // Up to date.
    entry.skipDownload = true;
  }
}


namespace
{
  template <typename Entry, typename T>
  inline Entry *findCacheEntry(T &cache, unsigned regionHash, const glm::i16vec3 &regionKey)
  {
    auto searchIter = cache.find(regionHash);

    while (searchIter != cache.end() && searchIter->first == regionHash &&
           searchIter->second.regionKey != regionKey)
    {
      ++searchIter;
    }

    if (searchIter != cache.end() && searchIter->first == regionHash &&
        searchIter->second.regionKey == regionKey)
    {
      return &searchIter->second;
    }

    // Not in the GPU cache.
    return nullptr;
  }
}


GpuCacheEntry *GpuLayerCache::findCacheEntry(const glm::i16vec3 &regionKey)
{
  const unsigned regionHash = MapRegion::Hash::calculate(regionKey);
  return ::findCacheEntry<GpuCacheEntry>(_imp->cache, regionHash, regionKey);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const glm::i16vec3 &regionKey) const
{
  const unsigned regionHash = MapRegion::Hash::calculate(regionKey);
  return ::findCacheEntry<const GpuCacheEntry>(_imp->cache, regionHash, regionKey);
}


GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk)
{
  return ::findCacheEntry<GpuCacheEntry>(_imp->cache, chunk.region.hash, chunk.region.coord);
}


const GpuCacheEntry *GpuLayerCache::findCacheEntry(const MapChunk &chunk) const
{
  return ::findCacheEntry<const GpuCacheEntry>(_imp->cache, chunk.region.hash, chunk.region.coord);
}
