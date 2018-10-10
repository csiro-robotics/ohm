//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "MapCache.h"

#include "MapChunk.h"
#include "MapRegion.h"
#include "Voxel.h"

#include <cstring>

using namespace ohm;

MapCache::MapCache(unsigned cache_size) // NOLINT
{
#if OHM_MULTI_CACHE
  _chunks = new MapChunk *[cacheSize];
  _cacheSize = cacheSize;
  memset(_chunks, 0, sizeof(*_chunks) * cacheSize);
#else  // OHM_MULTI_CACHE
  // cache_size is unused on this path.
  (void)cache_size;
  chunk_ = nullptr;
#endif // OHM_MULTI_CACHE
  memset(&stats_, 0, sizeof(stats_));
}


MapCache::~MapCache()
{
#if OHM_MULTI_CACHE
  delete [] _chunks;
#endif // OHM_MULTI_CACHE
}


MapChunk *MapCache::lookup(const OccupancyKey &key)
{
#if OHM_MULTI_CACHE
  for (int i = 0; i < _cacheSize; ++i)
  {
    if (_chunks[i] && _chunks[i]->region.coord == key.regionKey())
    {
      ++_stats.hit;
      return _chunks[i];
    }
  }
#else  // OHM_MULTI_CACHE
  if (chunk_ && chunk_->region.coord == key.regionKey())
  {
    ++stats_.hit;
    return chunk_;
  }
#endif // OHM_MULTI_CACHE

  ++stats_.miss;
  return nullptr;
}


void MapCache::push(MapChunk *chunk)
{
#if OHM_MULTI_CACHE
  for (int i = 0; i < _cacheSize; ++i)
  {
    if (chunk == _chunks[i])
    {
      return;
    }
  }

  if (_cacheSize > 1)
  {
    memmove(&_chunks[0], &_chunks[1], sizeof(*_chunks) * (_cacheSize - 1));
  }
  _chunks[0] = chunk;
#else  // OHM_MULTI_CACHE
  chunk_ = chunk;
#endif // OHM_MULTI_CACHE
}


void MapCache::clear()
{
#if OHM_MULTI_CACHE
  memset(_chunks, 0, sizeof(*_chunks) * _cacheSize);
#else  // OHM_MULTI_CACHE
  chunk_ = nullptr;
#endif // OHM_MULTI_CACHE
}


void MapCache::getStats(Stats &stats) const
{
  stats = stats_;
}
