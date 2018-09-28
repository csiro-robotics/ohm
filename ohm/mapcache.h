//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef MAPCACHE_H_
#define MAPCACHE_H_

#include "ohmconfig.h"

#define OHM_MULTI_CACHE 0

namespace ohm
{
  struct MapChunk;
  class OccupancyKey;

  /// A cache object used to speed up some lookups into an @c OccupancyMap.
  class ohm_API MapCache
  {
  public:
    struct Stats
    {
      unsigned miss;
      unsigned hit;
    };

    MapCache(unsigned cacheSize = 16);
    ~MapCache();

#if OHM_MULTI_CACHE
    inline unsigned cacheSize() const { return _cacheSize; }
#else  // OHM_MULTI_CACHE
    inline unsigned cacheSize() const { return 1; }
#endif // OHM_MULTI_CACHE

    MapChunk *lookup(const OccupancyKey &key);
    void push(MapChunk *chunk);
    void clear();

    void getStats(Stats &stats) const;

  private:
#if OHM_MULTI_CACHE
    MapChunk **_chunks;
    unsigned _cacheSize;
#else  // OHM_MULTI_CACHE
    MapChunk *_chunk;
#endif // OHM_MULTI_CACHE
    Stats _stats;
  };
}

#endif // MAPCACHE_H_
