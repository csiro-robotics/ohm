//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OHM_MAPCACHE_H
#define OHM_MAPCACHE_H

#include "OhmConfig.h"

#define OHM_MULTI_CACHE 0

namespace ohm
{
  struct MapChunk;
  class Key;

  /// A cache object used to speed up some lookups into an @c OccupancyMap.
  ///
  /// When passed to supporting @c OccupanyMap methods, the cache may speedup finding the correct region to update.
  /// The implementation assumes that subsequent calls will have some spatial consistency, thus the default implementation
  /// cache simply tracks the last region used. Multiple regions may be tracked by defining @c OHM_MULTI_CACHE.
  ///
  /// General usage is to maintain a @c MapCache as a stack allocated object and use it for a specific set of
  /// queries on the @c OccupancyMap. The @p MapCache is not threadsafe.
  class ohm_API MapCache
  {
  public:
    struct Stats
    {
      unsigned miss;
      unsigned hit;
    };

    /// Instantiate the cache.
    /// @param cache_size The size of the cache when @c OHM_MULTI_CACHE is defined as non-zero.
    MapCache(unsigned cache_size = 16);
    ~MapCache();

    /// @fn unsigned cacheSize() const
    /// Query the cache size.
    /// @return The cache size.
#if OHM_MULTI_CACHE
    inline unsigned cacheSize() const { return _cacheSize; }
#else  // OHM_MULTI_CACHE
    inline unsigned cacheSize() const { return 1; }
#endif // OHM_MULTI_CACHE

    /// Lookup the given @p key in the cache. This returns the @c MapChunk for the @p key if it is present in the
    /// cache. Otherwise a cache miss occurs and @c null is returned.
    ///
    /// @param key The map key to lookup.
    /// @return The @c MapChunk which @p key references if it is cached, or null otherwise.
    MapChunk *lookup(const Key &key);

    /// Push the given @p chunk into the cache. The user must ensure the @p chunk remains value for
    /// the life of the @c MapCache.
    ///
    /// @param chunk The chunk to cache.
    void push(MapChunk *chunk);

    /// Clear the cache.
    void clear();

    void getStats(Stats &stats) const;

  private:
#if OHM_MULTI_CACHE
    MapChunk **_chunks;
    unsigned _cacheSize;
#else  // OHM_MULTI_CACHE
    MapChunk *chunk_;
#endif // OHM_MULTI_CACHE
    Stats stats_;
  };
}

#endif // OHM_MAPCACHE_H
