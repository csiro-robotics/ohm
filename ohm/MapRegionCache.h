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

  /// Base class for any memory cache responsible for ensuring map regions are available in the correct memory.
  ///
  /// Initially this class is simply a placeholder base class for the GPU cache. In future it map be extended to
  /// maintain an in-core, uncompressed region cache with compressed data attached to @c MapRegion objects.
  class ohm_API MapRegionCache
  {
  public:
    MapRegionCache();
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

  private:
    std::unique_ptr<MapRegionCacheDetail> imp_;
  };
}  // namespace ohm

#endif  // OHM_MAPREGIONCACHE_H
