// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUCACHE_H_
#define GPUCACHE_H_

#include "ohmconfig.h"

#include <cstddef>

namespace gputil
{
  class Device;
  class Queue;
}

namespace ohm
{
  struct GpuCacheDetail;
  struct GpuCacheParams;
  class GpuLayerCache;
  class OccupancyMap;

  /// IDs for GPU caches.
  enum GpuCacheID
  {
    /// Cache used for populating the map occupancy values.
    GCID_Occupancy,
    /// Cache of occupancy map values when calculating voxel clearance values. Does not write back to host.
    GCID_Clearance,
  };

  /// Provides access to the @c GpuLayerCache objects used to cache host voxel data in GPU memory and manage
  /// synchronisation.
  ///
  /// The GPU cache contains a number of @c GpuLayerCache objects, each of which is bound to a particular @c MapLayer.
  /// Each layer cache is initialised in the same way; sized to @c layerGpuMemSize() bytes and optionally mappable
  /// (@c mappableBuffers()) to host memory.
  ///
  /// @c GpuLayerCache objects are instantiates by @c layerCache(). Each cache is identified by user defined ID
  /// and set to cache data from a specific @c MapLayer.
  ///
  /// The @c GpuCache also defines the @c gputil::Device and @p gputil::Queue associated with GPU operations.
  ///
  /// For more information on layer cache usage see @c GpuLayerCache.
  class GpuCache
  {
  public:
    /// Define for 1 MiB in bytes.
    static const size_t MiB = 1024u * 1024u;
    /// Define for 1 GiB in bytes.
    static const size_t GiB = 1024u * 1024u * 1024u;
    /// The default byte size of each GPU layer if not specified.
    static const size_t DefaultLayerMemSize = GiB / 2;

    /// Instantiate the @c GpuCache for @p map.
    /// @param map The map to cache data for.
    /// @param defaultGpuMemSize The default size of allocated @c GpuLayerCache objects if not specified.
    GpuCache(OccupancyMap &map, size_t defaultGpuMemSize = DefaultLayerMemSize);

    /// Destructor, cleaning up all owned @c GpuLayerCache objects.
    ~GpuCache();

    /// Query the default byte size of each layer cache.
    /// @return The default size of for a layer cache in bytes.
    size_t defaultGpuMemSize() const;

    /// Returns the number of indexable layers. Some may be null.
    /// @return The number of indexable layers.
    unsigned layerCount() const;

    /// Creates a new @c GpuLayerCache with the specified @p params.
    ///
    /// The @p id should always be in a relative low range as a @c vector is used to allocate and access
    /// cache pointers.
    ///
    /// See @c GpuCacheParams for configuration details.
    ///
    /// Fails when:
    /// - A layer cache with @p id already exists.
    ///
    /// @param id The unique ID for the cache.
    /// @param params Cache configuration.
    /// @return A pointer to the new cache on success, null on failure.
    GpuLayerCache *createCache(unsigned id, const GpuCacheParams &params);

    /// Request or create a new @c GpuLayerCache.
    ///
    /// The cache is uniquely identified by the specified @p id, the sematics of which depend on usage. A default
    /// set of IDs is defined in @@@. The call also specifies a @p layer which identifies the @c MapLayer associated
    /// with the cache.
    GpuLayerCache *layerCache(unsigned id);

    /// Access the GPU @c gputil::Device associated with GPU operations.
    /// @return The bound @c gputil::Device.
    gputil::Device &gpu();
    /// @overload
    const gputil::Device &gpu() const;
    /// Access the GPU @c gputil::Queue associated with GPU operations.
    /// @return The bound @c gputil::Queue.
    gputil::Queue &gpuQueue();
    /// @overload
    const gputil::Queue &gpuQueue() const;

  private:
    GpuCacheDetail *_imp;
  };
}

#endif // GPUCACHE_H_
