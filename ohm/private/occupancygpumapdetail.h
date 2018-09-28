// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYGPUMAPDETAIL_H_
#define OCCUPANCYGPUMAPDETAIL_H_

#include "ohmconfig.h"

#include "gpucache.h"
#include "ohmdefaultlayers.h"

#include <glm/glm.hpp>

#include <gpubuffer.h>
#include <gpuevent.h>

#include <unordered_map>

namespace ohm
{
  class OccupancyMap;

  struct GpuMapDetail
  {
    enum { BUFFERS_COUNT = 2 };
    OccupancyMap *map;
    typedef std::unordered_multimap<unsigned, glm::i16vec3> RegionKeyMap;
    gputil::Event rayUploadEvents[BUFFERS_COUNT];
    gputil::Buffer rayBuffers[BUFFERS_COUNT];

    gputil::Event regionKeyUploadEvents[BUFFERS_COUNT];
    gputil::Event regionOffsetUploadEvents[BUFFERS_COUNT];
    gputil::Buffer regionKeyBuffers[BUFFERS_COUNT];
    gputil::Buffer regionOffsetBuffers[BUFFERS_COUNT];

    gputil::Event regionUpdateEvents[BUFFERS_COUNT];

    double maxRangeFilter;

    unsigned rayCounts[BUFFERS_COUNT];
    unsigned regionCounts[BUFFERS_COUNT];

    int nextBuffersIndex;
    // Should be a multi-map in case of hash clashes.
    RegionKeyMap regions;
    /// Used as @c GpuLayerCache::upload() @c batchMarker argument.
    unsigned batchMarker;
    bool borrowedMap;
    bool gpuOk;

    inline GpuMapDetail(OccupancyMap *map, bool borrowedMap)
      : map(map)
      , maxRangeFilter(0)
      , nextBuffersIndex(0)
      , batchMarker(1)  // Will cycle odd numbers to avoid zero.
      , borrowedMap(borrowedMap)
      , gpuOk(false)
    {
      rayCounts[0] = rayCounts[1] = regionCounts[0] = regionCounts[1] = 0;
    }

    RegionKeyMap::iterator findRegion(const unsigned regionHash, const glm::i16vec3 &regionKey);
    RegionKeyMap::const_iterator findRegion(const unsigned regionHash, const glm::i16vec3 &regionKey) const;

  protected:
    template <typename Iter, typename T>
    static Iter findRegion(T &regions, const unsigned regionHash, const glm::i16vec3 &regionKey);
  };


  /// Ensure the GPU cache is initialised. Ok to call if already initialised.
  GpuCache *initialiseGpuCache(OccupancyMap &map, size_t layerGpuMemSize, bool mappableBuffers);

  inline GpuMapDetail::RegionKeyMap::iterator GpuMapDetail::findRegion(const unsigned regionHash, const glm::i16vec3 &regionKey)
  {
    return findRegion<RegionKeyMap::iterator>(regions, regionHash, regionKey);
  }


  inline GpuMapDetail::RegionKeyMap::const_iterator GpuMapDetail::findRegion(const unsigned regionHash, const glm::i16vec3 &regionKey) const
  {
    return findRegion<RegionKeyMap::const_iterator>(regions, regionHash, regionKey);
  }

  template <typename Iter, typename T>
  Iter GpuMapDetail::findRegion(T &regions, const unsigned regionHash, const glm::i16vec3 &regionKey)
  {
    Iter iter = regions.find(regionHash);
    while (iter != regions.end() && iter->first == regionHash && iter->second != regionKey)
    {
      ++iter;
    }

    if (iter != regions.end() && iter->first == regionHash && iter->second == regionKey)
    {
      return iter;
    }

    return regions.end();
  }
}

#endif // OCCUPANCYGPUMAPDETAIL_H_
