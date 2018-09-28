//
// Author: Kazys Stepanas
//
#ifndef OCCUPANCYMAPDETAIL_H_
#define OCCUPANCYMAPDETAIL_H_

#include "ohmconfig.h"

#include <glm/glm.hpp>

#include "occupancynode.h"
#include "mapchunk.h"
#include "maplayout.h"
#include "mapregion.h"

#include <mutex>
#include <unordered_map>

namespace ohm
{
  typedef std::unordered_multimap<unsigned, MapChunk *> ChunkMap;

  class GpuCache;
  class OccupancyMap;

  struct OccupancyMapDetail
  {
    glm::dvec3 origin = glm::dvec3(0);
    glm::dvec3 regionSpatialDimensions = glm::dvec3(0);
    glm::u8vec3 regionVoxelDimensions = glm::u8vec3(0);
    double resolution = 0.0;
    uint64_t stamp = 0;
    float occupancyThresholdValue = 0.0f;
    float occupancyThresholdProbability = 0.0f;
    float hitValue = 0.0f;
    float hitProbability = 0.0f;
    float missValue = 0.0f;
    float missProbability = 0.0f;
    float minNodeValue = 0.0f;
    float maxNodeValue = 0.0f;
    bool saturateAtMinValue = false;
    bool saturateAtMaxValue = false;
    MapLayout layout;
    ChunkMap chunks;
    mutable std::mutex mutex;

    GpuCache *gpuCache = nullptr;

    ~OccupancyMapDetail();

    /// A helper function for finding the @c MapChunk for the given @p regionKey.
    /// Deals with having regions with the same hash in the map (though unlikely).
    /// @param regionKey They key for the region of interest.
    /// @return The interator in @c chunks to the region of interest or @c chunks.end() when not found.
    ChunkMap::iterator findRegion(const glm::i16vec3 &regionKey);

    /// @overload
    ChunkMap::const_iterator findRegion(const glm::i16vec3 &regionKey) const;

    void setDefaultLayout();

    /// Copy internal details from @p other. For cloning.
    /// @param other The map detail to copy from.
    void copyFrom(const OccupancyMapDetail &other);

  protected:
    template <typename Iter, typename T>
    static Iter findRegion(T &chunks, const glm::i16vec3 &regionKey);
  };


  inline ChunkMap::iterator OccupancyMapDetail::findRegion(const glm::i16vec3 &regionKey)
  {
    return findRegion<ChunkMap::iterator>(chunks, regionKey);
  }


  inline ChunkMap::const_iterator OccupancyMapDetail::findRegion(const glm::i16vec3 &regionKey) const
  {
    return findRegion<ChunkMap::const_iterator>(chunks, regionKey);
  }


  template <typename Iter, typename T>
  inline Iter OccupancyMapDetail::findRegion(T &chunks, const glm::i16vec3 &regionKey)
  {
    const unsigned regionHash = MapRegion::Hash::calculate(regionKey);
    Iter iter = chunks.find(regionHash);
    while (iter != chunks.end() && iter->first == regionHash && iter->second->region.coord != regionKey)
    {
      ++iter;
    }

    if (iter != chunks.end() && iter->first == regionHash && iter->second->region.coord == regionKey)
    {
      return iter;
    }

    return chunks.end();
  }
}

#endif // OCCUPANCYMAPDETAIL_H_
