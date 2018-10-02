// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancymap.h"

#include "mapcache.h"
#include "mapchunk.h"
#include "maplayer.h"
#include "mapprobability.h"
#include "occupancykeylist.h"
#include "occupancynode.h"
#include "occupancytype.h"

#include "gpucache.h"

#include "private/occupancymapdetail.h"

#include <ohmutil/linewalk.h>

#include <algorithm>
#include <cassert>
#ifdef OHM_VALIDATION
#include <cstdio>
#endif  // OHM_VALIDATION
#include <limits>
#include <utility>
#include "mapnode.h"
#include "ohmdefaultlayers.h"

using namespace ohm;

namespace
{
  // Iterator functions.
  bool nextLocalIndex(OccupancyKey &key, const glm::ivec3 &dim)
  {
    const glm::u8vec3 local = key.localKey();
    if (local.x + 1 < dim.x)
    {
      key.setLocalKey(glm::u8vec3(local.x + 1, local.y, local.z));
      return true;
    }
    if (local.y + 1 < dim.y)
    {
      key.setLocalKey(glm::u8vec3(0, local.y + 1, local.z));
      return true;
    }
    if (local.z + 1 < dim.z)
    {
      key.setLocalKey(glm::u8vec3(0, 0, local.z + 1));
      return true;
    }

    return false;
  }


  inline OccupancyKey firstKeyForChunk(const OccupancyMapDetail &map, const MapChunk &chunk)
  {
#ifdef OHM_VALIDATION
    chunk.validateFirstValid(map.regionVoxelDimensions);
#endif  // OHM_VALIDATION

    // We use std::min() to ensure the firstValidIndex is in range and we at least check
    // the last voxel. This primarily deals with iterating a chunk with contains no
    // valid voxels.
    return OccupancyKey(chunk.region.coord, std::min(chunk.firstValidIndex.x, uint8_t(map.regionVoxelDimensions.x - 1)),
                        std::min(chunk.firstValidIndex.y, uint8_t(map.regionVoxelDimensions.y - 1)),
                        std::min(chunk.firstValidIndex.z, uint8_t(map.regionVoxelDimensions.z - 1)));
    // if (nodeIndex(key, map.regionVoxelDimensions) >= map.regionVoxelDimensions.x * map.regionVoxelDimensions.y *
    // map.regionVoxelDimensions.z)
    // {
    //   // First valid index is out of range. This implies it has not been set correctly.
    //   // Modify the key to address voxel [0, 0, 0].
    //   key.setLocalKey(glm::u8vec3(0, 0, 0));
    // }
    // return key;
  }


  bool nextChunk(OccupancyMapDetail &map, ChunkMap::iterator &chunkIter, OccupancyKey &key)
  {
    ++chunkIter;
    if (chunkIter != map.chunks.end())
    {
      const MapChunk *chunk = chunkIter->second;
      key = firstKeyForChunk(map, *chunk);
      return true;
    }

    return false;
  }

  ChunkMap::iterator &initChunkIter(uint8_t *mem)
  {
    // Placement new.
    return *(new (mem) ChunkMap::iterator());
  }

  ChunkMap::iterator &chunkIter(uint8_t *mem) { return *reinterpret_cast<ChunkMap::iterator *>(mem); }

  const ChunkMap::iterator &chunkIter(const uint8_t *mem) { return *reinterpret_cast<const ChunkMap::iterator *>(mem); }

  void releaseChunkIter(uint8_t *mem)
  {
    using Iterator = ChunkMap::iterator;
    chunkIter(mem).~Iterator();
  }
}


OccupancyMap::base_iterator::base_iterator()
  : _map(nullptr)
  , _key(OccupancyKey::null)
{
  static_assert(sizeof(ChunkMap::iterator) <= sizeof(OccupancyMap::base_iterator::_chunkMem),
                "Insufficient space for chunk iterator.");
  initChunkIter(_chunkMem);
}


OccupancyMap::base_iterator::base_iterator(OccupancyMapDetail *map, const OccupancyKey &key)
  : _map(map)
  , _key(key)
{
  ChunkMap::iterator &chunkIter = initChunkIter(_chunkMem);
  if (!key.isNull())
  {
    std::unique_lock<decltype(map->mutex)> guard(map->mutex);
    chunkIter = map->findRegion(key.regionKey());
  }
}


OccupancyMap::base_iterator::base_iterator(const base_iterator &other)
  : _map(other._map)
  , _key(other._key)
{
  static_assert(sizeof(ChunkMap::iterator) <= sizeof(OccupancyMap::base_iterator::_chunkMem),
                "Insufficient space for chunk iterator.");
  initChunkIter(_chunkMem) = chunkIter(other._chunkMem);
}


OccupancyMap::base_iterator::~base_iterator()
{
  // Explicit destructor invocation.
  releaseChunkIter(_chunkMem);
}


OccupancyMap::base_iterator &OccupancyMap::base_iterator::operator=(const base_iterator &other)
{
  _map = other._map;
  _key = other._key;
  chunkIter(_chunkMem) = chunkIter(other._chunkMem);
  return *this;
}


bool OccupancyMap::base_iterator::operator==(const base_iterator &other) const
{
  // Chunks only have to match when not the end/invalid iterator.
  return _map == other._map && _key == other._key &&
         (_key.isNull() || chunkIter(_chunkMem) == chunkIter(other._chunkMem));
}


bool OccupancyMap::base_iterator::operator!=(const base_iterator &other) const
{
  return !(*this == other);
}


bool OccupancyMap::base_iterator::base_iterator::isValid() const
{
  return _map && !_key.isNull();
}


OccupancyNodeConst OccupancyMap::base_iterator::node() const
{
  const ChunkMap::iterator &iter = chunkIter(_chunkMem);
  return isValid() ? OccupancyNodeConst(_key, iter->second, _map) : OccupancyNodeConst();
}


void OccupancyMap::base_iterator::walkNext()
{
  if (!_key.isNull())
  {
    if (!nextLocalIndex(_key, _map->regionVoxelDimensions))
    {
      // Need to move to the next chunk.
      ChunkMap::iterator &chunk = chunkIter(_chunkMem);
      if (!nextChunk(*_map, chunk, _key))
      {
        // Invalidate.
        _key = OccupancyKey::null;
        releaseChunkIter(_chunkMem);
        initChunkIter(_chunkMem);
      }
    }
  }
}


OccupancyNode OccupancyMap::iterator::node()
{
  return isValid() ? OccupancyNode(_key, chunkIter(_chunkMem)->second, _map) : OccupancyNode();
}


OccupancyMap::OccupancyMap(double resolution, const glm::u8vec3 &regionVoxelDimensions)
  : _imp(new OccupancyMapDetail)
{
  _imp->resolution = resolution;
  _imp->regionVoxelDimensions.x = (regionVoxelDimensions.x > 0) ? regionVoxelDimensions.x : OHM_DEFAULT_CHUNK_DIM_X;
  _imp->regionVoxelDimensions.y = (regionVoxelDimensions.y > 0) ? regionVoxelDimensions.y : OHM_DEFAULT_CHUNK_DIM_Y;
  _imp->regionVoxelDimensions.z = (regionVoxelDimensions.z > 0) ? regionVoxelDimensions.z : OHM_DEFAULT_CHUNK_DIM_Z;
  _imp->regionSpatialDimensions.x = _imp->regionVoxelDimensions.x * resolution;
  _imp->regionSpatialDimensions.y = _imp->regionVoxelDimensions.y * resolution;
  _imp->regionSpatialDimensions.z = _imp->regionVoxelDimensions.z * resolution;
  _imp->saturateAtMinValue = _imp->saturateAtMaxValue = false;
  // Default thresholds taken from octomap as a guide.
  _imp->minNodeValue = -2.0f;
  _imp->maxNodeValue = 3.511f;
  setHitProbability(0.7f);
  setMissProbability(0.4f);
  setOccupancyThresholdProbability(0.5f);
  _imp->setDefaultLayout();
}


OccupancyMap::~OccupancyMap()
{
  if (_imp)
  {
    clear();
    delete _imp;
  }
}


size_t OccupancyMap::nodeMemoryPerRegion(glm::u8vec3 regionVoxelDimensions)
{
  regionVoxelDimensions.x = (regionVoxelDimensions.x > 0) ? regionVoxelDimensions.x : OHM_DEFAULT_CHUNK_DIM_X;
  regionVoxelDimensions.y = (regionVoxelDimensions.y > 0) ? regionVoxelDimensions.y : OHM_DEFAULT_CHUNK_DIM_Y;
  regionVoxelDimensions.z = (regionVoxelDimensions.z > 0) ? regionVoxelDimensions.z : OHM_DEFAULT_CHUNK_DIM_Z;
  const size_t volume = regionVoxelDimensions.x * regionVoxelDimensions.y * regionVoxelDimensions.z;
  // TODO: make more robust to change now there is no MapNode structure.
  const size_t bytes = volume * 2 * sizeof(float);
  // TODO: add size for coarseClearance array.
  return bytes;
}


OccupancyMap::iterator OccupancyMap::begin()
{
  return iterator(_imp, firstIterationKey());
}


OccupancyMap::const_iterator OccupancyMap::begin() const
{
  return const_iterator(_imp, firstIterationKey());
}


OccupancyMap::iterator OccupancyMap::end()
{
  return iterator(_imp, OccupancyKey::null);
}


OccupancyMap::const_iterator OccupancyMap::end() const
{
  return const_iterator(_imp, OccupancyKey::null);
}


namespace
{
  template <typename Node, typename Detail>
  Node getMapNode(Detail *detail, const OccupancyKey &key, const OccupancyMapDetail *map, bool allowCreat)
  {
    std::unique_lock<decltype(map->mutex)> guard(map->mutex);
    auto regionRef = detail->findRegion(key.regionKey());
    if (regionRef != detail->chunks.end())
    {
      return Node(key, regionRef->second, map);
    }
    return Node();
  }
}


OccupancyNode OccupancyMap::node(const OccupancyKey &key, bool allowCreate, MapCache *cache)
{
  MapChunk *chunk = (cache) ? cache->lookup(key) : nullptr;

  if (!chunk)
  {
    std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
    const auto regionRef = _imp->findRegion(key.regionKey());
    if (regionRef != _imp->chunks.end())
    {
      chunk = regionRef->second;
    }
    else if (allowCreate)
    {
      // No such chunk. Create one.
      chunk = newChunk(key);
      _imp->chunks.insert(std::make_pair(chunk->region.hash, chunk));
      // No need to touch the map here. We haven't changed the semantics of the map until
      // we change the value of a node in the region.
    }
  }

  if (chunk)
  {
    if (cache)
    {
      cache->push(chunk);
    }
    return OccupancyNode(key, chunk, _imp);
  }
  return OccupancyNode();
}


OccupancyNodeConst OccupancyMap::node(const OccupancyKey &key, MapCache *cache) const
{
  MapChunk *chunk = (cache) ? cache->lookup(key) : nullptr;

  if (!chunk)
  {
    std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
    const auto regionRef = _imp->findRegion(key.regionKey());
    if (regionRef != _imp->chunks.end())
    {
      chunk = regionRef->second;
      if (cache)
      {
        cache->push(chunk);
      }
    }
  }

  if (chunk)
  {
    return OccupancyNodeConst(key, chunk, _imp);
  }

  return OccupancyNodeConst();
}


int OccupancyMap::occupancyType(const OccupancyNodeConst &node) const
{
  if (!node.isNull())
  {
    const float value = node.value();
    if (value < NodeBase::invalidMarkerValue())
    {
      if (value < occupancyThresholdValue())
      {
        return Free;
      }

      return Occupied;
    }

    return Uncertain;
  }

  return Null;
}


size_t OccupancyMap::calculateApproximateMemory() const
{
  size_t byteCount = 0;
  byteCount += sizeof(this);
  if (_imp)
  {
    std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
    const glm::ivec3 dim = _imp->regionVoxelDimensions;
    const size_t nodesPerChunk = size_t(dim.x) * size_t(dim.y) * size_t(dim.z);

    byteCount += sizeof(OccupancyMapDetail);
    byteCount += _imp->chunks.size() * sizeof(MapChunk);
    byteCount += _imp->chunks.size() * 2 * sizeof(float) * nodesPerChunk;
    // TODO: consider coarseClearance array.

    // Approximate hash map usage.
    const size_t mapBuckedCount = _imp->chunks.bucket_count();
    const double mapLoad = _imp->chunks.max_load_factor();
    if (mapLoad > 1.0)
    {
      byteCount += size_t(mapBuckedCount * mapLoad * sizeof(MapChunk *));
    }
    else
    {
      byteCount += mapBuckedCount * sizeof(MapChunk *);
    }
  }

  return byteCount;
}


double OccupancyMap::resolution() const
{
  return _imp->resolution;
}


uint64_t OccupancyMap::stamp() const
{
  return _imp->stamp;
}


void OccupancyMap::touch()
{
  ++_imp->stamp;
}


glm::dvec3 OccupancyMap::regionSpatialResolution() const
{
  return _imp->regionSpatialDimensions;
}


glm::u8vec3 OccupancyMap::regionVoxelDimensions() const
{
  return _imp->regionVoxelDimensions;
}


size_t OccupancyMap::regionVoxelVolume() const
{
  size_t v = _imp->regionVoxelDimensions.x;
  v *= _imp->regionVoxelDimensions.y;
  v *= _imp->regionVoxelDimensions.z;
  return v;
}


glm::dvec3 OccupancyMap::regionSpatialMin(const glm::i16vec3 &regionKey) const
{
  const glm::dvec3 spatialMin = regionSpatialCentre(regionKey) - 0.5 * _imp->regionSpatialDimensions;
  return spatialMin;
}


glm::dvec3 OccupancyMap::regionSpatialMax(const glm::i16vec3 &regionKey) const
{
  const glm::dvec3 spatialMax = regionSpatialCentre(regionKey) + 0.5 * _imp->regionSpatialDimensions;
  return spatialMax;
}


glm::dvec3 OccupancyMap::regionSpatialCentre(const glm::i16vec3 &regionKey) const
{
  const glm::dvec3 centre(regionCentreCoord(regionKey.x, _imp->regionSpatialDimensions.x),
    regionCentreCoord(regionKey.y, _imp->regionSpatialDimensions.y),
    regionCentreCoord(regionKey.z, _imp->regionSpatialDimensions.z));
  return centre;
}


void OccupancyMap::setOrigin(const glm::dvec3 &origin)
{
  _imp->origin = origin;
}


const glm::dvec3 &OccupancyMap::origin() const
{
  return _imp->origin;
}


void OccupancyMap::calculateExtents(glm::dvec3 &minExt, glm::dvec3 &maxExt) const
{
  glm::dvec3 regionMin, regionMax;
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  minExt = maxExt = _imp->origin;
  for (auto &&chunk : _imp->chunks)
  {
    const MapRegion region = chunk.second->region;
    regionMin = regionMax = region.centre;
    regionMin -= 0.5 * regionSpatialResolution();
    regionMax += 0.5 * regionSpatialResolution();

    minExt.x = std::min(minExt.x, regionMin.x);
    minExt.y = std::min(minExt.y, regionMin.y);
    minExt.z = std::min(minExt.z, regionMin.z);

    maxExt.x = std::max(maxExt.x, regionMax.x);
    maxExt.y = std::max(maxExt.y, regionMax.y);
    maxExt.z = std::max(maxExt.z, regionMax.z);
  }
}


const MapLayout &OccupancyMap::layout() const
{
  return _imp->layout;
}


MapLayout &OccupancyMap::layout()
{
  return _imp->layout;
}


size_t OccupancyMap::regionCount() const
{
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  return _imp->chunks.size();
}


unsigned OccupancyMap::expireRegions(double timestamp)
{
  unsigned removedCount = 0;
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  auto regionIter = _imp->chunks.begin();
  MapChunk *chunk = nullptr;
  while (regionIter != _imp->chunks.end())
  {
    chunk = regionIter->second;
    if (chunk->touchedTime < timestamp)
    {
      // Remove from the map.
      regionIter = _imp->chunks.erase(regionIter);
      releaseChunk(chunk);
      ++removedCount;
    }
    else
    {
      // Next.
      ++regionIter;
    }
  }

  return removedCount;
}


unsigned OccupancyMap::removeDistanceRegions(const glm::dvec3 &relativeTo, float distance)
{
  unsigned removedCount = 0;
  const float distSqr = distance * distance;
  glm::vec3 separation;
  float regionDistanceSqr;
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  auto regionIter = _imp->chunks.begin();
  MapChunk *chunk = nullptr;
  while (regionIter != _imp->chunks.end())
  {
    chunk = regionIter->second;
    separation = chunk->region.centre - relativeTo;
    regionDistanceSqr = glm::dot(separation, separation);
    if (regionDistanceSqr > distSqr)
    {
      // Region too far. Remove from the map.
      regionIter = _imp->chunks.erase(regionIter);
      releaseChunk(chunk);
      ++removedCount;
    }
    else
    {
      // Next.
      ++regionIter;
    }
  }

  return removedCount;
}


void OccupancyMap::touchRegionByKey(const glm::i16vec3 &regionKey, double timestamp, bool allowCreate)
{
  MapChunk *chunk = region(regionKey, allowCreate);
  if (chunk)
  {
    chunk->touchedTime = timestamp;
  }
}


glm::dvec3 OccupancyMap::regionCentreGlobal(const glm::i16vec3 &regionKey) const
{
  return _imp->origin + regionCentreLocal(regionKey);
}


glm::dvec3 OccupancyMap::regionCentreLocal(const glm::i16vec3 &regionKey) const
{
  glm::dvec3 centre;
  centre.x = regionKey.x * _imp->regionSpatialDimensions.x;
  centre.y = regionKey.y * _imp->regionSpatialDimensions.y;
  centre.z = regionKey.z * _imp->regionSpatialDimensions.z;
  return centre;
}


glm::i16vec3 OccupancyMap::regionKey(const glm::dvec3 &point) const
{
  MapRegion region(point, _imp->origin, _imp->regionSpatialDimensions);
  return region.coord;
}


float OccupancyMap::hitValue() const
{
  return _imp->hitValue;
}


float OccupancyMap::hitProbability() const
{
  return _imp->hitProbability;
}


void OccupancyMap::setHitProbability(float probability)
{
  _imp->hitProbability = probability;
  ;
  _imp->hitValue = probabilityToValue(probability);
}


float OccupancyMap::missValue() const
{
  return _imp->missValue;
}


float OccupancyMap::missProbability() const
{
  return _imp->missProbability;
}


void OccupancyMap::setMissProbability(float probability)
{
  _imp->missProbability = probability;
  ;
  _imp->missValue = probabilityToValue(probability);
}


float OccupancyMap::occupancyThresholdValue() const
{
  return _imp->occupancyThresholdValue;
}


float OccupancyMap::occupancyThresholdProbability() const
{
  return _imp->occupancyThresholdProbability;
}


void OccupancyMap::setOccupancyThresholdProbability(float probability)
{
  _imp->occupancyThresholdProbability = probability;
  ;
  _imp->occupancyThresholdValue = probabilityToValue(probability);
}


OccupancyNode OccupancyMap::addNode(const OccupancyKey &key, float value)
{
  MapChunk *chunk;
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  const auto regionSearch = _imp->findRegion(key.regionKey());
  if (regionSearch == _imp->chunks.end())
  {
    // Allocate a new chunk.
    chunk = newChunk(key);
    _imp->chunks.insert(std::make_pair(chunk->region.hash, chunk));
    // No need to touch here. We haven't changed the semantics of the map at this point.
    // Also, the setValue() call below will touch the map.
  }
  else
  {
    chunk = regionSearch->second;
  }

  OccupancyNode node(key, chunk, _imp);
  // Set value through this function to ensure first valid index is maintained.
  node.setValue(value);
  return node;
}


float OccupancyMap::minNodeValue() const
{
  return _imp->minNodeValue;
}


void OccupancyMap::setMinNodeValue(float value)
{
  _imp->minNodeValue = value;
}


bool OccupancyMap::saturateAtMinValue() const
{
  return _imp->saturateAtMinValue;
}


void OccupancyMap::setSaturateAtMinValue(bool saturate)
{
  _imp->saturateAtMinValue = saturate;
}


float OccupancyMap::maxNodeValue() const
{
  return _imp->maxNodeValue;
}


void OccupancyMap::setMaxNodeValue(float value)
{
  _imp->maxNodeValue = value;
}


bool OccupancyMap::saturateAtMaxValue() const
{
  return _imp->saturateAtMaxValue;
}


void OccupancyMap::setSaturateAtMaxValue(bool saturate)
{
  _imp->saturateAtMaxValue = saturate;
}


glm::dvec3 OccupancyMap::voxelCentreLocal(const OccupancyKey &key) const
{
  glm::dvec3 centre;
  // Region centre
  centre = glm::vec3(key.regionKey());
  centre.x *= _imp->regionSpatialDimensions.x;
  centre.y *= _imp->regionSpatialDimensions.y;
  centre.z *= _imp->regionSpatialDimensions.z;
  // Offset to the lower extents of the region.
  centre -= 0.5 * _imp->regionSpatialDimensions;
  // Local offset.
  centre += glm::dvec3(key.localKey()) * _imp->resolution;
  centre += glm::dvec3(0.5 * _imp->resolution);
  return centre;
}


glm::dvec3 OccupancyMap::voxelCentreGlobal(const OccupancyKey &key) const
{
  glm::dvec3 centre;
  // Region centre
  centre = glm::dvec3(key.regionKey());
  centre.x *= _imp->regionSpatialDimensions.x;
  centre.y *= _imp->regionSpatialDimensions.y;
  centre.z *= _imp->regionSpatialDimensions.z;
  // Offset to the lower extents of the region.
  centre -= 0.5 * glm::dvec3(_imp->regionSpatialDimensions);
  // Map offset.
  centre += _imp->origin;
  // Local offset.
  centre += glm::dvec3(key.localKey()) * double(_imp->resolution);
  centre += glm::dvec3(0.5 * _imp->resolution);
  return centre;
}


OccupancyKey OccupancyMap::voxelKey(const glm::dvec3 &point) const
{
  OccupancyKey key;
  MapRegion region(point, _imp->origin, _imp->regionSpatialDimensions);
  // VALIDATION code ensures the region we calculate to contain the point does.
  // Floating point error was causing issues where it nearly, but not quite would.
#ifdef OHM_VALIDATION
  const bool voxelKeyOk =
#endif  // OHM_VALIDATION
    region.voxelKey(key, point, _imp->origin, _imp->regionSpatialDimensions, _imp->regionVoxelDimensions,
                    _imp->resolution);
#ifdef OHM_VALIDATION
  if (!voxelKeyOk)
  {
    fprintf(stderr, "E: Validation failure: Point (%lg %lg %lg) fell into a region which generated an invalid key.\n",
            point.x, point.y, point.z);
    fprintf(stderr, "  Point: %.20lf %.20lf %.20lf\n", point.x, point.y, point.z);
    fprintf(stderr, "  Map origin: %lf %lf %lf\n", _imp->origin.x, _imp->origin.y, _imp->origin.z);
    fprintf(stderr, "  Map resolution: %lf\n", _imp->resolution);
    fprintf(stderr, "  Region sizing: %lf %lf %lf\n", _imp->regionSpatialDimensions.x, _imp->regionSpatialDimensions.y,
            _imp->regionSpatialDimensions.z);
    fprintf(stderr, "  Region voxels: %d %d %d\n", _imp->regionVoxelDimensions.x, _imp->regionVoxelDimensions.y,
            _imp->regionVoxelDimensions.z);
    fprintf(stderr, "  Region coord: %d %d %d\n", region.coord.x, region.coord.y, region.coord.z);
    fprintf(stderr, "  Region centre: %lf %lf %lf\n", region.centre.x, region.centre.y, region.centre.z);
  }
#endif  // OHM_VALIDATION
  return key;
}


OccupancyKey OccupancyMap::voxelKey(const glm::vec3 &point) const
{
  OccupancyKey key;
  MapRegion region(point, _imp->origin, _imp->regionSpatialDimensions);
  region.voxelKey(key, point, _imp->origin, _imp->regionSpatialDimensions, _imp->regionVoxelDimensions,
                  _imp->resolution);
  return key;
}


OccupancyKey OccupancyMap::voxelKeyLocal(const glm::vec3 &localPoint) const
{
  OccupancyKey key;
  const glm::dvec3 zeroOrigin(0, 0, 0);
  MapRegion region(localPoint, zeroOrigin, _imp->regionSpatialDimensions);
  region.voxelKey(key, localPoint, zeroOrigin, _imp->regionSpatialDimensions, _imp->regionVoxelDimensions,
                  _imp->resolution);
  return key;
}


void OccupancyMap::moveKeyAlongAxis(OccupancyKey &key, int axis, int step) const
{
  const glm::ivec3 LocalLimits = _imp->regionVoxelDimensions;

  if (step == 0)
  {
    // No change.
    return;
  }

  // We first step within the chunk region. If we can't then we step the region and reset
  // stepped local axis value.
  glm::i16vec3 regionKey = key.regionKey();
  glm::ivec3 localKey = key.localKey();
  if (step > 0)
  {
    // Positive step.
    localKey[axis] += step;
    regionKey[axis] += localKey[axis] / LocalLimits[axis];
    localKey[axis] %= LocalLimits[axis];
  }
  else
  {
    // Negative step.
    localKey[axis] += step;
    // Create a region step which simulates a floating point floor.
    regionKey[axis] += ((localKey[axis] - (LocalLimits[axis] - 1)) / LocalLimits[axis]);
    if (localKey[axis] < 0)
    {
      // This is nuts. In C/C++, the % operator is not actually a modulus operator.
      // It's a "remainder" operator. A modulus operator should only give positive results,
      // but in C a negative input will generate a negative output. Through the magic of
      // StackOverflow, here's a good explanation:
      //  https://stackoverflow.com/questions/11720656/modulo-operation-with-negative-numbers
      // This means that here, given localKey[axis] is negative, the modulus:
      //    localKey[axis] % LocalLimits[axis]
      // will give results in the range (-LocalLimits[axis], 0]. So, lets set the limit
      // to 4, then we get the following: like follows:
      //
      // i  i%4   4 - i%4
      //  0  0    4
      // -1 -1    3
      // -2 -2    2
      // -3 -3    1
      // -4  0    4
      // -5 -1    3
      // -6 -2    2
      // -7 -3    1
      // -8  0    4
      //
      // The last column above shows the results of the following line of code.
      // This generates the wrong results in that the '4' results in the last
      // column should be 0. We could apply another "% LocalLimits[axis]" to
      // the result or just add the if statement below.
      localKey[axis] = LocalLimits[axis] + localKey[axis] % LocalLimits[axis];
      if (localKey[axis] == LocalLimits[axis])
      {
        localKey[axis] = 0;
      }
    }
    // else
    // {
    //   localKey[axis] %= LocalLimits[axis];
    // }
  }

  key = OccupancyKey(regionKey, localKey.x, localKey.y, localKey.z);
}


void OccupancyMap::stepKey(OccupancyKey &key, int axis, int dir) const
{
  int localKey = key.localKey()[axis] + dir;
  int regionKey = key.regionKey()[axis];

  if (localKey < 0)
  {
    --regionKey;
    localKey = _imp->regionVoxelDimensions[axis] - 1;
  }
  else if (localKey >= _imp->regionVoxelDimensions[axis])
  {
    ++regionKey;
    localKey = 0;
  }

  key.setLocalAxis(axis, (uint8_t)localKey);
  key.setRegionAxis(axis, (uint16_t)regionKey);
}


void OccupancyMap::moveKey(OccupancyKey &key, int x, int y, int z) const
{
  moveKeyAlongAxis(key, 0, x);
  moveKeyAlongAxis(key, 1, y);
  moveKeyAlongAxis(key, 2, z);
}


size_t OccupancyMap::calculateSegmentKeys(OccupancyKeyList &keys, const glm::dvec3 &startPoint,
                                          const glm::dvec3 &endPoint, bool includeEndPoint) const
{
  struct KeyAdaptor
  {
    const OccupancyMap &map;

    inline KeyAdaptor(const OccupancyMap &map)
      : map(map)
    {}

    inline OccupancyKey voxelKey(const glm::dvec3 &pt) const { return map.voxelKey(pt); }
    inline bool isNull(const OccupancyKey &key) const { return key.isNull(); }
    inline glm::dvec3 voxelCentre(const OccupancyKey &key) const { return map.voxelCentreLocal(key); }
    inline void stepKey(OccupancyKey &key, int axis, int dir) const { map.stepKey(key, axis, dir); }
    inline double voxelResolution(int /*axis*/) const { return map.resolution(); }
  };
  const glm::dvec3 startPointLocal = glm::dvec3(startPoint - origin());
  const glm::dvec3 endPointLocal = glm::dvec3(endPoint - origin());


  keys.clear();
  return ohmutil::walkSegmentKeys<OccupancyKey>([&keys](const OccupancyKey &key) { keys.add(key); },
                                                startPointLocal, endPointLocal, includeEndPoint,
                                                KeyAdaptor(*this));
}


void OccupancyMap::integrateRays(const glm::dvec3 *rays, size_t pointCount, bool endPointsAsOccupied)
{
  OccupancyKeyList keys;
  MapCache cache;

  for (size_t i = 0; i < pointCount; i += 2)
  {
    calculateSegmentKeys(keys, rays[i], rays[i + 1], false);

    for (auto &&key : keys)
    {
      integrateMiss(key, &cache);
    }

    if (endPointsAsOccupied)
    {
      integrateHit(voxelKey(rays[i + 1]), &cache);
    }
    else
    {
      integrateMiss(voxelKey(rays[i + 1]), &cache);
    }
  }
}


OccupancyMap *OccupancyMap::clone() const
{
  return clone(-glm::dvec3(std::numeric_limits<double>::infinity()),
               glm::dvec3(std::numeric_limits<double>::infinity()));
}


OccupancyMap *OccupancyMap::clone(const glm::dvec3 &minExt, const glm::dvec3 &maxExt) const
{
  OccupancyMap *newMap = new OccupancyMap(_imp->resolution, _imp->regionVoxelDimensions);

  // Copy general details.
  newMap->detail()->copyFrom(*_imp);

  glm::dvec3 regionMin, regionMax;
  const glm::dvec3 regionHalfExt = 0.5 * _imp->regionSpatialDimensions;
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  for (const auto chunkIter : _imp->chunks)
  {
    const MapChunk *srcChunk = chunkIter.second;
    regionMin = regionMax = srcChunk->region.centre;
    regionMin -= regionHalfExt;
    regionMax += regionHalfExt;

    if (!glm::any(glm::lessThan(regionMax, minExt)) &&
        !glm::any(glm::greaterThan(regionMin, maxExt)))
    {
      MapChunk *dstChunk = newMap->region(srcChunk->region.coord, true);
      dstChunk->firstValidIndex = srcChunk->firstValidIndex;
      dstChunk->touchedTime = srcChunk->touchedTime;
      dstChunk->dirtyStamp = srcChunk->dirtyStamp;
      dstChunk->flags = srcChunk->flags;

      for (unsigned i = 0; i < _imp->layout.layerCount(); ++i)
      {
        const MapLayer &layer = srcChunk->layout->layer(i);
        dstChunk->touchedStamps[i] = static_cast<uint64_t>(srcChunk->touchedStamps[i]);
        if (srcChunk->voxelMaps[i])
        {
          memcpy(dstChunk->voxelMaps[i], srcChunk->voxelMaps[i], layer.layerByteSize(_imp->regionVoxelDimensions));
        }
      }
    }
  }

  return newMap;
}


void OccupancyMap::enumerateRegions(std::vector<const MapChunk *> &chunks) const
{
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  for (auto &&chunkIter : _imp->chunks)
  {
    chunks.push_back(chunkIter.second);
  }
}


MapChunk *OccupancyMap::region(const glm::i16vec3 &regionKey, bool allowCreate)
{
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  const auto regionSearch = _imp->findRegion(regionKey);
  if (regionSearch != _imp->chunks.end())
  {
    MapChunk *chunk = regionSearch->second;
#ifdef OHM_VALIDATION
    chunk->validateFirstValid(_imp->regionVoxelDimensions);
#endif  // OHM_VALIDATION
    return chunk;
  }

  if (allowCreate)
  {
    // No such chunk. Create one.
    MapChunk *chunk = newChunk(OccupancyKey(regionKey, 0, 0, 0));
    _imp->chunks.insert(std::make_pair(chunk->region.hash, chunk));
    // No need to touch the map here. We haven't changed the semantics of the map.
    // That happens when the value of a node in the region changes.
    return chunk;
  }

  return nullptr;
}


const MapChunk *OccupancyMap::region(const glm::i16vec3 &regionKey) const
{
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  const auto regionSearch = _imp->findRegion(regionKey);
  if (regionSearch != _imp->chunks.end())
  {
    const MapChunk *chunk = regionSearch->second;
    return chunk;
  }

  return nullptr;
}


unsigned OccupancyMap::collectDirtyRegions(uint64_t fromStamp, std::vector<std::pair<uint64_t, glm::i16vec3>> &regions) const
{
  // Brute for for now.
  unsigned addedCount = 0;
  bool added;
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  for (auto &&chunkRef : _imp->chunks)
  {
    if (chunkRef.second->dirtyStamp > fromStamp)
    {
      added = false;

      // Insertion sorted on the chunk's dirty stamp. Least recently touched (oldtest) first.
      // TODO: test efficiency of the sorted insertion on a vector.
      // Scope should be small so I expect little impact.
      auto item = std::make_pair(chunkRef.second->dirtyStamp, chunkRef.second->region.coord);
      for (auto iter = regions.begin(); iter != regions.end(); ++iter)
      {
        if (item.first < iter->first)
        {
          regions.insert(iter, item);
          added = true;
          break;
        }
      }

      if (!added)
      {
        regions.push_back(item);
      }

      ++addedCount;
    }
  }

  return addedCount;
}


void OccupancyMap::calculateDirtyExtents(uint64_t *fromStamp, glm::i16vec3 *minExt, glm::i16vec3 *maxExt) const
{
  *minExt = glm::i16vec3(std::numeric_limits<decltype(minExt->x)>::max());
  *maxExt = glm::i16vec3(std::numeric_limits<decltype(minExt->x)>::min());

  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  const uint64_t atStamp = _imp->stamp;
  for (auto &&chunkRef : _imp->chunks)
  {
    if (chunkRef.second->dirtyStamp > *fromStamp)
    {
      minExt->x = std::min(chunkRef.second->region.coord.x, minExt->x);
      minExt->y = std::min(chunkRef.second->region.coord.y, minExt->y);
      minExt->z = std::min(chunkRef.second->region.coord.z, minExt->z);

      maxExt->x = std::max(chunkRef.second->region.coord.x, maxExt->x);
      maxExt->y = std::max(chunkRef.second->region.coord.y, maxExt->y);
      maxExt->z = std::max(chunkRef.second->region.coord.z, maxExt->z);
    }
  }
  guard.unlock();

  if (minExt->x > maxExt->x)
  {
    *minExt = glm::i16vec3(1);
    *maxExt = glm::i16vec3(0);
  }
  *fromStamp = atStamp;
}

void OccupancyMap::calculateDirtyClearanceExtents(glm::i16vec3 *minExt, glm::i16vec3 *maxExt, unsigned regionPadding) const
{
  *minExt = glm::i16vec3(std::numeric_limits<decltype(minExt->x)>::max());
  *maxExt = glm::i16vec3(std::numeric_limits<decltype(minExt->x)>::min());

  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  const uint64_t atStamp = _imp->stamp;
  for (auto &&chunkRef : _imp->chunks)
  {
    if (chunkRef.second->touchedStamps[DL_Clearance] < chunkRef.second->touchedStamps[DL_Occupancy])
    {
      minExt->x = std::min<int>(chunkRef.second->region.coord.x - regionPadding, minExt->x);
      minExt->y = std::min<int>(chunkRef.second->region.coord.y - regionPadding, minExt->y);
      minExt->z = std::min<int>(chunkRef.second->region.coord.z - regionPadding, minExt->z);

      maxExt->x = std::max<int>(chunkRef.second->region.coord.x + regionPadding, maxExt->x);
      maxExt->y = std::max<int>(chunkRef.second->region.coord.y + regionPadding, maxExt->y);
      maxExt->z = std::max<int>(chunkRef.second->region.coord.z + regionPadding, maxExt->z);
    }
  }
  guard.unlock();

  if (minExt->x > maxExt->x)
  {
    *minExt = glm::i16vec3(1);
    *maxExt = glm::i16vec3(0);
  }
}


void OccupancyMap::clear()
{
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  for (auto &&chunkRef : _imp->chunks)
  {
    releaseChunk(chunkRef.second);
  }
  _imp->chunks.clear();
}


OccupancyKey OccupancyMap::firstIterationKey() const
{
  std::unique_lock<decltype(_imp->mutex)> guard(_imp->mutex);
  auto firstChunkIter = _imp->chunks.begin();
  if (firstChunkIter != _imp->chunks.end())
  {
    MapChunk *chunk = firstChunkIter->second;
    return firstKeyForChunk(*_imp, *chunk);
  }

  return OccupancyKey::null;
}


MapChunk *OccupancyMap::newChunk(const OccupancyKey &forKey)
{
  MapChunk *chunk = new MapChunk(MapRegion(voxelCentreGlobal(forKey), _imp->origin, _imp->regionSpatialDimensions),
                                 _imp->layout, _imp->regionVoxelDimensions);
  return chunk;
}


void OccupancyMap::releaseChunk(const MapChunk *chunk)
{
  delete chunk;
}
