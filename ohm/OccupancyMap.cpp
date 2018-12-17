// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OccupancyMap.h"

#include "Aabb.h"
#include "DefaultLayer.h"
#include "KeyList.h"
#include "MapCache.h"
#include "MapChunk.h"
#include "MapCoord.h"
#include "MapLayer.h"
#include "MapProbability.h"
#include "OccupancyType.h"
#include "Voxel.h"

#include "GpuCache.h"

#include "private/OccupancyMapDetail.h"

#include <ohmutil/LineWalk.h>

#include <algorithm>
#include <cassert>
#ifdef OHM_VALIDATION
#include <cstdio>
#endif  // OHM_VALIDATION
#include <functional>
#include <limits>
#include <utility>


using namespace ohm;

namespace
{
  inline Key firstKeyForChunk(const OccupancyMapDetail &map, const MapChunk &chunk)
  {
#ifdef OHM_VALIDATION
    chunk.validateFirstValid(map.region_voxel_dimensions);
#endif  // OHM_VALIDATION

    // We use std::min() to ensure the first_valid_index is in range and we at least check
    // the last voxel. This primarily deals with iterating a chunk with contains no
    // valid voxels.
    return Key(chunk.region.coord, std::min(chunk.first_valid_index.x, uint8_t(map.region_voxel_dimensions.x - 1)),
               std::min(chunk.first_valid_index.y, uint8_t(map.region_voxel_dimensions.y - 1)),
               std::min(chunk.first_valid_index.z, uint8_t(map.region_voxel_dimensions.z - 1)));
    // if (voxelIndex(key, map.region_voxel_dimensions) >= map.region_voxel_dimensions.x * map.region_voxel_dimensions.y
    // * map.region_voxel_dimensions.z)
    // {
    //   // First valid index is out of range. This implies it has not been set correctly.
    //   // Modify the key to address voxel [0, 0, 0].
    //   key.setLocalKey(glm::u8vec3(0, 0, 0));
    // }
    // return key;
  }


  bool nextChunk(OccupancyMapDetail &map, ChunkMap::iterator &chunk_iter, Key &key)
  {
    ++chunk_iter;
    if (chunk_iter != map.chunks.end())
    {
      const MapChunk *chunk = chunk_iter->second;
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
}  // namespace


OccupancyMap::base_iterator::base_iterator()
  : map_(nullptr)
  , key_(Key::kNull)
{
  static_assert(sizeof(ChunkMap::iterator) <= sizeof(OccupancyMap::base_iterator::chunk_mem_),
                "Insufficient space for chunk iterator.");
  initChunkIter(chunk_mem_);
}


OccupancyMap::base_iterator::base_iterator(OccupancyMapDetail *map, const Key &key)
  : map_(map)
  , key_(key)
{
  ChunkMap::iterator &chunk_iter = initChunkIter(chunk_mem_);
  if (!key.isNull())
  {
    std::unique_lock<decltype(map->mutex)> guard(map->mutex);
    chunk_iter = map->findRegion(key.regionKey());
  }
}


OccupancyMap::base_iterator::base_iterator(const base_iterator &other)
  : map_(other.map_)
  , key_(other.key_)
{
  static_assert(sizeof(ChunkMap::iterator) <= sizeof(OccupancyMap::base_iterator::chunk_mem_),
                "Insufficient space for chunk iterator.");
  initChunkIter(chunk_mem_) = chunkIter(other.chunk_mem_);
}


OccupancyMap::base_iterator::~base_iterator()
{
  // Explicit destructor invocation.
  releaseChunkIter(chunk_mem_);
}


OccupancyMap::base_iterator &OccupancyMap::base_iterator::operator=(const base_iterator &other)
{
  map_ = other.map_;
  key_ = other.key_;
  chunkIter(chunk_mem_) = chunkIter(other.chunk_mem_);
  return *this;
}


bool OccupancyMap::base_iterator::operator==(const base_iterator &other) const
{
  // Chunks only have to match when not the end/invalid iterator.
  return map_ == other.map_ && key_ == other.key_ &&
         (key_.isNull() || chunkIter(chunk_mem_) == chunkIter(other.chunk_mem_));
}


bool OccupancyMap::base_iterator::operator!=(const base_iterator &other) const
{
  return !(*this == other);
}


bool OccupancyMap::base_iterator::base_iterator::isValid() const
{
  return map_ && !key_.isNull();
}


VoxelConst OccupancyMap::base_iterator::voxel() const
{
  const ChunkMap::iterator &iter = chunkIter(chunk_mem_);
  return isValid() ? VoxelConst(key_, iter->second, map_) : VoxelConst();
}


void OccupancyMap::base_iterator::walkNext()
{
  if (!key_.isNull())
  {
    if (!nextLocalKey(key_, map_->region_voxel_dimensions))
    {
      // Need to move to the next chunk.
      ChunkMap::iterator &chunk = chunkIter(chunk_mem_);
      if (!nextChunk(*map_, chunk, key_))
      {
        // Invalidate.
        key_ = Key::kNull;
        releaseChunkIter(chunk_mem_);
        initChunkIter(chunk_mem_);
      }
    }
  }
}


Voxel OccupancyMap::iterator::voxel()
{
  return isValid() ? Voxel(key_, chunkIter(chunk_mem_)->second, map_) : Voxel();
}


OccupancyMap::OccupancyMap(double resolution, const glm::u8vec3 &region_voxel_dimensions)
  : imp_(new OccupancyMapDetail)
{
  imp_->resolution = resolution;
  imp_->region_voxel_dimensions.x =
    (region_voxel_dimensions.x > 0) ? region_voxel_dimensions.x : OHM_DEFAULT_CHUNK_DIM_X;
  imp_->region_voxel_dimensions.y =
    (region_voxel_dimensions.y > 0) ? region_voxel_dimensions.y : OHM_DEFAULT_CHUNK_DIM_Y;
  imp_->region_voxel_dimensions.z =
    (region_voxel_dimensions.z > 0) ? region_voxel_dimensions.z : OHM_DEFAULT_CHUNK_DIM_Z;
  imp_->region_spatial_dimensions.x = imp_->region_voxel_dimensions.x * resolution;
  imp_->region_spatial_dimensions.y = imp_->region_voxel_dimensions.y * resolution;
  imp_->region_spatial_dimensions.z = imp_->region_voxel_dimensions.z * resolution;
  imp_->saturate_at_min_value = imp_->saturate_at_max_value = false;
  // Default thresholds taken from octomap as a guide.
  imp_->min_voxel_value = -2.0f;
  imp_->max_voxel_value = 3.511f;
  setHitProbability(0.7f);
  setMissProbability(0.4f);
  setOccupancyThresholdProbability(0.5f);
  imp_->setDefaultLayout();
  imp_->ray_filter = [](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) {
    return ohm::goodRayFilter(start, end, filter_flags, 0);
  };
}


OccupancyMap::~OccupancyMap()
{
  if (imp_)
  {
    clear();
    delete imp_;
  }
}


OccupancyMap::iterator OccupancyMap::begin()
{
  return iterator(imp_, firstIterationKey());
}


OccupancyMap::const_iterator OccupancyMap::begin() const
{
  return const_iterator(imp_, firstIterationKey());
}


OccupancyMap::iterator OccupancyMap::end()
{
  return iterator(imp_, Key::kNull);
}


OccupancyMap::const_iterator OccupancyMap::end() const
{
  return const_iterator(imp_, Key::kNull);
}


namespace
{
  template <typename NODE, typename DETAIL>
  NODE getMapNode(DETAIL *detail, const Key &key, const OccupancyMapDetail *map)
  {
    std::unique_lock<decltype(map->mutex)> guard(map->mutex);
    auto region_ref = detail->findRegion(key.regionKey());
    if (region_ref != detail->chunks.end())
    {
      return NODE(key, region_ref->second, map);
    }
    return NODE();
  }
}  // namespace


Voxel OccupancyMap::voxel(const Key &key, bool allow_create, MapCache *cache)
{
  MapChunk *chunk = (cache) ? cache->lookup(key) : nullptr;

  if (!chunk)
  {
    std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
    const auto region_ref = imp_->findRegion(key.regionKey());
    if (region_ref != imp_->chunks.end())
    {
      chunk = region_ref->second;
    }
    else if (allow_create)
    {
      // No such chunk. Create one.
      chunk = newChunk(key);
      imp_->chunks.insert(std::make_pair(chunk->region.hash, chunk));
      // No need to touch the map here. We haven't changed the semantics of the map until
      // we change the value of a voxel in the region.
    }
  }

  if (chunk)
  {
    if (cache)
    {
      cache->push(chunk);
    }
    return Voxel(key, chunk, imp_);
  }
  return Voxel();
}


VoxelConst OccupancyMap::voxel(const Key &key, MapCache *cache) const
{
  MapChunk *chunk = (cache) ? cache->lookup(key) : nullptr;

  if (!chunk)
  {
    std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
    const auto region_ref = imp_->findRegion(key.regionKey());
    if (region_ref != imp_->chunks.end())
    {
      chunk = region_ref->second;
      if (cache)
      {
        cache->push(chunk);
      }
    }
  }

  if (chunk)
  {
    return VoxelConst(key, chunk, imp_);
  }

  return VoxelConst();
}


int OccupancyMap::occupancyType(const VoxelConst &voxel) const
{
  if (!voxel.isNull())
  {
    const float value = voxel.value();
    if (value < voxel::invalidMarkerValue())
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
  size_t byte_count = 0;
  byte_count += sizeof(this);
  if (imp_)
  {
    std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
    const glm::ivec3 dim = imp_->region_voxel_dimensions;
    const size_t voxels_per_chunk = size_t(dim.x) * size_t(dim.y) * size_t(dim.z);

    byte_count += sizeof(OccupancyMapDetail);
    byte_count += imp_->chunks.size() * sizeof(MapChunk);
    byte_count += imp_->chunks.size() * 2 * sizeof(float) * voxels_per_chunk;
    // TODO: consider coarseClearance array.

    // Approximate hash map usage.
    const size_t map_bucked_count = imp_->chunks.bucket_count();
    const double map_load = imp_->chunks.max_load_factor();
    if (map_load > 1.0)
    {
      byte_count += size_t(map_bucked_count * map_load * sizeof(MapChunk *));
    }
    else
    {
      byte_count += map_bucked_count * sizeof(MapChunk *);
    }
  }

  return byte_count;
}


double OccupancyMap::resolution() const
{
  return imp_->resolution;
}


uint64_t OccupancyMap::stamp() const
{
  return imp_->stamp;
}


void OccupancyMap::touch()
{
  ++imp_->stamp;
}


glm::dvec3 OccupancyMap::regionSpatialResolution() const
{
  return imp_->region_spatial_dimensions;
}


glm::u8vec3 OccupancyMap::regionVoxelDimensions() const
{
  return imp_->region_voxel_dimensions;
}


size_t OccupancyMap::regionVoxelVolume() const
{
  size_t v = imp_->region_voxel_dimensions.x;
  v *= imp_->region_voxel_dimensions.y;
  v *= imp_->region_voxel_dimensions.z;
  return v;
}


glm::dvec3 OccupancyMap::regionSpatialMin(const glm::i16vec3 &region_key) const
{
  const glm::dvec3 spatial_min = regionSpatialCentre(region_key) - 0.5 * imp_->region_spatial_dimensions;
  return spatial_min;
}


glm::dvec3 OccupancyMap::regionSpatialMax(const glm::i16vec3 &region_key) const
{
  const glm::dvec3 spatial_max = regionSpatialCentre(region_key) + 0.5 * imp_->region_spatial_dimensions;
  return spatial_max;
}


glm::dvec3 OccupancyMap::regionSpatialCentre(const glm::i16vec3 &region_key) const
{
  const glm::dvec3 centre(regionCentreCoord(region_key.x, imp_->region_spatial_dimensions.x),
                          regionCentreCoord(region_key.y, imp_->region_spatial_dimensions.y),
                          regionCentreCoord(region_key.z, imp_->region_spatial_dimensions.z));
  return centre;
}


void OccupancyMap::setOrigin(const glm::dvec3 &origin)
{
  imp_->origin = origin;
}


const glm::dvec3 &OccupancyMap::origin() const
{
  return imp_->origin;
}


void OccupancyMap::calculateExtents(glm::dvec3 &min_ext, glm::dvec3 &max_ext) const
{
  glm::dvec3 region_min, region_max;
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  if (imp_->chunks.empty())
  {
    // Empty map. Use the origin.
    min_ext = max_ext = imp_->origin;
    return;
  }

  min_ext = glm::dvec3(std::numeric_limits<double>::max());
  max_ext = glm::dvec3(-std::numeric_limits<double>::max());

  for (auto &&chunk : imp_->chunks)
  {
    const MapRegion region = chunk.second->region;
    region_min = region_max = region.centre;
    region_min -= 0.5 * regionSpatialResolution();
    region_max += 0.5 * regionSpatialResolution();

    min_ext.x = std::min(min_ext.x, region_min.x);
    min_ext.y = std::min(min_ext.y, region_min.y);
    min_ext.z = std::min(min_ext.z, region_min.z);

    max_ext.x = std::max(max_ext.x, region_max.x);
    max_ext.y = std::max(max_ext.y, region_max.y);
    max_ext.z = std::max(max_ext.z, region_max.z);
  }
}


MapInfo &OccupancyMap::mapInfo()
{
  return imp_->info;
}


const MapInfo &OccupancyMap::mapInfo() const
{
  return imp_->info;
}


const MapLayout &OccupancyMap::layout() const
{
  return imp_->layout;
}


MapLayout &OccupancyMap::layout()
{
  return imp_->layout;
}


void OccupancyMap::enableSubVoxelPositioning()
{
  imp_->enableSubVoxelPositioning();
}


size_t OccupancyMap::regionCount() const
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  return imp_->chunks.size();
}


unsigned OccupancyMap::expireRegions(double timestamp)
{
  const auto should_remove_chunk = [timestamp](const MapChunk &chunk) {
    if (chunk.touched_time < timestamp)
    {
      return true;
    }

    return false;
  };

  return cullRegions(should_remove_chunk);
}


unsigned OccupancyMap::removeDistanceRegions(const glm::dvec3 &relative_to, float distance)
{
  const float dist_sqr = distance * distance;
  const auto should_remove_chunk = [relative_to, dist_sqr](const MapChunk &chunk) {
    glm::dvec3 separation;
    separation = chunk.region.centre - relative_to;
    const double region_distance_sqr = glm::dot(separation, separation);
    if (region_distance_sqr >= dist_sqr)
    {
      return true;
    }

    return false;
  };

  return cullRegions(should_remove_chunk);
}


unsigned OccupancyMap::cullRegionsOutside(const glm::dvec3 &min_extents, const glm::dvec3 &max_extents)
{
  const glm::dvec3 region_extents = imp_->region_spatial_dimensions;
  const Aabb cullBox(min_extents, max_extents);
  const auto should_remove_chunk = [cullBox, region_extents](const MapChunk &chunk) {
    return !cullBox.overlaps(
      Aabb(chunk.region.centre - 0.5 * region_extents, chunk.region.centre + 0.5 * region_extents));
  };

  return cullRegions(should_remove_chunk);
}


void OccupancyMap::touchRegionByKey(const glm::i16vec3 &region_key, double timestamp, bool allow_create)
{
  MapChunk *chunk = region(region_key, allow_create);
  if (chunk)
  {
    chunk->touched_time = timestamp;
  }
}


glm::dvec3 OccupancyMap::regionCentreGlobal(const glm::i16vec3 &region_key) const
{
  return imp_->origin + regionCentreLocal(region_key);
}


glm::dvec3 OccupancyMap::regionCentreLocal(const glm::i16vec3 &region_key) const
{
  glm::dvec3 centre;
  centre.x = region_key.x * imp_->region_spatial_dimensions.x;
  centre.y = region_key.y * imp_->region_spatial_dimensions.y;
  centre.z = region_key.z * imp_->region_spatial_dimensions.z;
  return centre;
}


glm::i16vec3 OccupancyMap::regionKey(const glm::dvec3 &point) const
{
  MapRegion region(point, imp_->origin, imp_->region_spatial_dimensions);
  return region.coord;
}


float OccupancyMap::hitValue() const
{
  return imp_->hit_value;
}


float OccupancyMap::hitProbability() const
{
  return imp_->hit_probability;
}


void OccupancyMap::setHitProbability(float probability)
{
  imp_->hit_probability = probability;
  ;
  imp_->hit_value = probabilityToValue(probability);
}


float OccupancyMap::missValue() const
{
  return imp_->miss_value;
}


float OccupancyMap::missProbability() const
{
  return imp_->miss_probability;
}


void OccupancyMap::setMissProbability(float probability)
{
  imp_->miss_probability = probability;
  ;
  imp_->miss_value = probabilityToValue(probability);
}


float OccupancyMap::occupancyThresholdValue() const
{
  return imp_->occupancy_threshold_value;
}


float OccupancyMap::occupancyThresholdProbability() const
{
  return imp_->occupancy_threshold_probability;
}


void OccupancyMap::setOccupancyThresholdProbability(float probability)
{
  imp_->occupancy_threshold_probability = probability;
  ;
  imp_->occupancy_threshold_value = probabilityToValue(probability);
}


Voxel OccupancyMap::addNode(const Key &key, float value)
{
  MapChunk *chunk;
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const auto region_search = imp_->findRegion(key.regionKey());
  if (region_search == imp_->chunks.end())
  {
    // Allocate a new chunk.
    chunk = newChunk(key);
    imp_->chunks.insert(std::make_pair(chunk->region.hash, chunk));
    // No need to touch here. We haven't changed the semantics of the map at this point.
    // Also, the setValue() call below will touch the map.
  }
  else
  {
    chunk = region_search->second;
  }

  Voxel voxel(key, chunk, imp_);
  // Set value through this function to ensure first valid index is maintained.
  voxel.setValue(value);
  return voxel;
}


float OccupancyMap::minNodeValue() const
{
  return imp_->min_voxel_value;
}


void OccupancyMap::setMinNodeValue(float value)
{
  imp_->min_voxel_value = value;
}


bool OccupancyMap::saturateAtMinValue() const
{
  return imp_->saturate_at_min_value;
}


void OccupancyMap::setSaturateAtMinValue(bool saturate)
{
  imp_->saturate_at_min_value = saturate;
}


float OccupancyMap::maxNodeValue() const
{
  return imp_->max_voxel_value;
}


void OccupancyMap::setMaxNodeValue(float value)
{
  imp_->max_voxel_value = value;
}


bool OccupancyMap::saturateAtMaxValue() const
{
  return imp_->saturate_at_max_value;
}


void OccupancyMap::setSaturateAtMaxValue(bool saturate)
{
  imp_->saturate_at_max_value = saturate;
}


glm::dvec3 OccupancyMap::voxelCentreLocal(const Key &key) const
{
  return voxel::centreLocal(key, *imp_);
}


glm::dvec3 OccupancyMap::voxelCentreGlobal(const Key &key) const
{
  return voxel::centreGlobal(key, *imp_);
}


Key OccupancyMap::voxelKey(const glm::dvec3 &point) const
{
  Key key;
  MapRegion region(point, imp_->origin, imp_->region_spatial_dimensions);
  // VALIDATION code ensures the region we calculate to contain the point does.
  // Floating point error was causing issues where it nearly, but not quite would.
#ifdef OHM_VALIDATION
  const bool voxelKeyOk =
#endif  // OHM_VALIDATION
    region.voxelKey(key, point, imp_->origin, imp_->region_spatial_dimensions, imp_->region_voxel_dimensions,
                    imp_->resolution);
#ifdef OHM_VALIDATION
  if (!voxelKeyOk)
  {
    fprintf(stderr, "E: Validation failure: Point (%lg %lg %lg) fell into a region which generated an invalid key.\n",
            point.x, point.y, point.z);
    fprintf(stderr, "  Point: %.20lf %.20lf %.20lf\n", point.x, point.y, point.z);
    fprintf(stderr, "  Map origin: %lf %lf %lf\n", imp_->origin.x, imp_->origin.y, imp_->origin.z);
    fprintf(stderr, "  Map resolution: %lf\n", imp_->resolution);
    fprintf(stderr, "  Region sizing: %lf %lf %lf\n", imp_->region_spatial_dimensions.x,
            imp_->region_spatial_dimensions.y, imp_->region_spatial_dimensions.z);
    fprintf(stderr, "  Region voxels: %d %d %d\n", imp_->region_voxel_dimensions.x, imp_->region_voxel_dimensions.y,
            imp_->region_voxel_dimensions.z);
    fprintf(stderr, "  Region coord: %d %d %d\n", region.coord.x, region.coord.y, region.coord.z);
    fprintf(stderr, "  Region centre: %lf %lf %lf\n", region.centre.x, region.centre.y, region.centre.z);
  }
#endif  // OHM_VALIDATION
  return key;
}


Key OccupancyMap::voxelKey(const glm::vec3 &point) const
{
  Key key;
  MapRegion region(point, imp_->origin, imp_->region_spatial_dimensions);
  region.voxelKey(key, point, imp_->origin, imp_->region_spatial_dimensions, imp_->region_voxel_dimensions,
                  imp_->resolution);
  return key;
}


Key OccupancyMap::voxelKeyLocal(const glm::vec3 &local_point) const
{
  Key key;
  const glm::dvec3 zero_origin(0, 0, 0);
  MapRegion region(local_point, zero_origin, imp_->region_spatial_dimensions);
  region.voxelKey(key, local_point, zero_origin, imp_->region_spatial_dimensions, imp_->region_voxel_dimensions,
                  imp_->resolution);
  return key;
}


void OccupancyMap::moveKeyAlongAxis(Key &key, int axis, int step) const
{
  imp_->moveKeyAlongAxis(key, axis, step);
}


void OccupancyMap::stepKey(Key &key, int axis, int dir) const
{
  int local_key = key.localKey()[axis] + dir;
  int region_key = key.regionKey()[axis];

  if (local_key < 0)
  {
    --region_key;
    local_key = imp_->region_voxel_dimensions[axis] - 1;
  }
  else if (local_key >= imp_->region_voxel_dimensions[axis])
  {
    ++region_key;
    local_key = 0;
  }

  key.setLocalAxis(axis, uint8_t(local_key));
  key.setRegionAxis(axis, uint16_t(region_key));
}


void OccupancyMap::moveKey(Key &key, int x, int y, int z) const
{
  moveKeyAlongAxis(key, 0, x);
  moveKeyAlongAxis(key, 1, y);
  moveKeyAlongAxis(key, 2, z);
}


size_t OccupancyMap::calculateSegmentKeys(KeyList &keys, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                                          bool include_end_point) const
{
  struct KeyAdaptor
  {
    const OccupancyMap &map;

    inline KeyAdaptor(const OccupancyMap &map)
      : map(map)
    {}

    inline Key voxelKey(const glm::dvec3 &pt) const { return map.voxelKey(pt); }
    inline bool isNull(const Key &key) const { return key.isNull(); }
    inline glm::dvec3 voxelCentre(const Key &key) const { return map.voxelCentreLocal(key); }
    inline void stepKey(Key &key, int axis, int dir) const { map.stepKey(key, axis, dir); }
    inline double voxelResolution(int /*axis*/) const { return map.resolution(); }
  };
  const glm::dvec3 start_point_local = glm::dvec3(start_point - origin());
  const glm::dvec3 end_point_local = glm::dvec3(end_point - origin());


  keys.clear();
  return ohmutil::walkSegmentKeys<Key>([&keys](const Key &key) { keys.add(key); }, start_point_local, end_point_local,
                                       include_end_point, KeyAdaptor(*this));
}


void OccupancyMap::setRayFilter(const RayFilterFunction &ray_filter)
{
  imp_->ray_filter = ray_filter;
}


const RayFilterFunction &OccupancyMap::rayFilter() const
{
  return imp_->ray_filter;
}


void OccupancyMap::clearRayFilter()
{
  imp_->ray_filter = RayFilterFunction();
}


void OccupancyMap::integrateRays(const glm::dvec3 *rays, size_t element_count, bool end_points_as_occupied)
{
  KeyList keys;
  MapCache cache;
  bool clipped_sample_voxel;

  const bool use_filter = bool(imp_->ray_filter);

  glm::dvec3 start, end;
  unsigned filter_flags;
  for (size_t i = 0; i < element_count; i += 2)
  {
    filter_flags = 0;
    start = rays[i];
    end = rays[i + 1];


    if (use_filter)
    {
      if (!imp_->ray_filter(&start, &end, &filter_flags))
      {
        // Bad ray.
        continue;
      }
    }

    clipped_sample_voxel = (filter_flags & kRffClippedEnd);

    // Calculate line key for the last voxel if the end point has been clipped
    calculateSegmentKeys(keys, start, end, clipped_sample_voxel);

    for (auto &&key : keys)
    {
      integrateMiss(key, &cache);
    }

    if (!clipped_sample_voxel)
    {
      if (end_points_as_occupied)
      {
        integrateHit(voxelKey(rays[i + 1]), &cache);
      }
      else
      {
        integrateMiss(voxelKey(rays[i + 1]), &cache);
      }
    }
  }
}


OccupancyMap *OccupancyMap::clone() const
{
  return clone(-glm::dvec3(std::numeric_limits<double>::infinity()),
               glm::dvec3(std::numeric_limits<double>::infinity()));
}


OccupancyMap *OccupancyMap::clone(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext) const
{
  OccupancyMap *new_map = new OccupancyMap(imp_->resolution, imp_->region_voxel_dimensions);

  if (imp_->ray_filter)
  {
    new_map->setRayFilter(imp_->ray_filter);
  }

  // Copy general details.
  new_map->detail()->copyFrom(*imp_);

  glm::dvec3 region_min, region_max;
  const glm::dvec3 region_half_ext = 0.5 * imp_->region_spatial_dimensions;
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  for (const auto chunk_iter : imp_->chunks)
  {
    const MapChunk *src_chunk = chunk_iter.second;
    region_min = region_max = src_chunk->region.centre;
    region_min -= region_half_ext;
    region_max += region_half_ext;

    if (!glm::any(glm::lessThan(region_max, min_ext)) && !glm::any(glm::greaterThan(region_min, max_ext)))
    {
      MapChunk *dst_chunk = new_map->region(src_chunk->region.coord, true);
      dst_chunk->first_valid_index = src_chunk->first_valid_index;
      dst_chunk->touched_time = src_chunk->touched_time;
      dst_chunk->dirty_stamp = src_chunk->dirty_stamp;
      dst_chunk->flags = src_chunk->flags;

      for (unsigned i = 0; i < imp_->layout.layerCount(); ++i)
      {
        const MapLayer &layer = src_chunk->layout->layer(i);
        dst_chunk->touched_stamps[i] = static_cast<uint64_t>(src_chunk->touched_stamps[i]);
        if (src_chunk->voxel_maps[i])
        {
          memcpy(dst_chunk->voxel_maps[i], src_chunk->voxel_maps[i],
                 layer.layerByteSize(imp_->region_voxel_dimensions));
        }
      }
    }
  }

  return new_map;
}


void OccupancyMap::enumerateRegions(std::vector<const MapChunk *> &chunks) const
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  for (auto &&chunk_iter : imp_->chunks)
  {
    chunks.push_back(chunk_iter.second);
  }
}


MapChunk *OccupancyMap::region(const glm::i16vec3 &region_key, bool allow_create)
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const auto region_search = imp_->findRegion(region_key);
  if (region_search != imp_->chunks.end())
  {
    MapChunk *chunk = region_search->second;
#ifdef OHM_VALIDATION
    chunk->validateFirstValid(imp_->region_voxel_dimensions);
#endif  // OHM_VALIDATION
    return chunk;
  }

  if (allow_create)
  {
    // No such chunk. Create one.
    MapChunk *chunk = newChunk(Key(region_key, 0, 0, 0));
    imp_->chunks.insert(std::make_pair(chunk->region.hash, chunk));
    // No need to touch the map here. We haven't changed the semantics of the map.
    // That happens when the value of a voxel in the region changes.
    return chunk;
  }

  return nullptr;
}


const MapChunk *OccupancyMap::region(const glm::i16vec3 &region_key) const
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const auto region_search = imp_->findRegion(region_key);
  if (region_search != imp_->chunks.end())
  {
    const MapChunk *chunk = region_search->second;
    return chunk;
  }

  return nullptr;
}


unsigned OccupancyMap::collectDirtyRegions(uint64_t from_stamp,
                                           std::vector<std::pair<uint64_t, glm::i16vec3>> &regions) const
{
  // Brute for for now.
  unsigned added_count = 0;
  bool added;
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  for (auto &&chunk_ref : imp_->chunks)
  {
    if (chunk_ref.second->dirty_stamp > from_stamp)
    {
      added = false;

      // Insertion sorted on the chunk's dirty stamp. Least recently touched (oldtest) first.
      // TODO: test efficiency of the sorted insertion on a vector.
      // Scope should be small so I expect little impact.
      auto item = std::make_pair(chunk_ref.second->dirty_stamp, chunk_ref.second->region.coord);
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

      ++added_count;
    }
  }

  return added_count;
}


void OccupancyMap::calculateDirtyExtents(uint64_t *from_stamp, glm::i16vec3 *min_ext, glm::i16vec3 *max_ext) const
{
  *min_ext = glm::i16vec3(std::numeric_limits<decltype(min_ext->x)>::max());
  *max_ext = glm::i16vec3(std::numeric_limits<decltype(min_ext->x)>::min());

  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const uint64_t at_stamp = imp_->stamp;
  for (auto &&chunk_ref : imp_->chunks)
  {
    if (chunk_ref.second->dirty_stamp > *from_stamp)
    {
      min_ext->x = std::min(chunk_ref.second->region.coord.x, min_ext->x);
      min_ext->y = std::min(chunk_ref.second->region.coord.y, min_ext->y);
      min_ext->z = std::min(chunk_ref.second->region.coord.z, min_ext->z);

      max_ext->x = std::max(chunk_ref.second->region.coord.x, max_ext->x);
      max_ext->y = std::max(chunk_ref.second->region.coord.y, max_ext->y);
      max_ext->z = std::max(chunk_ref.second->region.coord.z, max_ext->z);
    }
  }
  guard.unlock();

  if (min_ext->x > max_ext->x)
  {
    *min_ext = glm::i16vec3(1);
    *max_ext = glm::i16vec3(0);
  }
  *from_stamp = at_stamp;
}

void OccupancyMap::calculateDirtyClearanceExtents(glm::i16vec3 *min_ext, glm::i16vec3 *max_ext,
                                                  unsigned region_padding) const
{
  *min_ext = glm::i16vec3(std::numeric_limits<decltype(min_ext->x)>::max());
  *max_ext = glm::i16vec3(std::numeric_limits<decltype(min_ext->x)>::min());

  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const int occupancy_layer = imp_->layout.occupancyLayer();
  const int clearance_layer = imp_->layout.clearanceLayer();

  if (occupancy_layer >= 0 && clearance_layer >= 0)
  {
    for (auto &&chunk_ref : imp_->chunks)
    {
      if (chunk_ref.second->touched_stamps[clearance_layer] < chunk_ref.second->touched_stamps[occupancy_layer])
      {
        min_ext->x = std::min<int>(chunk_ref.second->region.coord.x - region_padding, min_ext->x);
        min_ext->y = std::min<int>(chunk_ref.second->region.coord.y - region_padding, min_ext->y);
        min_ext->z = std::min<int>(chunk_ref.second->region.coord.z - region_padding, min_ext->z);

        max_ext->x = std::max<int>(chunk_ref.second->region.coord.x + region_padding, max_ext->x);
        max_ext->y = std::max<int>(chunk_ref.second->region.coord.y + region_padding, max_ext->y);
        max_ext->z = std::max<int>(chunk_ref.second->region.coord.z + region_padding, max_ext->z);
      }
    }
  }
  guard.unlock();

  if (min_ext->x > max_ext->x)
  {
    *min_ext = glm::i16vec3(1);
    *max_ext = glm::i16vec3(0);
  }
}


void OccupancyMap::clear()
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  for (auto &&chunk_ref : imp_->chunks)
  {
    releaseChunk(chunk_ref.second);
  }
  imp_->chunks.clear();

  // Clear the GPU cache (if present).
  if (imp_->gpu_cache)
  {
    imp_->gpu_cache->clear();
  }
}


Key OccupancyMap::firstIterationKey() const
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const auto first_chunk_iter = imp_->chunks.begin();
  if (first_chunk_iter != imp_->chunks.end())
  {
    MapChunk *chunk = first_chunk_iter->second;
    return firstKeyForChunk(*imp_, *chunk);
  }

  return Key::kNull;
}


MapChunk *OccupancyMap::newChunk(const Key &for_key)
{
  MapChunk *chunk = new MapChunk(MapRegion(voxelCentreGlobal(for_key), imp_->origin, imp_->region_spatial_dimensions),
                                 imp_->layout, imp_->region_voxel_dimensions);
  return chunk;
}


void OccupancyMap::releaseChunk(const MapChunk *chunk)
{
  delete chunk;
}


unsigned OccupancyMap::cullRegions(const RegionCullFunc &cull_func)
{
  unsigned removed_count = 0;
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  auto region_iter = imp_->chunks.begin();
  const MapChunk *chunk = nullptr;
  while (region_iter != imp_->chunks.end())
  {
    chunk = region_iter->second;

    if (cull_func(*chunk))
    {
      // Remove from the GPU cache.
      if (imp_->gpu_cache)
      {
        imp_->gpu_cache->remove(chunk->region.coord);
      }

      // Culled region. Remove from the map.
      region_iter = imp_->chunks.erase(region_iter);
      releaseChunk(chunk);
      ++removed_count;
    }
    else
    {
      // Next.
      ++region_iter;
    }
  }

  return removed_count;
}
