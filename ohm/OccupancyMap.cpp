// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OccupancyMap.h"

#include "Aabb.h"
#include "DefaultLayer.h"
#include "KeyList.h"
#include "KeyRange.h"
#include "MapChunk.h"
#include "MapCoord.h"
#include "MapLayer.h"
#include "MapProbability.h"
#include "MapRegionCache.h"
#include "RayMapperOccupancy.h"
#include "VoxelBlockCompressionQueue.h"
#include "VoxelBuffer.h"
#include "VoxelOccupancy.h"

#include "OccupancyUtil.h"

#include "private/OccupancyMapDetail.h"

#include <ohmutil/LineWalk.h>

#include <algorithm>
#include <cassert>
#ifdef OHM_VALIDATION
#include <cstdio>
#endif  // OHM_VALIDATION
#include <cstring>
#include <functional>
#include <limits>
#include <utility>

namespace ohm
{
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
  const glm::u8vec3 first_valid_key = chunk.firstValidKey(map.region_voxel_dimensions);
  return Key(chunk.region.coord, std::min(first_valid_key.x, uint8_t(map.region_voxel_dimensions.x - 1)),
             std::min(first_valid_key.y, uint8_t(map.region_voxel_dimensions.y - 1)),
             std::min(first_valid_key.z, uint8_t(map.region_voxel_dimensions.z - 1)));
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

ChunkMap::iterator &initChunkIter(uint8_t *mem)  // NOLINT(readability-non-const-parameter)
{
  // Placement new.
  return *(new (mem) ChunkMap::iterator());
}

// NOLINTNEXTLINE(readability-non-const-parameter)
ChunkMap::iterator &chunkIter(uint8_t *mem)
{
  return *reinterpret_cast<ChunkMap::iterator *>(mem);
}

const ChunkMap::iterator &chunkIter(const uint8_t *mem)
{
  return *reinterpret_cast<const ChunkMap::iterator *>(mem);
}

void releaseChunkIter(uint8_t *mem)
{
  using Iterator = ChunkMap::iterator;
  chunkIter(mem).~Iterator();
}
}  // namespace

OccupancyMap::base_iterator::base_iterator()  // NOLINT
  : key_(Key::kNull)
{
  static_assert(sizeof(ChunkMap::iterator) <= sizeof(OccupancyMap::base_iterator::chunk_mem_), "Insufficient space for "
                                                                                               "chunk iterator.");
  initChunkIter(chunk_mem_.data());
}

OccupancyMap::base_iterator::base_iterator(OccupancyMap *map, const Key &key)  // NOLINT
  : map_(map)
  , key_(key)
{
  ChunkMap::iterator &chunk_iter = initChunkIter(chunk_mem_.data());
  if (!key.isNull())
  {
    std::unique_lock<decltype(map->detail()->mutex)> guard(map->detail()->mutex);
    chunk_iter = map->detail()->chunks.find(key.regionKey());
  }
}

OccupancyMap::base_iterator::base_iterator(const base_iterator &other)  // NOLINT
  : map_(other.map_)
  , key_(other.key_)
{
  static_assert(sizeof(ChunkMap::iterator) <= sizeof(OccupancyMap::base_iterator::chunk_mem_),  //
                "Insufficient space for chunk iterator.");
  initChunkIter(chunk_mem_.data()) = chunkIter(other.chunk_mem_.data());
}

OccupancyMap::base_iterator::~base_iterator()
{
  // Explicit destructor invocation.
  releaseChunkIter(chunk_mem_.data());
}

OccupancyMap::base_iterator &OccupancyMap::base_iterator::operator=(const base_iterator &other)
{
  if (this != &other)
  {
    map_ = other.map_;
    key_ = other.key_;
    chunkIter(chunk_mem_.data()) = chunkIter(other.chunk_mem_.data());
  }
  return *this;
}

bool OccupancyMap::base_iterator::operator==(const base_iterator &other) const
{
  // Chunks only have to match when not the end/invalid iterator.
  return map_ == other.map_ && key_ == other.key_ &&
         (key_.isNull() || chunkIter(chunk_mem_.data()) == chunkIter(other.chunk_mem_.data()));
}

bool OccupancyMap::base_iterator::operator!=(const base_iterator &other) const
{
  return !(*this == other);
}

bool OccupancyMap::base_iterator::base_iterator::isValid() const
{
  return map_ && !key_.isNull();
}

void OccupancyMap::base_iterator::walkNext()
{
  if (!key_.isNull())
  {
    if (!nextLocalKey(key_, map_->detail()->region_voxel_dimensions))
    {
      // Need to move to the next chunk.
      ChunkMap::iterator &chunk = chunkIter(chunk_mem_.data());
      if (!nextChunk(*map_->detail(), chunk, key_))
      {
        // Invalidate.
        key_ = Key::kNull;
        releaseChunkIter(chunk_mem_.data());
        initChunkIter(chunk_mem_.data());
      }
    }
  }
}

const MapChunk *OccupancyMap::base_iterator::chunk() const
{
  return chunkIter(chunk_mem_.data())->second;
}

MapChunk *OccupancyMap::iterator::chunk() const
{
  return chunkIter(chunk_mem_.data())->second;
}

OccupancyMap::OccupancyMap(double resolution, const glm::u8vec3 &region_voxel_dimensions, MapFlag flags)
  : imp_(new OccupancyMapDetail)
{
  // Start the voxel map compression queue thread.
  VoxelBlockCompressionQueue::instance().retain();
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
  // Default min/max thresholds taken from octomap as a guide.
  imp_->min_voxel_value = -2.0f;   // NOLINT(readability-magic-numbers)
  imp_->max_voxel_value = 3.511f;  // NOLINT(readability-magic-numbers)
  setHitProbability(0.9f);         // NOLINT(readability-magic-numbers)
  setMissProbability(0.45f);       // NOLINT(readability-magic-numbers)
  setOccupancyThresholdProbability(0.5f);

  imp_->ray_filter = [](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) {
    const double large_long_ray_limit = 1e10;
    return ohm::goodRayFilter(start, end, filter_flags, large_long_ray_limit);
  };

  imp_->flags = flags;
  imp_->setDefaultLayout(flags);
}

OccupancyMap::OccupancyMap(double resolution, const glm::u8vec3 &region_voxel_dimensions, MapFlag flags,
                           const MapLayout &seed_layout)
  : OccupancyMap(resolution, region_voxel_dimensions, flags)
{
  imp_->layout = seed_layout;
  if ((flags & MapFlag::kVoxelMean) != MapFlag::kNone)
  {
    addVoxelMeanLayer();
  }
  if ((flags & MapFlag::kTraversal) != MapFlag::kNone)
  {
    addTraversalLayer();
  }
}

OccupancyMap::OccupancyMap(double resolution, MapFlag flags, const MapLayout &seed_layout)
  : OccupancyMap(resolution, glm::u8vec3(0, 0, 0), flags, seed_layout)
{}

OccupancyMap::OccupancyMap(double resolution, MapFlag flags)
  : OccupancyMap(resolution, glm::u8vec3(0, 0, 0), flags)
{}

OccupancyMap::~OccupancyMap()
{
  if (imp_)
  {
    clear();
    delete imp_;
  }
  // Release the voxel map compression queue thread.
  VoxelBlockCompressionQueue::instance().release();
}

OccupancyMap::iterator OccupancyMap::begin()
{
  return iterator(this, firstIterationKey());
}

OccupancyMap::const_iterator OccupancyMap::begin() const
{
  // TODO(KS): remove const cast by templating the base_iterator
  return const_iterator(const_cast<OccupancyMap *>(this), firstIterationKey());
}

OccupancyMap::iterator OccupancyMap::end()
{
  return iterator(this, Key::kNull);
}

OccupancyMap::const_iterator OccupancyMap::end() const
{
  // TODO(KS): remove const cast by templating the base_iterator
  return const_iterator(const_cast<OccupancyMap *>(this), Key::kNull);
}

size_t OccupancyMap::calculateApproximateMemory() const
{
  size_t byte_count = 0;
  byte_count += sizeof(*this);
  if (imp_)
  {
    std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);

    const size_t chunk_count = (!imp_->chunks.empty()) ? imp_->chunks.size() : imp_->loaded_region_count;
    byte_count += sizeof(OccupancyMapDetail);
    byte_count += chunk_count * sizeof(MapChunk);

    for (unsigned i = 0; i < imp_->layout.layerCount(); ++i)
    {
      const MapLayer &layer = imp_->layout.layer(i);
      byte_count += chunk_count * layer.layerByteSize(imp_->region_voxel_dimensions);
    }

    // Approximate hash map usage.
    const size_t map_bucked_count = imp_->chunks.bucket_count();
    const double map_load = imp_->chunks.max_load_factor();
    if (map_load > 1.0)
    {
      // Lint(KS): Size of pointer is correct
      // NOLINTNEXTLINE(bugprone-sizeof-expression)
      byte_count += size_t(map_bucked_count * map_load * sizeof(MapChunk *));
    }
    else
    {
      // Lint(KS): Size of pointer is correct
      // NOLINTNEXTLINE(bugprone-sizeof-expression)
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

uint64_t OccupancyMap::touch()
{
  return ++imp_->stamp;
}

double OccupancyMap::firstRayTime() const
{
  return imp_->first_ray_time;
}

void OccupancyMap::setFirstRayTime(double time)
{
  imp_->first_ray_time = time;
}

double OccupancyMap::updateFirstRayTime(double time)
{
  imp_->first_ray_time = (imp_->first_ray_time < 0) ? time : imp_->first_ray_time;
  return imp_->first_ray_time;
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

bool OccupancyMap::calculateExtents(glm::dvec3 *min_ext, glm::dvec3 *max_ext, KeyRange *key_range) const
{
  glm::dvec3 region_min;
  glm::dvec3 region_max;
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  // Empty map if there are no chunks or the voxel dimensions are zero (latter just shouldn't happen).
  if (imp_->chunks.empty() || glm::any(glm::equal(imp_->region_voxel_dimensions, glm::u8vec3(0))))
  {
    // Empty map. Use the origin.
    if (min_ext)
    {
      *min_ext = imp_->origin;
    }
    if (max_ext)
    {
      *max_ext = imp_->origin;
    }

    if (key_range)
    {
      // Set an invalid range.
      *key_range = KeyRange();
    }
    return false;
  }

  glm::dvec3 min_spatial(std::numeric_limits<double>::max());
  glm::dvec3 max_spatial(-std::numeric_limits<double>::max());
  // We only need to track the min/max region keys. The min local voxel coordinate within a region is always (0, 0, 0),
  // while the maximum is always the region voxel dimensions - 1
  glm::i16vec3 min_region_key(std::numeric_limits<int16_t>::max());
  glm::i16vec3 max_region_key(std::numeric_limits<int16_t>::min());
  bool have_extents = false;

  for (auto &&chunk : imp_->chunks)
  {
    const MapRegion region = chunk.second->region;
    region_min = region_max = region.centre;
    region_min -= 0.5 * regionSpatialResolution();
    region_max += 0.5 * regionSpatialResolution();

    min_spatial.x = std::min(min_spatial.x, region_min.x);
    min_spatial.y = std::min(min_spatial.y, region_min.y);
    min_spatial.z = std::min(min_spatial.z, region_min.z);

    max_spatial.x = std::max(max_spatial.x, region_max.x);
    max_spatial.y = std::max(max_spatial.y, region_max.y);
    max_spatial.z = std::max(max_spatial.z, region_max.z);

    min_region_key.x = std::min(region.coord.x, min_region_key.x);
    min_region_key.y = std::min(region.coord.y, min_region_key.y);
    min_region_key.z = std::min(region.coord.z, min_region_key.z);
    max_region_key.x = std::max(region.coord.x, max_region_key.x);
    max_region_key.y = std::max(region.coord.y, max_region_key.y);
    max_region_key.z = std::max(region.coord.z, max_region_key.z);

    have_extents = true;
  }

  // Finalise the min/max voxel keys.
  const Key min_voxel(min_region_key, glm::u8vec3(0, 0, 0));
  const Key max_voxel(max_region_key, imp_->region_voxel_dimensions - glm::u8vec3(1, 1, 1));

  // Write output values.
  if (min_ext)
  {
    *min_ext = min_spatial;
  }

  if (max_ext)
  {
    *max_ext = max_spatial;
  }

  if (key_range)
  {
    key_range->setRegionDimensions(imp_->region_voxel_dimensions);
    key_range->setMinKey(min_voxel);
    key_range->setMaxKey(max_voxel);
  }

  return have_extents;
}

bool OccupancyMap::calculateExtents(glm::dvec3 *min_ext, glm::dvec3 *max_ext, Key *min_key, Key *max_key) const
{
  KeyRange range;
  bool valid = calculateExtents(min_ext, max_ext, &range);
  if (min_key)
  {
    *min_key = range.minKey();
  }
  if (max_key)
  {
    *max_key = range.maxKey();
  }
  return valid;
}

MapInfo &OccupancyMap::mapInfo()
{
  return imp_->info;
}

const MapInfo &OccupancyMap::mapInfo() const
{
  return imp_->info;
}

MapFlag OccupancyMap::flags() const
{
  return imp_->flags;
}

const MapLayout &OccupancyMap::layout() const
{
  return imp_->layout;
}

MapLayout &OccupancyMap::layout()
{
  return imp_->layout;
}

void OccupancyMap::addVoxelMeanLayer()
{
  if (imp_->layout.meanLayer() >= 0)
  {
    // Already present.
    return;
  }

  MapLayout layout = imp_->layout;
  addVoxelMean(layout);
  updateLayout(layout);
}


bool OccupancyMap::voxelMeanEnabled() const
{
  return imp_->layout.meanLayer() >= 0;
}


void OccupancyMap::addTraversalLayer()
{
  if (imp_->layout.traversalLayer() >= 0)
  {
    // Already present.
    return;
  }

  MapLayout layout = imp_->layout;
  addTraversal(layout);
  updateLayout(layout);
}


bool OccupancyMap::traversalEnabled() const
{
  return imp_->layout.traversalLayer() >= 0;
}


void OccupancyMap::addTouchTimeLayer()
{
  if (touchTimeEnabled())
  {
    // Already present.
    return;
  }

  MapLayout layout = imp_->layout;
  addTouchTime(layout);
  updateLayout(layout);
}


bool OccupancyMap::touchTimeEnabled() const
{
  return imp_->layout.layerIndex(default_layer::touchTimeLayerName()) >= 0;
}


void OccupancyMap::addIncidentNormalLayer()
{
  if (touchTimeEnabled())
  {
    // Already present.
    return;
  }

  MapLayout layout = imp_->layout;
  addIncidentNormal(layout);
  updateLayout(layout);
}


bool OccupancyMap::incidentNormalEnabled() const
{
  return imp_->layout.layerIndex(default_layer::incidentNormalLayerName()) >= 0;
}


void OccupancyMap::updateLayout(const MapLayout &new_layout, bool preserve_map)
{
  // First check if there is a difference between the @c MapLayout and the actual layout.
  // There's no work to do otherwise.

  const MapLayoutMatch match = imp_->layout.checkEquivalent(new_layout);
  if (match == MapLayoutMatch::kExact)
  {
    // Already matches the current layout. Nothing to do.
    return;
  }

  // Check for partial match. In this case we just have to update to the new layout values and don't need to adjust
  // the chunks.
  if (match == MapLayoutMatch::kEquivalent)
  {
    imp_->layout = new_layout;
    return;
  }

  // We have a memory change. A full update is required.

  // First we have to synchronise the GPU cache(s).
  if (imp_->gpu_cache)
  {
    imp_->gpu_cache->clear();
  }

  if (preserve_map)
  {
    /// Tracking of layer indices to preserve. First item is the layer index in the current layout, while the second is
    /// in the new layout.
    std::vector<std::pair<const MapLayer *, const MapLayer *>> layer_mapping;

    for (size_t i = 0; i < new_layout.layerCount(); ++i)
    {
      const MapLayer &new_layer = new_layout.layer(i);
      // Look for a matching layer in the current layout.
      if (const MapLayer *layer = imp_->layout.layer(new_layer.name()))
      {
        // Found a layer with matching name. Check for equivalent layout.
        if (layer->checkEquivalent(new_layer) != MapLayoutMatch::kDifferent)
        {
          // Layers are equivalent. Add a mapping.
          layer_mapping.emplace_back(std::make_pair(layer, &new_layer));
        }
      }
    }

    // Walk the chunks preserving which layers we can.
    for (auto &chunk : imp_->chunks)
    {
      chunk.second->updateLayout(&new_layout, layer_mapping);
    }
  }
  else
  {
    clear();
  }

  imp_->layout = new_layout;

  // Now reallocate any GPU cache which relies on the occupancy layer.
  if (imp_->gpu_cache)
  {
    imp_->gpu_cache->reinitialise();
  }
}


size_t OccupancyMap::regionCount() const
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  return imp_->chunks.size();
}

unsigned OccupancyMap::expireRegions(double timestamp)
{
  const auto should_remove_chunk = [timestamp](const MapChunk &chunk) { return chunk.touched_time < timestamp; };

  return cullRegions(should_remove_chunk);
}

unsigned OccupancyMap::removeDistanceRegions(const glm::dvec3 &relative_to, float distance)
{
  const float dist_sqr = distance * distance;
  const auto should_remove_chunk = [relative_to, dist_sqr](const MapChunk &chunk) {
    glm::dvec3 separation;
    separation = chunk.region.centre - relative_to;
    const double region_distance_sqr = glm::dot(separation, separation);
    return region_distance_sqr >= dist_sqr;
  };

  return cullRegions(should_remove_chunk);
}

unsigned OccupancyMap::cullRegionsOutside(const glm::dvec3 &min_extents, const glm::dvec3 &max_extents)
{
  const glm::dvec3 region_extents = imp_->region_spatial_dimensions;
  const Aabb cull_box(min_extents, max_extents);
  const auto should_remove_chunk = [cull_box, region_extents](const MapChunk &chunk) {
    return !cull_box.overlaps(
      Aabb(chunk.region.centre - 0.5 * region_extents, chunk.region.centre + 0.5 * region_extents));
  };

  return cullRegions(should_remove_chunk);
}

void OccupancyMap::touchRegionTimestampByKey(const glm::i16vec3 &region_key, double timestamp, bool allow_create)
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
  return valueToProbability(imp_->hit_value);
}

void OccupancyMap::setHitProbability(float probability)
{
  imp_->hit_value = probabilityToValue(probability);
}

void OccupancyMap::setHitValue(float value)
{
  imp_->hit_value = value;
  ;
}

float OccupancyMap::missValue() const
{
  return imp_->miss_value;
}

float OccupancyMap::missProbability() const
{
  return valueToProbability(imp_->miss_value);
}

void OccupancyMap::setMissProbability(float probability)
{
  imp_->miss_value = probabilityToValue(probability);
}

void OccupancyMap::setMissValue(float value)
{
  imp_->miss_value = value;
  ;
}

float OccupancyMap::occupancyThresholdValue() const
{
  return imp_->occupancy_threshold_value;
}

float OccupancyMap::occupancyThresholdProbability() const
{
  return valueToProbability(imp_->occupancy_threshold_value);
}

void OccupancyMap::setOccupancyThresholdProbability(float probability)
{
  imp_->occupancy_threshold_value = probabilityToValue(probability);
}

float OccupancyMap::minVoxelValue() const
{
  return imp_->min_voxel_value;
}

void OccupancyMap::setMinVoxelValue(float value)
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

float OccupancyMap::maxVoxelValue() const
{
  return imp_->max_voxel_value;
}

void OccupancyMap::setMaxVoxelValue(float value)
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
  return ohm::OccupancyMap::voxelCentre(key, imp_->resolution, imp_->region_spatial_dimensions);
}

glm::dvec3 OccupancyMap::voxelCentreGlobal(const Key &key) const
{
  return ohm::OccupancyMap::voxelCentre(key, imp_->resolution, imp_->region_spatial_dimensions, imp_->origin);
}

Key OccupancyMap::voxelKey(const glm::dvec3 &point) const
{
  Key key;
  // const glm::dvec3 map_local_key = point - imp_->origin;
  MapRegion region(point, imp_->origin, imp_->region_spatial_dimensions);
  // VALIDATION code ensures the region we calculate to contain the point does.
  // Floating point error was causing issues where it nearly, but not quite would.
  const bool voxelKeyOk = region.voxelKey(key, point, imp_->origin, imp_->region_spatial_dimensions,
                                          imp_->region_voxel_dimensions, imp_->resolution);
  (void)voxelKeyOk;
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
  stepKey(key, axis, dir, imp_->region_voxel_dimensions);
}

void OccupancyMap::moveKey(Key &key, int x, int y, int z) const
{
  moveKeyAlongAxis(key, 0, x);
  moveKeyAlongAxis(key, 1, y);
  moveKeyAlongAxis(key, 2, z);
}

glm::ivec3 OccupancyMap::rangeBetween(const Key &from, const Key &to) const
{
  return rangeBetween(from, to, imp_->region_voxel_dimensions);
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

void OccupancyMap::integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                                 const double *timestamps, unsigned ray_update_flags)
{
  // This function has been updated to leverage the new RayMapper interface and remove code duplication. It is
  // maintained for legacy reasons.
  // TODO(KS): remove this function and require the use of a RayMapper.
  RayMapperOccupancy(this).integrateRays(rays, element_count, intensities, timestamps, ray_update_flags);
}

OccupancyMap *OccupancyMap::clone() const
{
  return clone(-glm::dvec3(std::numeric_limits<double>::infinity()),
               glm::dvec3(std::numeric_limits<double>::infinity()));
}

OccupancyMap *OccupancyMap::clone(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext) const
{
  auto *new_map = new OccupancyMap(imp_->resolution, imp_->region_voxel_dimensions);

  if (imp_->ray_filter)
  {
    new_map->setRayFilter(imp_->ray_filter);
  }

  // Copy general details.
  new_map->detail()->copyFrom(*imp_);

  glm::dvec3 region_min;
  glm::dvec3 region_max;
  const glm::dvec3 region_half_ext = 0.5 * imp_->region_spatial_dimensions;
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  for (const auto &chunk_iter : imp_->chunks)
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
      dst_chunk->dirty_stamp = src_chunk->dirty_stamp.load();
      dst_chunk->flags = src_chunk->flags;

      for (unsigned i = 0; i < imp_->layout.layerCount(); ++i)
      {
        dst_chunk->touched_stamps[i] = static_cast<uint64_t>(src_chunk->touched_stamps[i]);
        if (src_chunk->voxel_blocks[i])
        {
          VoxelBuffer<const VoxelBlock> src_buffer(src_chunk->voxel_blocks[i]);
          VoxelBuffer<VoxelBlock> dst_buffer(dst_chunk->voxel_blocks[i]);
          memcpy(dst_buffer.voxelMemory(), src_buffer.voxelMemory(), dst_buffer.voxelMemorySize());
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
  const auto region_search = imp_->chunks.find(region_key);
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
    imp_->chunks.insert(std::make_pair(chunk->region.coord, chunk));
    // No need to touch the map here. We haven't changed the semantics of the map.
    // That happens when the value of a voxel in the region changes.
    return chunk;
  }

  return nullptr;
}

const MapChunk *OccupancyMap::region(const glm::i16vec3 &region_key) const
{
  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const auto region_search = imp_->chunks.find(region_key);
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
      // TODO(KS): test efficiency of the sorted insertion on a vector.
      // Scope should be small so I expect little impact.
      auto item = std::make_pair(chunk_ref.second->dirty_stamp.load(), chunk_ref.second->region.coord);
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

uint64_t OccupancyMap::calculateDirtyExtents(uint64_t from_stamp, glm::i16vec3 *min_ext, glm::i16vec3 *max_ext) const
{
  *min_ext = glm::i16vec3(std::numeric_limits<decltype(min_ext->x)>::max());
  *max_ext = glm::i16vec3(std::numeric_limits<decltype(min_ext->x)>::min());

  std::unique_lock<decltype(imp_->mutex)> guard(imp_->mutex);
  const uint64_t at_stamp = imp_->stamp;
  for (auto &&chunk_ref : imp_->chunks)
  {
    if (chunk_ref.second->dirty_stamp > from_stamp)
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
  return at_stamp;
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
  // Clear the GPU cache (if present).
  // Must occur before deleting the chunks as it will be referencing some.
  if (imp_->gpu_cache)
  {
    imp_->gpu_cache->clear();
  }

  for (auto &&chunk_ref : imp_->chunks)
  {
    releaseChunk(chunk_ref.second);
  }

  imp_->chunks.clear();
  imp_->loaded_region_count = 0;
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
  auto *chunk =
    new MapChunk(MapRegion(voxelCentreGlobal(for_key), imp_->origin, imp_->region_spatial_dimensions), *imp_);
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
}  // namespace ohm
