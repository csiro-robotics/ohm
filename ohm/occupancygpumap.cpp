// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancygpumap.h"

#include "mapchunk.h"
#include "mapregion.h"
#include "occupancymap.h"
#include "occupancyutil.h"
#include "ohmdefaultlayers.h"

#include "gpucache.h"
#include "gpulayercache.h"

#include "private/occupancygpumapdetail.h"
#include "private/occupancymapdetail.h"

#include <gpubuffer.h>
#include <gpuevent.h>
#include <gpupinnedbuffer.h>
#include <gpuplatform.h>

#include <glm/ext.hpp>

#include <cassert>
#include <functional>
#include <iostream>
#include <initializer_list>

#define DEBUG_RAY 0

#if DEBUG_RAY
#pragma optimize("", off)
#endif // DEBUG_RAY

using namespace ohm;

namespace
{
  typedef std::function<void (const glm::i16vec3 &, const glm::dvec3 &, const glm::dvec3 &)> RegionWalkFunction;

  void walkRegions(const OccupancyMap &map,
                   const glm::dvec3 &startPoint, const glm::dvec3 &endPoint,
                   const RegionWalkFunction &func)
  {
    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    glm::i16vec3 startPointKey = map.regionKey(startPoint);
    glm::i16vec3 endPointKey = map.regionKey(endPoint);
    const glm::dvec3 startPointLocal = glm::vec3(startPoint - map.origin());
    const glm::dvec3 endPointLocal = glm::vec3(endPoint- map.origin());

    glm::dvec3 direction = glm::vec3(endPoint - startPoint);
    double length = glm::dot(direction, direction);
    length = (length >= 1e-6) ? std::sqrt(length) : 0;
    direction *= 1.0 / length;

    if (startPointKey == endPointKey)
    {
      func(startPointKey, startPoint, endPoint);
      return;
    }

    int step[3] = { 0 };
    glm::dvec3 region;
    double timeMax[3];
    double timeDelta[3];
    double timeLimit[3];
    double nextRegionBorder;
    double directionAxisInv;
    const glm::dvec3 regionResolution = map.regionSpatialResolution();
    glm::i16vec3 currentKey = startPointKey;

    region = map.regionCentreLocal(currentKey);

    // Compute step direction, increments and maximums along each axis.
    for (unsigned i = 0; i < 3; ++i)
    {
      if (direction[i] != 0)
      {
        directionAxisInv = 1.0 / direction[i];
        step[i] = (direction[i] > 0) ? 1 : -1;
        // Time delta is the ray time between voxel boundaries calculated for each axis.
        timeDelta[i] = regionResolution[i] * std::abs(directionAxisInv);
        // Calculate the distance from the origin to the nearest voxel edge for this axis.
        nextRegionBorder = region[i] + step[i] * 0.5f * regionResolution[i];
        timeMax[i] = (nextRegionBorder - startPointLocal[i]) * directionAxisInv;
        timeLimit[i] = std::abs((endPointLocal[i] - startPointLocal[i]) * directionAxisInv);// +0.5f * regionResolution[i];
      }
      else
      {
        timeMax[i] = timeDelta[i] = std::numeric_limits<double>::max();
        timeLimit[i] = 0;
      }
    }

    bool limitReached = false;
    int axis = 0;
    while (!limitReached && currentKey != endPointKey)
    {
      func(currentKey, startPoint, endPoint);

      if (timeMax[0] < timeMax[2])
      {
        axis = (timeMax[0] < timeMax[1]) ? 0 : 1;
      }
      else
      {
        axis = (timeMax[1] < timeMax[2]) ? 1 : 2;
      }

      limitReached = std::abs(timeMax[axis]) > timeLimit[axis];
      currentKey[axis] += step[axis];
      timeMax[axis] += timeDelta[axis];
    }

    // Touch the last region.
    func(currentKey, startPoint, endPoint);
  }

  inline bool goodRay(const glm::dvec3 &start, const glm::dvec3 &end, double maxRange = 500.0)
  {
    bool isGood = true;
    if (glm::any(glm::isnan(start)))
    {
      // std::cerr << "NAN start point" << std::endl;
      isGood = false;
    }
    if (glm::any(glm::isnan(end)))
    {
      // std::cerr << "NAN end point" << std::endl;
      isGood = false;
    }

    const glm::dvec3 ray = end - start;
    if (maxRange && glm::dot(ray, ray) > maxRange * maxRange)
    {
      // std::cerr << "Ray too long: (" << glm::distance(start, end) << "): " << start << " -> " << end << std::endl;
      isGood = false;
    }

    return isGood;
  }
}

// GPU related functions.
namespace ohm
{
  int initialiseRegionUpdateGpu(gputil::Device &gpu);
  void releaseRegionUpdateGpu();

  int updateRegion(gputil::Device &gpu,
                   gputil::Queue &queue,
                   gputil::Buffer &chunkMem,
                   gputil::Buffer &regionKeyBuffer, gputil::Buffer &regionOffsetBuffer,
                   unsigned regionCount,
                   gputil::Buffer &rayMem, unsigned rayCount,
                   const glm::ivec3 &regionVoxelDimensions, double voxelResolution,
                   float adjustMiss, float adjustHit,
                   float minVoxelValue, float maxVoxelValue,
                   std::initializer_list<gputil::Event> events,
                   gputil::Event *completionEvent);
}


const double GpuMap::DefaultMaxRange = 500.0;


GpuCache *ohm::gpumap::enableGpu(OccupancyMap &map)
{
  return enableGpu(map, GpuCache::GiB / 2, true);
}


GpuCache *ohm::gpumap::enableGpu(OccupancyMap &map, size_t layerGpuMemSize, bool mappableBuffers)
{
  OccupancyMapDetail &mapDetail = *map.detail();
  if (mapDetail.gpuCache)
  {
    return mapDetail.gpuCache;
  }

  initialiseGpuCache(map, layerGpuMemSize, mappableBuffers);
  return mapDetail.gpuCache;
}


void ohm::gpumap::sync(OccupancyMap &map)
{
  if (GpuCache *cache = gpuCache(map))
  {
    for (unsigned i = 0; i < cache->layerCount(); ++i)
    {
      if (GpuLayerCache *layer = cache->layerCache(i))
      {
        layer->syncToMainMemory();
      }
    }
  }
}


void ohm::gpumap::sync(OccupancyMap &map, unsigned layerIndex)
{
  if (GpuCache *cache = gpuCache(map))
  {
    if (GpuLayerCache *layer = cache->layerCache(layerIndex))
    {
      layer->syncToMainMemory();
    }
  }
}


GpuCache *ohm::gpumap::gpuCache(OccupancyMap &map)
{
  return map.detail()->gpuCache;
}


GpuMap::GpuMap(OccupancyMap *map, bool borrowedMap, unsigned expectedPointCount, size_t gpuMemSize)
  : _imp(new GpuMapDetail(map, borrowedMap))
{
  gpumap::enableGpu(*map);
  GpuCache &gpuCache = *map->detail()->gpuCache;
  _imp->gpuOk = initialiseRegionUpdateGpu(gpuCache.gpu()) == 0;
  const unsigned preallocRegionCount = 1024u;
  for (int i = 0; i < GpuMapDetail::BUFFERS_COUNT; ++i)
  {
    _imp->rayBuffers[i] = gputil::Buffer(gpuCache.gpu(), sizeof(gputil::float3) * expectedPointCount, gputil::BF_ReadHost);
    _imp->regionKeyBuffers[i] = gputil::Buffer(gpuCache.gpu(), sizeof(gputil::int3) * preallocRegionCount, gputil::BF_ReadHost);
    _imp->regionOffsetBuffers[i] = gputil::Buffer(gpuCache.gpu(), sizeof(gputil::ulong1) * preallocRegionCount, gputil::BF_ReadHost);
  }
  _imp->maxRangeFilter = DefaultMaxRange;
}


GpuMap::~GpuMap()
{
  releaseRegionUpdateGpu();

  if (!_imp->borrowedMap)
  {
    delete _imp->map;
  }

  delete _imp;
}


bool GpuMap::gpuOk() const
{
  return _imp->gpuOk;
}


OccupancyMap &GpuMap::map()
{
  return *_imp->map;
}


const OccupancyMap &GpuMap::map() const
{
  return *_imp->map;
}


bool GpuMap::borrowedMap() const
{
  return _imp->borrowedMap;
}


double GpuMap::maxRangeFilter() const
{
  return _imp->maxRangeFilter;
}


void GpuMap::setMaxRangeFilter(double range)
{
  _imp->maxRangeFilter = range;
}


void GpuMap::syncOccupancy()
{
  if (_imp->map)
  {
    gpumap::sync(*_imp->map, GCID_Occupancy);
  }
}


unsigned GpuMap::integrateRays(const glm::dvec3 *rays, unsigned pointCount, bool endPointsAsOccupied)
{
  if (!_imp->map)
  {
    return 0u;
  }

  if (!_imp->gpuOk)
  {
    return 0u;
  }

  OccupancyMap &map = *_imp->map;
  GpuCache *gpuCache = gpumap::enableGpu(map);

  if (!gpuCache)
  {
    return 0u;
  }

  if (pointCount == 0)
  {
    return 0u;
  }

  // Touch the map.
  map.touch();

  // Wait for previous ray operations to complete.
  const int bufIdx = _imp->nextBuffersIndex;
  waitOnPreviousOperation(bufIdx);

  // Get the GPU cache.
  GpuLayerCache &layerCache = *gpuCache->layerCache(GCID_Occupancy);
  _imp->batchMarker = layerCache.beginBatch();

  // Region walking function tracking which regions are affected by a ray.
  auto regionFunc = [this](const glm::i16vec3 &regionKey, const glm::dvec3 &origin, const glm::dvec3 &sample) {
    const unsigned regionHash = MapRegion::Hash::calculate(regionKey);
    if (_imp->findRegion(regionHash, regionKey) == _imp->regions.end())
    {
      _imp->regions.insert(std::make_pair(regionHash, regionKey));
    }
  };

  // Reserve GPU memory for the rays.
  _imp->rayBuffers[bufIdx].resize(sizeof(gputil::float3) * pointCount);
  gputil::PinnedBuffer rayBuffer(_imp->rayBuffers[bufIdx], gputil::PinWrite);
  glm::vec4 rayStart, rayEnd;

  // TODO: break up long lines. Requires the kernel knows which are real end points and which aren't.
  // Build region set and upload rays.
  _imp->regions.clear();
  unsigned uploadCount = 0u;
  for (unsigned i = 0; i < pointCount; i += 2)
  {
    rayStart = glm::vec4(glm::vec3(rays[i + 0]), 0);
    rayEnd = glm::vec4(glm::vec3(rays[i + 1]), 0);
    if (!goodRay(rayStart, rayEnd, _imp->maxRangeFilter))
    {
      continue;
    }
    rayBuffer.write(glm::value_ptr(rayStart), sizeof(glm::vec3), (uploadCount + 0) * sizeof(gputil::float3));
    rayBuffer.write(glm::value_ptr(rayEnd), sizeof(glm::vec3), (uploadCount + 1) * sizeof(gputil::float3));
    uploadCount += 2;
    // std::cout << i / 2 << ' ' << _imp->map->voxelKey(rays[i + 0]) << " -> " << _imp->map->voxelKey(rays[i + 1])
    //          << "  <=>  " << rays[i + 0] << " -> " << rays[i + 1] << std::endl;
    walkRegions(*_imp->map, rays[i + 0], rays[i + 1], regionFunc);
  }

  // Asynchronous unpin. Kernels will wait on the associated event.
  rayBuffer.unpin(&layerCache.gpuQueue(), nullptr, &_imp->rayUploadEvents[bufIdx]);
  _imp->rayCounts[bufIdx] = unsigned(uploadCount / 2);

  if (uploadCount == 0)
  {
    return 0u;
  }

  // Size the region buffers.
  _imp->regionKeyBuffers[bufIdx].gputil::Buffer::elementsResize<gputil::int3>(_imp->regions.size());
  _imp->regionOffsetBuffers[bufIdx].gputil::Buffer::elementsResize<gputil::ulong1>(_imp->regions.size());

  // Execute on each region.
  gputil::PinnedBuffer regionsBuffer(_imp->regionKeyBuffers[bufIdx], gputil::PinWrite);
  gputil::PinnedBuffer offsetsBuffer(_imp->regionOffsetBuffers[bufIdx], gputil::PinWrite);
  // Note: bufIdx may have change when calling enqueueRegion. Do not use after this point.
  for (auto &&regionDetails : _imp->regions)
  {
    enqueueRegion(regionDetails.first, regionDetails.second, regionsBuffer, offsetsBuffer, endPointsAsOccupied, true);
  }

  finaliseBatch(regionsBuffer, offsetsBuffer, endPointsAsOccupied);

  return uploadCount;
}


GpuCache *GpuMap::gpuCache() const
{
  return _imp->map->detail()->gpuCache;
}


void GpuMap::waitOnPreviousOperation(int bufferIndex)
{
  // Wait first on the event known to complete last.
  _imp->regionUpdateEvents[bufferIndex].wait();
  _imp->regionUpdateEvents[bufferIndex].release();

  _imp->rayUploadEvents[bufferIndex].wait();
  _imp->rayUploadEvents[bufferIndex].release();

  _imp->regionKeyUploadEvents[bufferIndex].wait();
  _imp->regionKeyUploadEvents[bufferIndex].release();

  _imp->regionOffsetUploadEvents[bufferIndex].wait();
  _imp->regionOffsetUploadEvents[bufferIndex].release();
}



void GpuMap::enqueueRegion(unsigned regionHash, const glm::i16vec3 &regionKey,
                           gputil::PinnedBuffer &regionsBuffer, gputil::PinnedBuffer &offsetsBuffer,
                           bool allowRetry, bool endPointsAsOccupied)
{
  // Upload chunk to GPU.
  MapChunk *chunk = nullptr;
  OccupancyMapDetail *map = _imp->map->detail();
  gputil::Event uploadEvent;
  gputil::Buffer *gpuMem = nullptr;
  gputil::ulong1 memOffset = 0;
  GpuLayerCache::CacheStatus status;

  int bufIdx = _imp->nextBuffersIndex;
  GpuCache &gpuCache = *this->gpuCache();
  GpuLayerCache &layerCache = *gpuCache.layerCache(GCID_Occupancy);
  memOffset = (gputil::ulong1)layerCache.upload(*_imp->map, regionKey, chunk, &uploadEvent, &status, _imp->batchMarker,
                                                GpuLayerCache::AllowRegionCreate);

  if (status != GpuLayerCache::CacheFull)
  {
    // std::cout << "region: [" << regionKey.x << ' ' << regionKey.y << ' ' << regionKey.z << ']' << std::endl;
    gputil::int3 gpuRKey = { regionKey.x, regionKey.y, regionKey.z };
    regionsBuffer.write(&gpuRKey, sizeof(gpuRKey), _imp->regionCounts[bufIdx] * sizeof(gpuRKey));
    offsetsBuffer.write(&memOffset, sizeof(memOffset), _imp->regionCounts[bufIdx] * sizeof(memOffset));
    ++_imp->regionCounts[bufIdx];
  }
  else if (allowRetry)
  {
    const int previousBufIdx = bufIdx;
    finaliseBatch(regionsBuffer, offsetsBuffer, endPointsAsOccupied);

    // Repin these buffers, but the index has changed.
    const unsigned regionsProcessed = _imp->regionCounts[bufIdx];
    bufIdx = _imp->nextBuffersIndex;
    waitOnPreviousOperation(bufIdx);

    // Copy the rays buffer from the batch we just finalised.
    gputil::copyBuffer(_imp->rayBuffers[bufIdx], _imp->rayBuffers[previousBufIdx],
                       &gpuCache.gpuQueue(), nullptr, &_imp->rayUploadEvents[previousBufIdx]);
    _imp->rayCounts[bufIdx] = _imp->rayCounts[previousBufIdx];

    // This statement should always be true, but it would be bad to underflow.
    if (regionsProcessed < _imp->regions.size())
    {
      // Size the region buffers.
      _imp->regionKeyBuffers[bufIdx].gputil::Buffer::elementsResize<gputil::int3>(_imp->regions.size() - regionsProcessed);
      _imp->regionOffsetBuffers[bufIdx].gputil::Buffer::elementsResize<gputil::ulong1>(_imp->regions.size() - regionsProcessed);

      regionsBuffer = gputil::PinnedBuffer(_imp->regionKeyBuffers[bufIdx], gputil::PinRead);
      offsetsBuffer = gputil::PinnedBuffer(_imp->regionOffsetBuffers[bufIdx], gputil::PinRead);

      // Try again, but don't allow retry.
      enqueueRegion(regionHash, regionKey, regionsBuffer, offsetsBuffer, endPointsAsOccupied, false);
    }
  }

  // Mark the region as dirty.
  chunk->dirtyStamp = chunk->touchedStamps[DL_Occupancy] = _imp->map->stamp();
}

void GpuMap::finaliseBatch(gputil::PinnedBuffer &regionsBuffer, gputil::PinnedBuffer &offsetsBuffer, bool endPointsAsOccupied)
{
  const int bufIdx = _imp->nextBuffersIndex;
  const OccupancyMapDetail *map = _imp->map->detail();

  // Complete region data upload.
  GpuCache &gpuCache = *this->gpuCache();
  GpuLayerCache &layerCache = *gpuCache.layerCache(GCID_Occupancy);
  regionsBuffer.unpin(&layerCache.gpuQueue(), nullptr, &_imp->regionKeyUploadEvents[bufIdx]);
  offsetsBuffer.unpin(&layerCache.gpuQueue(), nullptr, &_imp->regionOffsetUploadEvents[bufIdx]);

  // Enqueue update kernel.
  updateRegion(layerCache.gpu(), layerCache.gpuQueue(),
               *layerCache.buffer(),
               _imp->regionKeyBuffers[bufIdx], _imp->regionOffsetBuffers[bufIdx],
               _imp->regionCounts[bufIdx],
               _imp->rayBuffers[bufIdx], _imp->rayCounts[bufIdx],
               map->regionVoxelDimensions, map->resolution,
               map->missValue, (endPointsAsOccupied) ? map->hitValue : map->missValue,
               map->minNodeValue, map->maxNodeValue,
               { _imp->rayUploadEvents[bufIdx], _imp->regionKeyUploadEvents[bufIdx], _imp->regionOffsetUploadEvents[bufIdx] },
               &_imp->regionUpdateEvents[bufIdx]);
  //layerCache.gpuQueue().flush();

  // Update most recent chunk GPU event.
  layerCache.updateEvents(_imp->batchMarker, _imp->regionUpdateEvents[bufIdx]);
  //layerCache.updateEvent(*chunk, updateEvent);

  // std::cout << _imp->regionCounts[bufIdx] << " regions\n" << std::flush;

  _imp->regionCounts[bufIdx] = 0;
  // Cycle odd numbers to avoid zero which is a special case value.
  _imp->batchMarker = layerCache.beginBatch();
  _imp->nextBuffersIndex = 1 - _imp->nextBuffersIndex;
}
