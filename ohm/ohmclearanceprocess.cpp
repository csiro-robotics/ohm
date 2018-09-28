// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmclearanceprocess.h"

#include "gpucache.h"
#include "gpukey.h"
#include "mapcache.h"
#include "mapchunk.h"
#include "maplayout.h"
#include "mapregion.h"
#include "occupancykey.h"
#include "occupancymap.h"
#include "occupancyqueryflag.h"
#include "ohmdefaultlayers.h"
#include "private/clearanceprocessdetail.h"
#include "private/maplayoutdetail.h"
#include "private/occupancygpumapdetail.h"
#include "private/occupancymapdetail.h"
#include "private/occupancynodealgorithms.h"
#include "private/occupancyqueryalg.h"

#include "occupancyutil.h"

#include "gpulayercache.h"
#include "occupancygpumap.h"
#include <gpuplatform.h>

#include <3esservermacros.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#ifdef OHM_THREADS
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>
#endif  // OHM_THREADS

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>

#ifdef OHM_PROFILE
#define PROFILING 1
#endif  // OHM_PROFILE

#include <profile.h>

using namespace ohm;

#define VALIDATE_VALUES_UNCHANGED 0

#include "gpupinnedbuffer.h"
#include "occupancygpu.h"

namespace
{
  void regionClearanceProcessCpuBlock(OccupancyMap &map, ClearanceProcessDetail &query, const glm::ivec3 &blockStart,
                                      const glm::ivec3 &blockEnd, const glm::i16vec3 &regionKey, MapChunk *chunk,
                                      const glm::ivec3 &voxelSearchHalfExtents)
  {
    OccupancyMapDetail &mapData = *map.detail();
    OccupancyKey nodeKey(nullptr);
    float range;

    nodeKey.setRegionKey(regionKey);
    for (int z = blockStart.z; z < blockEnd.z; ++z)
    {
      nodeKey.setLocalAxis(2, z);
      for (int y = blockStart.y; y < blockEnd.y; ++y)
      {
        nodeKey.setLocalAxis(1, y);
        for (int x = blockStart.x; x < blockEnd.x; ++x)
        {
          nodeKey.setLocalAxis(0, x);
          OccupancyNode node = OccupancyNode(nodeKey, chunk, &mapData);
          if (!node.isNull())
          {
            range = calculateNearestNeighbour(
              nodeKey, map, voxelSearchHalfExtents, (query.queryFlags & QF_UnknownAsOccupied) != 0,
              false, query.searchRadius, query.axisScaling,
              (query.queryFlags & QF_ReportUnscaledResults) != 0);
            node.setClearance(range);
          }
        }
      }
    }
  }


  void regionSeedFloodFillCpuBlock(OccupancyMap &map, ClearanceProcessDetail &query, const glm::ivec3 &blockStart,
                                   const glm::ivec3 &blockEnd, const glm::i16vec3 &regionKey, MapChunk *chunk,
                                   const glm::ivec3 &voxelSearchHalfExtents)
  {
    OccupancyMapDetail &mapData = *map.detail();
    OccupancyKey nodeKey(nullptr);

    nodeKey.setRegionKey(regionKey);
    for (int z = blockStart.z; z < blockEnd.z; ++z)
    {
      nodeKey.setLocalAxis(2, z);
      for (int y = blockStart.y; y < blockEnd.y; ++y)
      {
        nodeKey.setLocalAxis(1, y);
        for (int x = blockStart.x; x < blockEnd.x; ++x)
        {
          nodeKey.setLocalAxis(0, x);
          OccupancyNode node = OccupancyNode(nodeKey, chunk, &mapData);
          if (!node.isNull())
          {
            if (node.isOccupied() || ((query.queryFlags & QF_UnknownAsOccupied) != 0 && node.isUncertain()))
            {
              node.setClearance(0.0f);
            }
            else
            {
              node.setClearance(-1.0f);
            }
          }
        }
      }
    }
  }


  void regionFloodFillStepCpuBlock(OccupancyMap &map, ClearanceProcessDetail &query, const glm::ivec3 &blockStart,
                                   const glm::ivec3 &blockEnd, const glm::i16vec3 &regionKey, MapChunk *chunk,
                                   const glm::ivec3 &voxelSearchHalfExtents)
  {
    OccupancyMapDetail &mapData = *map.detail();
    OccupancyKey nodeKey(nullptr);
    OccupancyKey neighbourKey(nullptr);
    float nodeRange;

    nodeKey.setRegionKey(regionKey);
    for (int z = blockStart.z; z < blockEnd.z; ++z)
    {
      nodeKey.setLocalAxis(2, z);
      for (int y = blockStart.y; y < blockEnd.y; ++y)
      {
        nodeKey.setLocalAxis(1, y);
        for (int x = blockStart.x; x < blockEnd.x; ++x)
        {
          nodeKey.setLocalAxis(0, x);
          OccupancyNode node = OccupancyNode(nodeKey, chunk, &mapData);
          if (!node.isNull())
          {
            nodeRange = node.clearance();
            for (int nz = -1; nz <= 1; ++nz)
            {
              for (int ny = -1; ny <= 1; ++ny)
              {
                for (int nx = -1; nx <= 1; ++nx)
                {
                  if (nx || ny || nz)
                  {
                    // This is wrong. It will propagate changed from this iteration. Not what we want.
                    neighbourKey = nodeKey;
                    map.moveKey(neighbourKey, nx, ny, nz);
                    OccupancyNodeConst neighbour = map.node(neighbourKey);
                    // Get neighbour value.
                    if (!neighbour.isNull())
                    {
                      float neighbourRange = (neighbour.isNull()) ? neighbour.clearance() : -1.0f;
                      if (neighbourRange >= 0)
                      {
                        // Adjust by range to neighbour.
                        neighbourRange += glm::length(glm::vec3(nx, ny, nz));
                        if (neighbourRange < nodeRange)
                        {
                          node.setClearance(neighbourRange);
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }


  unsigned regionClearRanges(OccupancyMap &map, ClearanceProcessDetail &query, const glm::i16vec3 &regionKey,
                             ClosestResult &closest)
  {
    OccupancyMapDetail &mapData = *map.detail();
    auto chunkSearch = mapData.findRegion(regionKey);
    glm::ivec3 voxelSearchHalfExtents;
    MapChunk *chunk = nullptr;

    if (chunkSearch == mapData.chunks.end())
    {
      // The entire region is unknown space. Nothing to do as we can't write to anything.
      return 0;
    }

    chunk = chunkSearch->second;
    float *clearance = chunk->layout->layer(DL_Clearance).voxelsAs<float>(*chunk);
    for (size_t i = 0; i < map.regionVoxelVolume(); ++i)
    {
      clearance[i] = -1.0f;
    }

    return unsigned(map.regionVoxelVolume());
  }


  unsigned regionClearanceProcessCpu(OccupancyMap &map, ClearanceProcessDetail &query, const glm::i16vec3 &regionKey)
  {
    OccupancyMapDetail &mapData = *map.detail();
    const auto chunkSearch = mapData.findRegion(regionKey);
    glm::ivec3 voxelSearchHalfExtents;
    MapChunk *chunk = nullptr;

    if (chunkSearch == mapData.chunks.end())
    {
      // The entire region is unknown space. Nothing to do as we can't write to anything.
      return 0;
    }

    voxelSearchHalfExtents = ohm::calculateVoxelSearchHalfExtents(map, query.searchRadius);
    chunk = chunkSearch->second;

#ifdef OHM_THREADS
    const auto parallelQueryFunc = [&query, &map, regionKey, chunk, voxelSearchHalfExtents]
                                   (const tbb::blocked_range3d<int> &range) {
      regionClearanceProcessCpuBlock(map, query,
                                     glm::ivec3(range.cols().begin(), range.rows().begin(), range.pages().begin()),
                                     glm::ivec3(range.cols().end(), range.rows().end(), range.pages().end()), regionKey,
                                     chunk, voxelSearchHalfExtents);
    };
    tbb::parallel_for(tbb::blocked_range3d<int>(0, mapData.regionVoxelDimensions.z, 0, mapData.regionVoxelDimensions.y,
                                                0, mapData.regionVoxelDimensions.x),
                      parallelQueryFunc);

#else   // OHM_THREADS
    regionClearanceProcessCpuBlock(map, query, glm::ivec3(0, 0, 0), mapData.regionVoxelDimensions, regionKey, chunk,
                                   voxelSearchHalfExtents);
#endif  // OHM_THREADS

    return unsigned(map.regionVoxelVolume());
  }

  unsigned regionSeedFloodFillCpu(OccupancyMap &map, ClearanceProcessDetail &query, const glm::i16vec3 &regionKey,
                                  const glm::ivec3 &voxelExtents, const glm::ivec3 &calcExtents)
  {
    OccupancyMapDetail &mapData = *map.detail();
    const auto chunkSearch = mapData.findRegion(regionKey);
    glm::ivec3 voxelSearchHalfExtents;
    MapChunk *chunk = nullptr;

    if (chunkSearch == mapData.chunks.end())
    {
      // The entire region is unknown space. Nothing to do as we can't write to anything.
      return 0;
    }

    voxelSearchHalfExtents = ohm::calculateVoxelSearchHalfExtents(map, query.searchRadius);
    chunk = chunkSearch->second;

#ifdef OHM_THREADS
    const auto parallelQueryFunc = [&query, &map, regionKey, chunk,
                                   voxelSearchHalfExtents](const tbb::blocked_range3d<int> &range) {
      regionSeedFloodFillCpuBlock(map, query,
                                  glm::ivec3(range.cols().begin(), range.rows().begin(), range.pages().begin()),
                                  glm::ivec3(range.cols().end(), range.rows().end(), range.pages().end()), regionKey,
                                  chunk, voxelSearchHalfExtents);
    };
    tbb::parallel_for(tbb::blocked_range3d<int>(0, mapData.regionVoxelDimensions.z, 0, mapData.regionVoxelDimensions.y,
                                                0, mapData.regionVoxelDimensions.x),
                      parallelQueryFunc);

#else   // OHM_THREADS
    regionSeedFloodFillCpuBlock(map, query, glm::ivec3(0, 0, 0), mapData.regionVoxelDimensions, regionKey, chunk,
                                voxelSearchHalfExtents);
#endif  // OHM_THREADS

    return calcExtents.x * calcExtents.y * calcExtents.z;
  }

  unsigned regionFloodFillStepCpu(OccupancyMap &map, ClearanceProcessDetail &query, const glm::i16vec3 &regionKey,
                                  const glm::ivec3 &voxelExtents, const glm::ivec3 &calcExtents)
  {
    OccupancyMapDetail &mapData = *map.detail();
    const auto chunkSearch = mapData.findRegion(regionKey);
    glm::ivec3 voxelSearchHalfExtents;
    MapChunk *chunk = nullptr;

    if (chunkSearch == mapData.chunks.end())
    {
      // The entire region is unknown space. Nothing to do as we can't write to anything.
      return 0;
    }

    voxelSearchHalfExtents = ohm::calculateVoxelSearchHalfExtents(map, query.searchRadius);
    chunk = chunkSearch->second;

#ifdef OHM_THREADS
    const auto parallelQueryFunc = [&query, &map, regionKey, chunk,
                                   voxelSearchHalfExtents](const tbb::blocked_range3d<int> &range) {
      regionFloodFillStepCpuBlock(map, query,
                                  glm::ivec3(range.cols().begin(), range.rows().begin(), range.pages().begin()),
                                  glm::ivec3(range.cols().end(), range.rows().end(), range.pages().end()), regionKey,
                                  chunk, voxelSearchHalfExtents);
    };
    tbb::parallel_for(tbb::blocked_range3d<int>(0, mapData.regionVoxelDimensions.z, 0, mapData.regionVoxelDimensions.y,
                                                0, mapData.regionVoxelDimensions.x),
                      parallelQueryFunc);

#else   // OHM_THREADS
    regionFloodFillStepCpuBlock(map, query, glm::ivec3(0, 0, 0), mapData.regionVoxelDimensions, regionKey, chunk,
                                voxelSearchHalfExtents);
#endif  // OHM_THREADS

    return calcExtents.x * calcExtents.y * calcExtents.z;
  }
}


ClearanceProcess::ClearanceProcess()
  : _imp(new ClearanceProcessDetail)
{
  _imp->gpuQuery = std::unique_ptr<RoiRangeFill>(new RoiRangeFill(gpuDevice()));
}


ClearanceProcess::ClearanceProcess(float searchRadius, unsigned queryFlags)
  : ClearanceProcess()
{
  setSearchRadius(searchRadius);
  setQueryFlags(queryFlags);
}


ClearanceProcess::~ClearanceProcess()
{
  ClearanceProcessDetail *d = imp();
  delete d;
  _imp = nullptr;
}


float ClearanceProcess::searchRadius() const
{
  const ClearanceProcessDetail *d = imp();
  return d->searchRadius;
}


void ClearanceProcess::setSearchRadius(float range)
{
  ClearanceProcessDetail *d = imp();
  d->searchRadius = range;
}


unsigned ClearanceProcess::queryFlags() const
{
  const ClearanceProcessDetail *d = imp();
  return d->queryFlags;
}


void ClearanceProcess::setQueryFlags(unsigned flags)
{
  ClearanceProcessDetail *d = imp();
  d->queryFlags = flags;
}


glm::vec3 ClearanceProcess::axisScaling() const
{
  const ClearanceProcessDetail *d = imp();
  return d->axisScaling;
}


void ClearanceProcess::setAxisScaling(const glm::vec3 &scaling)
{
  ClearanceProcessDetail *d = imp();
  d->axisScaling = scaling;
}


void ClearanceProcess::reset()
{
  ClearanceProcessDetail *d = imp();
  d->resetWorking();
}


int ClearanceProcess::update(OccupancyMap &map, double timeSlice)
{
  ClearanceProcessDetail *d = imp();

  using Clock = std::chrono::high_resolution_clock;
  const auto startTime = Clock::now();
  double elapsedSec = 0;

  // Fetch outdated regions.
  // Results must be ordered by region touch stamp.
  // Add to previous results. There may be repeated regions.
  // FIXME: if a region is added, the so must its neighbours be due to the flooding effect of the update.

  if (!d->haveWork())
  {
    d->getWork(map);
    const auto curTime = Clock::now();
    elapsedSec = std::chrono::duration_cast<std::chrono::duration<double>>(curTime - startTime).count();
  }

  // Drop existing cached occupancy values before continuing.
  GpuCache *gpuCache = gpumap::gpuCache(map);
  if (gpuCache)
  {
    GpuLayerCache *clearanceCache = gpuCache->layerCache(GCID_Clearance);
    clearanceCache->syncToMainMemory();
    clearanceCache->clear();
  }

  unsigned totalProcessed = 0;
  const glm::i16vec3 step(1);
  while (d->haveWork() && (timeSlice <= 0 || elapsedSec < timeSlice))
  {
    // Iterate dirty regions
    unsigned processedCount = 0;
    const glm::dvec3 regionExtents = map.regionSpatialResolution();

    updateRegion(map, d->currentDirtyCursor, false);
    //updateExtendedRegion(map, d->mapStamp, d->currentDirtyCursor, d->currentDirtyCursor + step - glm::i16vec3(1));
    d->stepCursor(step);

    processedCount += volumeOf(step);
    totalProcessed += volumeOf(step);

    if (!d->haveWork())
    {
      d->getWork(map);
    }

    const auto curTime = Clock::now();
    elapsedSec = std::chrono::duration_cast<std::chrono::duration<double>>(curTime - startTime).count();
  }

  return (totalProcessed != 0 || d->haveWork()) ? MPR_Progressing : MPR_UpToDate;
}


void ohm::ClearanceProcess::calculateForExtents(OccupancyMap &map, const glm::dvec3 &minExtents,
                                                const glm::dvec3 &maxExtents, bool force)
{
  ClearanceProcessDetail *d = imp();

  const glm::i16vec3 minRegion = map.regionKey(minExtents);
  const glm::i16vec3 maxRegion = map.regionKey(maxExtents);
  //// Process in blocks containing up to this many regions in each dimension.
  //const int blockMax = 5;

  glm::i16vec3 regionKey, regionMax;

  // Drop existing cached occupancy values before continuing.
  GpuCache *gpuCache = gpumap::gpuCache(map);
  if (gpuCache)
  {
    GpuLayerCache *clearanceCache = gpuCache->layerCache(GCID_Clearance);
    clearanceCache->syncToMainMemory();
    clearanceCache->clear();
  }

  const uint64_t targetStamp = map.stamp();
  for (int z = minRegion.z; z <= maxRegion.z; ++z)
  {
    regionKey.z = z;
    for (int y = minRegion.y; y <= maxRegion.y; ++y)
    {
      regionKey.y = y;
      for (int x = minRegion.x; x <= maxRegion.x; ++x)
      {
        regionKey.x = x;
        updateRegion(map, regionKey, force);
      }
    }
  }
}

bool ClearanceProcess::updateRegion(OccupancyMap &map, const glm::i16vec3 &regionKey, bool force)
{
  // Get the next region.
  ClearanceProcessDetail *d = imp();

  MapChunk *region = map.region(regionKey, (d->queryFlags & QF_InstantiateUnknown));
  if (!region)
  {
    return false;
  }

  // Explore the region neighbours to see if they are out of date. That would invalidate this regon.
  glm::i16vec3 neighbourKey;

  // We are dirty if any input region has updated occupancy values since the last region->touchedStamps[DL_Clearance]
  // value. We iterate to work out the maximum touchedStamps[DL_Occupancy] value in the neighbourhood and compare
  // that to our region DL_Clerance stamp. Dirty if clearance stamp is lower. The target value also sets the
  // new stamp to apply to the region clearance stamp.
  uint64_t targetUpdateStamp = region->touchedStamps[DL_Occupancy];
  for (int z = -1; z <= 1; ++z)
  {
    neighbourKey.z = regionKey.z + z;
    for (int y = -1; y <= 1; ++y)
    {
      neighbourKey.y = regionKey.y + y;
      for (int x = -1; x <= 1; ++x)
      {
        neighbourKey.x = regionKey.x + x;
        MapChunk *neighbour = map.region(neighbourKey, false);
        if (neighbour)
        {
          targetUpdateStamp = std::max(targetUpdateStamp, uint64_t(neighbour->touchedStamps[DL_Occupancy]));
        }
      }
    }
  }

  if (!force && region->touchedStamps[DL_Clearance] >= targetUpdateStamp)
  {
    // Nothing to update in these extents.
    return false;
  }

  // Cache the map stamp now in case it changes concurrently.
  const uint64_t targetStamp = map.stamp();

  // Debug highlight the region.
  TES_BOX_W(g_3es, TES_COLOUR(FireBrick), uint32_t((size_t)d->map), glm::value_ptr(d->map->regionSpatialCentre(regionKey)), glm::value_ptr(d->map->regionSpatialExtents()));

  if ((d->queryFlags & QF_GpuEvaluate) && d->gpuQuery->valid())
  {
    PROFILE(occupancyClearanceProcessGpu);
    d->gpuQuery->setAxisScaling(d->axisScaling);
    d->gpuQuery->setSearchRadius(d->searchRadius);
    d->gpuQuery->setQueryFlags(d->queryFlags);
    d->gpuQuery->calculateForRegion(map, regionKey);
  }
  else
  {
    if (d->queryFlags & QF_GpuEvaluate)
    {
      std::cerr << "ClearanceProcess requested GPU unavailable. Using CPU." << std::endl;
    }

    std::function<unsigned(OccupancyMap &, ClearanceProcessDetail &, const glm::i16vec3 &, ClosestResult &)> queryFunc;

#if 1
    queryFunc = [](OccupancyMap &map, ClearanceProcessDetail &query,
                                              const glm::i16vec3 &regionKey, ClosestResult &
                                              /* closest */) -> unsigned {
      return regionClearanceProcessCpu(map, query, regionKey);
    };
    ClosestResult closest;  // Not used.
    ohm::occupancyQueryRegions(map, *d, closest, map.regionSpatialMin(regionKey) - glm::dvec3(d->searchRadius),
                               map.regionSpatialMin(regionKey) + glm::dvec3(d->searchRadius), queryFunc);
#else   // #
    // Flood fill experiment. Incorrect as it will propagate changes from the current iteration
    // to some neighbours. Protect against that it it can be much faster than the brute force method.
    queryFunc = [&voxelExtents, &calcExtents](OccupancyMap &map, ClearanceProcessDetail &query,
                                              const glm::i16vec3 &regionKey, ClosestResult &
                                              /* closest */) -> unsigned {
      // return regionClearanceProcessCpu(map, query, regionKey, voxelExtents, calcExtents);
      return regionSeedFloodFillCpu(map, query, regionKey, voxelExtents, calcExtents);
    };

    ClosestResult closest;  // Not used.
    ohm::occupancyQueryRegions(map, *d, closest, minExtents - glm::dvec3(d->searchRadius),
                               maxExtents + glm::dvec3(d->searchRadius), queryFunc);

    queryFunc = [&voxelExtents, &calcExtents, map](OccupancyMap &constMap, ClearanceProcessDetail &query,
                                                   const glm::i16vec3 &regionKey, ClosestResult &
                                                   /* closest */) -> unsigned {
      return regionFloodFillStepCpu(map, query, regionKey, voxelExtents, calcExtents);
    };

    for (unsigned i = 0; i < voxelPadding; ++i)
    {
      ohm::occupancyQueryRegions(map, *d, closest, d->minExtents - glm::dvec3(d->searchRadius),
                                 d->maxExtents + glm::dvec3(d->searchRadius), queryFunc);
    }
#endif  // #
  }

  TES_SERVER_UPDATE(g_3es, 0.0f);
  // Delete debug objects.
  // TES_SPHERE_END(g_3es, uint32_t((size_t)map));
  TES_BOX_END(g_3es, uint32_t((size_t)map));

  // Regions are up to date *now*.
  region->touchedStamps[DL_Clearance] = targetUpdateStamp;
  return true;
}


ClearanceProcessDetail *ClearanceProcess::imp()
{
  return static_cast<ClearanceProcessDetail *>(_imp);
}


const ClearanceProcessDetail *ClearanceProcess::imp() const
{
  return static_cast<const ClearanceProcessDetail *>(_imp);
}
