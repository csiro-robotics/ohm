// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ClearanceProcess.h"

#include "GpuCache.h"
#include "GpuLayerCache.h"
#include "GpuMap.h"
#include "OhmGpu.h"

#include "private/ClearanceProcessDetail.h"

#include <ohm/DefaultLayer.h>
#include <ohm/Key.h>
#include <ohm/MapChunk.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/QueryFlag.h>
#include <ohm/VoxelData.h>

#include <ohm/private/MapLayoutDetail.h>
#include <ohm/private/OccupancyMapDetail.h>
#include <ohm/private/OccupancyQueryAlg.h>
#include <ohm/private/VoxelAlgorithms.h>

#include <ohmutil/Profile.h>

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
#include <memory>

using namespace ohm;

#define VALIDATE_VALUES_UNCHANGED 0

namespace
{
void regionClearanceProcessCpuBlock(OccupancyMap &map,              // NOLINT(google-runtime-references)
                                    ClearanceProcessDetail &query,  // NOLINT(google-runtime-references)
                                    const glm::ivec3 &block_start, const glm::ivec3 &block_end,
                                    const glm::i16vec3 &region_key, MapChunk *chunk,
                                    const glm::ivec3 &voxel_search_half_extents)
{
  Key voxel_key(nullptr);
  Voxel<float> voxel(&map, map.layout().clearanceLayer());
  float range;

  if (!voxel.isLayerValid())
  {
    return;
  }

  voxel_key.setRegionKey(region_key);
  for (int z = block_start.z; z < block_end.z; ++z)
  {
    voxel_key.setLocalAxis(2, z);
    for (int y = block_start.y; y < block_end.y; ++y)
    {
      voxel_key.setLocalAxis(1, y);
      for (int x = block_start.x; x < block_end.x; ++x)
      {
        voxel_key.setLocalAxis(0, x);
        voxel.setKey(voxel_key, chunk);
        // if (!voxel.isNull()) // Don't think this is possible any more.
        {
          range = calculateNearestNeighbour(voxel_key, map, voxel_search_half_extents,
                                            (query.query_flags & kQfUnknownAsOccupied) != 0, false, query.search_radius,
                                            query.axis_scaling, (query.query_flags & kQfReportUnscaledResults) != 0);
          voxel.write(range);
        }
      }
    }
  }
}


void regionSeedFloodFillCpuBlock(OccupancyMap &map,              // NOLINT(google-runtime-references)
                                 ClearanceProcessDetail &query,  // NOLINT(google-runtime-references)
                                 const glm::ivec3 &block_start, const glm::ivec3 &block_end,
                                 const glm::i16vec3 &region_key, MapChunk *chunk,
                                 const glm::ivec3 & /*voxel_search_half_extents*/)
{
  Key voxel_key(nullptr);

  if (!chunk)
  {
    return;
  }

  Voxel<float> occupancy(&map, map.layout().occupancyLayer());
  Voxel<float> clearance(&map, map.layout().clearanceLayer());

  if (!occupancy.isLayerValid() && !clearance.isLayerValid())
  {
    return;
  }

  voxel_key.setRegionKey(region_key);
  for (int z = block_start.z; z < block_end.z; ++z)
  {
    voxel_key.setLocalAxis(2, z);
    for (int y = block_start.y; y < block_end.y; ++y)
    {
      voxel_key.setLocalAxis(1, y);
      for (int x = block_start.x; x < block_end.x; ++x)
      {
        voxel_key.setLocalAxis(0, x);
        occupancy.setKey(voxel_key, chunk);
        clearance.setKey(voxel_key, chunk);
        // if (!voxel.isNull())
        {
          if (isOccupied(occupancy) || ((query.query_flags & kQfUnknownAsOccupied) != 0 && isUnobserved(occupancy)))
          {
            clearance.write(0.0f);
          }
          else
          {
            clearance.write(-1.0f);
          }
        }
      }
    }
  }
}


void regionFloodFillStepCpuBlock(OccupancyMap &map,                   // NOLINT(google-runtime-references)
                                 ClearanceProcessDetail & /*query*/,  // NOLINT(google-runtime-references)
                                 const glm::ivec3 &block_start, const glm::ivec3 &block_end,
                                 const glm::i16vec3 &region_key, MapChunk *chunk,
                                 const glm::ivec3 & /*voxel_search_half_extents*/)
{
  Key voxel_key(nullptr);
  Key neighbour_key(nullptr);
  float voxel_range;

  if (!chunk)
  {
    return;
  }

  Voxel<float> voxel_clearance(&map, map.layout().clearanceLayer());
  Voxel<const float> neighbour(&map, map.layout().clearanceLayer());

  if (!voxel_clearance.isLayerValid() || !neighbour.isLayerValid())
  {
    return;
  }

  voxel_key.setRegionKey(region_key);
  for (int z = block_start.z; z < block_end.z; ++z)
  {
    voxel_key.setLocalAxis(2, z);
    for (int y = block_start.y; y < block_end.y; ++y)
    {
      voxel_key.setLocalAxis(1, y);
      for (int x = block_start.x; x < block_end.x; ++x)
      {
        voxel_key.setLocalAxis(0, x);
        voxel_clearance.setKey(voxel_key);
        // if (!voxel.isNull())
        {
          voxel_clearance.read(&voxel_range);
          for (int nz = -1; nz <= 1; ++nz)
          {
            for (int ny = -1; ny <= 1; ++ny)
            {
              for (int nx = -1; nx <= 1; ++nx)
              {
                if (nx || ny || nz)
                {
                  // This is wrong. It will propagate changed from this iteration. Not what we want.
                  neighbour_key = voxel_key;
                  map.moveKey(neighbour_key, nx, ny, nz);
                  neighbour.setKey(neighbour_key);
                  // Get neighbour value.
                  if (!neighbour.isNull())
                  {
                    float neighbour_range = -1.0f;
                    if (!neighbour.isNull())
                    {
                      neighbour.read(&neighbour_range);
                    }
                    if (neighbour_range >= 0)
                    {
                      // Adjust by range to neighbour.
                      neighbour_range += glm::length(glm::vec3(nx, ny, nz));
                      if (neighbour_range < voxel_range)
                      {
                        voxel_clearance.write(neighbour_range);
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

  chunk->touched_stamps[map.layout().clearanceLayer()] = chunk->dirty_stamp = map.touch();
}


unsigned regionClearanceProcessCpu(OccupancyMap &map,              // NOLINT(google-runtime-references)
                                   ClearanceProcessDetail &query,  // NOLINT(google-runtime-references)
                                   const glm::i16vec3 &region_key)
{
  OccupancyMapDetail &map_data = *map.detail();
  const auto chunk_search = map_data.chunks.find(region_key);
  glm::ivec3 voxel_search_half_extents;

  if (chunk_search == map_data.chunks.end())
  {
    // The entire region is unknown space. Nothing to do as we can't write to anything.
    return 0;
  }

  voxel_search_half_extents = ohm::calculateVoxelSearchHalfExtents(map, query.search_radius);
  MapChunk *chunk = chunk_search->second;

#ifdef OHM_THREADS
  const auto parallel_query_func = [&query, &map, region_key, chunk,
                                    voxel_search_half_extents](const tbb::blocked_range3d<int> &range) {
    regionClearanceProcessCpuBlock(map, query,
                                   glm::ivec3(range.cols().begin(), range.rows().begin(), range.pages().begin()),
                                   glm::ivec3(range.cols().end(), range.rows().end(), range.pages().end()), region_key,
                                   chunk, voxel_search_half_extents);
  };
  tbb::parallel_for(
    tbb::blocked_range3d<int>(0, map_data.region_voxel_dimensions.z, 0, map_data.region_voxel_dimensions.y, 0,
                              map_data.region_voxel_dimensions.x),
    parallel_query_func);

#else   // OHM_THREADS
  regionClearanceProcessCpuBlock(map, query, glm::ivec3(0, 0, 0), map_data.region_voxel_dimensions, region_key, chunk,
                                 voxel_search_half_extents);
#endif  // OHM_THREADS

  return unsigned(map.regionVoxelVolume());
}

unsigned regionSeedFloodFillCpu(OccupancyMap &map,              // NOLINT(google-runtime-references)
                                ClearanceProcessDetail &query,  // NOLINT(google-runtime-references)
                                const glm::i16vec3 &region_key, const glm::ivec3 & /*voxel_extents*/,
                                const glm::ivec3 &calc_extents)
{
  OccupancyMapDetail &map_data = *map.detail();
  const auto chunk_search = map_data.chunks.find(region_key);
  glm::ivec3 voxel_search_half_extents;

  if (chunk_search == map_data.chunks.end())
  {
    // The entire region is unknown space. Nothing to do as we can't write to anything.
    return 0;
  }

  voxel_search_half_extents = ohm::calculateVoxelSearchHalfExtents(map, query.search_radius);
  MapChunk *chunk = chunk_search->second;

#ifdef OHM_THREADS
  const auto parallel_query_func = [&query, &map, region_key, chunk,
                                    voxel_search_half_extents](const tbb::blocked_range3d<int> &range) {
    regionSeedFloodFillCpuBlock(map, query,
                                glm::ivec3(range.cols().begin(), range.rows().begin(), range.pages().begin()),
                                glm::ivec3(range.cols().end(), range.rows().end(), range.pages().end()), region_key,
                                chunk, voxel_search_half_extents);
  };
  tbb::parallel_for(
    tbb::blocked_range3d<int>(0, map_data.region_voxel_dimensions.z, 0, map_data.region_voxel_dimensions.y, 0,
                              map_data.region_voxel_dimensions.x),
    parallel_query_func);

#else   // OHM_THREADS
  regionSeedFloodFillCpuBlock(map, query, glm::ivec3(0, 0, 0), map_data.region_voxel_dimensions, region_key, chunk,
                              voxel_search_half_extents);
#endif  // OHM_THREADS

  return calc_extents.x * calc_extents.y * calc_extents.z;
}

unsigned regionFloodFillStepCpu(OccupancyMap &map,              // NOLINT(google-runtime-references)
                                ClearanceProcessDetail &query,  // NOLINT(google-runtime-references)
                                const glm::i16vec3 &region_key, const glm::ivec3 & /*voxel_extents*/,
                                const glm::ivec3 &calc_extents)
{
  OccupancyMapDetail &map_data = *map.detail();
  const auto chunk_search = map_data.chunks.find(region_key);
  glm::ivec3 voxel_search_half_extents;

  if (chunk_search == map_data.chunks.end())
  {
    // The entire region is unknown space. Nothing to do as we can't write to anything.
    return 0;
  }

  voxel_search_half_extents = ohm::calculateVoxelSearchHalfExtents(map, query.search_radius);
  MapChunk *chunk = chunk_search->second;

#ifdef OHM_THREADS
  const auto parallel_query_func = [&query, &map, region_key, chunk,
                                    voxel_search_half_extents](const tbb::blocked_range3d<int> &range) {
    regionFloodFillStepCpuBlock(map, query,
                                glm::ivec3(range.cols().begin(), range.rows().begin(), range.pages().begin()),
                                glm::ivec3(range.cols().end(), range.rows().end(), range.pages().end()), region_key,
                                chunk, voxel_search_half_extents);
  };
  tbb::parallel_for(
    tbb::blocked_range3d<int>(0, map_data.region_voxel_dimensions.z, 0, map_data.region_voxel_dimensions.y, 0,
                              map_data.region_voxel_dimensions.x),
    parallel_query_func);

#else   // OHM_THREADS
  regionFloodFillStepCpuBlock(map, query, glm::ivec3(0, 0, 0), map_data.region_voxel_dimensions, region_key, chunk,
                              voxel_search_half_extents);
#endif  // OHM_THREADS

  return calc_extents.x * calc_extents.y * calc_extents.z;
}
}  // namespace


ClearanceProcess::ClearanceProcess()
  : imp_(new ClearanceProcessDetail)
{
  imp_->gpu_query = std::make_unique<RoiRangeFill>(gpuDevice());
}


ClearanceProcess::ClearanceProcess(float search_radius, unsigned query_flags)
  : ClearanceProcess()
{
  setSearchRadius(search_radius);
  setQueryFlags(query_flags);
}


ClearanceProcess::~ClearanceProcess()
{
  ClearanceProcessDetail *d = imp();
  delete d;
  imp_ = nullptr;
}


float ClearanceProcess::searchRadius() const
{
  const ClearanceProcessDetail *d = imp();
  return d->search_radius;
}


void ClearanceProcess::setSearchRadius(float range)
{
  ClearanceProcessDetail *d = imp();
  d->search_radius = range;
}


unsigned ClearanceProcess::queryFlags() const
{
  const ClearanceProcessDetail *d = imp();
  return d->query_flags;
}


void ClearanceProcess::setQueryFlags(unsigned flags)
{
  ClearanceProcessDetail *d = imp();
  d->query_flags = flags;
}


glm::vec3 ClearanceProcess::axisScaling() const
{
  const ClearanceProcessDetail *d = imp();
  return d->axis_scaling;
}


void ClearanceProcess::setAxisScaling(const glm::vec3 &scaling)
{
  ClearanceProcessDetail *d = imp();
  d->axis_scaling = scaling;
}


void ClearanceProcess::reset()
{
  ClearanceProcessDetail *d = imp();
  d->resetWorking();
}


void ClearanceProcess::ensureClearanceLayer(OccupancyMap &map)
{
  if (map.layout().clearanceLayer() != -1)
  {
    return;
  }

  // Duplicate the layout, add the layer and update the map, preserving the current map.
  MapLayout updated_layout(map.layout());
  addClearance(updated_layout);
  map.updateLayout(updated_layout, true);
}


int ClearanceProcess::update(OccupancyMap &map, double time_slice)
{
  ClearanceProcessDetail *d = imp();

  // Ensure clearnce layer is present.
  ensureClearanceLayer(map);

  using Clock = std::chrono::high_resolution_clock;
  const auto start_time = Clock::now();
  double elapsed_sec = 0;

  // Fetch outdated regions.
  // Results must be ordered by region touch stamp.
  // Add to previous results. There may be repeated regions.
  // FIXME: if a region is added, the so must its neighbours be due to the flooding effect of the update.

  if (!d->haveWork())
  {
    d->getWork(map);
    const auto cur_time = Clock::now();
    elapsed_sec = std::chrono::duration_cast<std::chrono::duration<double>>(cur_time - start_time).count();
  }

  // Drop existing cached occupancy values before continuing.
  GpuCache *gpu_cache = gpumap::gpuCache(map);
  if (gpu_cache)
  {
    GpuLayerCache *clearance_cache = gpu_cache->layerCache(kGcIdClearance);
    clearance_cache->syncToMainMemory();
    clearance_cache->clear();
  }

  unsigned total_processed = 0;
  const glm::i16vec3 step(1);
  while (d->haveWork() && (time_slice <= 0 || elapsed_sec < time_slice))
  {
    // Iterate dirty regions
    updateRegion(map, d->current_dirty_cursor, false);
    // updateExtendedRegion(map, d->mapStamp, d->currentDirtyCursor, d->currentDirtyCursor + step - glm::i16vec3(1));
    d->stepCursor(step);

    total_processed += volumeOf(step);

    if (!d->haveWork())
    {
      d->getWork(map);
    }

    const auto cur_time = Clock::now();
    elapsed_sec = std::chrono::duration_cast<std::chrono::duration<double>>(cur_time - start_time).count();
  }

  return (total_processed != 0 || d->haveWork()) ? kMprProgressing : kMprUpToDate;
}


void ohm::ClearanceProcess::calculateForExtents(OccupancyMap &map, const glm::dvec3 &min_extents,
                                                const glm::dvec3 &max_extents, bool force)
{
  // Ensure clearnce layer is present.
  ensureClearanceLayer(map);

  const glm::i16vec3 min_region = map.regionKey(min_extents);
  const glm::i16vec3 max_region = map.regionKey(max_extents);
  //// Process in blocks containing up to this many regions in each dimension.
  // const int blockMax = 5;

  glm::i16vec3 region_key;

  // Drop existing cached occupancy values before continuing.
  GpuCache *gpu_cache = gpumap::gpuCache(map);
  if (gpu_cache)
  {
    GpuLayerCache *clearance_cache = gpu_cache->layerCache(kGcIdClearance);
    clearance_cache->syncToMainMemory();
    clearance_cache->clear();
  }

  for (int z = min_region.z; z <= max_region.z; ++z)
  {
    region_key.z = z;
    for (int y = min_region.y; y <= max_region.y; ++y)
    {
      region_key.y = y;
      for (int x = min_region.x; x <= max_region.x; ++x)
      {
        region_key.x = x;
        updateRegion(map, region_key, force);
      }
    }
  }
}

bool ClearanceProcess::updateRegion(OccupancyMap &map, const glm::i16vec3 &region_key, bool force)
{
  // Get the next region.
  ClearanceProcessDetail *d = imp();

  MapChunk *region = map.region(region_key, (d->query_flags & kQfInstantiateUnknown));
  if (!region)
  {
    return false;
  }

  const int occupancy_layer_index = map.layout().occupancyLayer();
  const int clearance_layer_index = map.layout().clearanceLayer();

  if (occupancy_layer_index < 0 || clearance_layer_index < 0)
  {
    return false;
  }

  // Explore the region neighbours to see if they are out of date. That would invalidate this region.
  glm::i16vec3 neighbour_key;

  // We are dirty if any input region has updated occupancy values since the last
  // region->touchedStamps[clearance_layer_index] value. We iterate to work out the maximum
  // touchedStamps[occupancy_layer_index] value in the neighbourhood and compare that to our region
  // clearance_layer_index stamp. Dirty if clearance stamp is lower. The target value also sets the new stamp to apply
  // to the region clearance stamp.
  uint64_t target_update_stamp = region->touched_stamps[occupancy_layer_index];
  for (int z = -1; z <= 1; ++z)
  {
    neighbour_key.z = region_key.z + z;
    for (int y = -1; y <= 1; ++y)
    {
      neighbour_key.y = region_key.y + y;
      for (int x = -1; x <= 1; ++x)
      {
        neighbour_key.x = region_key.x + x;
        MapChunk *neighbour = map.region(neighbour_key, false);
        if (neighbour)
        {
          target_update_stamp =
            std::max(target_update_stamp, uint64_t(neighbour->touched_stamps[occupancy_layer_index]));
        }
      }
    }
  }

  if (!force && region->touched_stamps[clearance_layer_index] >= target_update_stamp)
  {
    // Nothing to update in these extents.
    return false;
  }

  // Debug highlight the region.
  TES_BOX_W(g_tes, TES_COLOUR(FireBrick), tes::Id(&map),
            tes::Transform(tes::Vector3d(glm::value_ptr(map.regionSpatialCentre(region_key))),
                           tes::Vector3d(glm::value_ptr(map.regionSpatialResolution()))));

  if ((d->query_flags & kQfGpuEvaluate))
  {
    PROFILE(occupancyClearanceProcessGpu);
    d->gpu_query->setAxisScaling(d->axis_scaling);
    d->gpu_query->setSearchRadius(d->search_radius);
    d->gpu_query->setQueryFlags(d->query_flags);
    d->gpu_query->calculateForRegion(map, region_key);
  }
  else
  {
    if (d->query_flags & kQfGpuEvaluate)
    {
      std::cerr << "ClearanceProcess requested GPU unavailable. Using CPU." << std::endl;
    }

    std::function<unsigned(OccupancyMap &, ClearanceProcessDetail &, const glm::i16vec3 &, ClosestResult &)> query_func;

#if 1
    query_func = [](OccupancyMap &map, ClearanceProcessDetail &query, const glm::i16vec3 &region_key, ClosestResult &
                    /* closest */) -> unsigned { return regionClearanceProcessCpu(map, query, region_key); };
    ClosestResult closest;  // Not used.
    ohm::occupancyQueryRegions(map, *d, closest, map.regionSpatialMin(region_key) - glm::dvec3(d->search_radius),
                               map.regionSpatialMin(region_key) + glm::dvec3(d->search_radius), query_func);
#else   // #
    // Flood fill experiment. Incorrect as it will propagate changes from the current iteration
    // to some neighbours. Protect against that it it can be much faster than the brute force method.
    queryFunc = [&voxel_extents, &calc_extents](
                  OccupancyMap &map, ClearanceProcessDetail &query, const glm::i16vec3 &region_key, ClosestResult &
                  /* closest */) -> unsigned {
      // return regionClearanceProcessCpu(map, query, region_key, voxel_extents, calc_extents);
      return regionSeedFloodFillCpu(map, query, region_key, voxel_extents, calc_extents);
    };

    ClosestResult closest;  // Not used.
    ohm::occupancyQueryRegions(map, *d, closest, minExtents - glm::dvec3(d->searchRadius),
                               maxExtents + glm::dvec3(d->searchRadius), queryFunc);

    queryFunc = [&voxel_extents, &calc_extents, map](
                  OccupancyMap &constMap, ClearanceProcessDetail &query, const glm::i16vec3 &region_key, ClosestResult &
                  /* closest */) -> unsigned {
      return regionFloodFillStepCpu(map, query, region_key, voxel_extents, calc_extents);
    };

    for (unsigned i = 0; i < voxelPadding; ++i)
    {
      ohm::occupancyQueryRegions(map, *d, closest, d->minExtents - glm::dvec3(d->searchRadius),
                                 d->maxExtents + glm::dvec3(d->searchRadius), queryFunc);
    }
#endif  // #
  }

  TES_SERVER_UPDATE(g_tes, 0.0f);
  // Delete debug objects.
  // TES_SPHERE_END(g_tes, uint32_t((size_t)&map));
  TES_BOX_END(g_tes, uint32_t((size_t)&map));

  // Regions are up to date *now*.
  region->touched_stamps[clearance_layer_index] = target_update_stamp;
  return true;
}


ClearanceProcessDetail *ClearanceProcess::imp()
{
  return static_cast<ClearanceProcessDetail *>(imp_);
}


const ClearanceProcessDetail *ClearanceProcess::imp() const
{
  return static_cast<const ClearanceProcessDetail *>(imp_);
}
