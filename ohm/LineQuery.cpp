// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "LineQuery.h"

#include "MapCache.h"
#include "GpuMap.h"
#include "Key.h"
#include "OccupancyMap.h"
#include "QueryFlag.h"
#include "ClearanceProcess.h"
#include "private/LineQueryDetail.h"
#include "private/OccupancyMapDetail.h"
#include "private/VoxelAlgorithms.h"
#include "private/OccupancyQueryAlg.h"

#include "GpuLayerCache.h"

#include <3esservermacros.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#ifdef OHM_THREADS
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif  // OHM_THREADS

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>

using namespace ohm;

namespace
{
  void calculateNearestNeighboursRange(LineQueryDetail &query, size_t start_index, size_t end_index,
                                       const OccupancyMap &map, const glm::ivec3 &voxel_search_half_extents)
  {
    float range;

    for (size_t i = start_index; i < end_index; ++i)
    {
      const Key &key = query.segment_keys[i];
      range =
        calculateNearestNeighbour(key, map, voxel_search_half_extents, (query.query_flags & kQfUnknownAsOccupied) != 0,
                                  false, query.search_radius, query.axis_scaling);
      // if (range < 0)
      // {
      //   range = query.default_range;
      // }
      query.intersected_voxels[i] = key;
      query.ranges[i] = range;
    }
  }

  unsigned occupancyLineQueryCpu(const OccupancyMap &map, LineQueryDetail &query, ClosestResult &closest)
  {
    glm::ivec3 voxel_search_half_extents = calculateVoxelSearchHalfExtents(map, query.search_radius);
    map.calculateSegmentKeys(query.segment_keys, query.start_point, query.end_point);

    // Allocate results.
    query.intersected_voxels.resize(query.segment_keys.size());
    query.ranges.resize(query.segment_keys.size());

    // Perform query.
#ifdef OHM_THREADS
    const auto parallel_query_func = [&query, &map, voxel_search_half_extents](const tbb::blocked_range<size_t> &range) {
      calculateNearestNeighboursRange(query, range.begin(), range.end(), map, voxel_search_half_extents);
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0u, query.segment_keys.size()), parallel_query_func);

#else   // OHM_THREADS
    calculateNearestNeighboursRange(query, 0u, query.segment_keys.size(), map, voxel_search_half_extents);
#endif  // OHM_THREADS

    // Find closest result.
    for (size_t i = 0; i < query.ranges.size(); ++i)
    {
      float range = query.ranges[i];
      if (range * range < closest.range)
      {
        closest.range = range * range;
        closest.index = i;
      }
    }

    return unsigned(query.segment_keys.size());
  }
}


LineQuery::LineQuery(LineQueryDetail *detail)
  : Query(detail)
{}


LineQuery::LineQuery()
  : LineQuery(new LineQueryDetail)
{}


LineQuery::LineQuery(OccupancyMap &map, const glm::dvec3 &start_point, const glm::dvec3 &end_point, float search_radius,
                     unsigned query_flags)
  : LineQuery(new LineQueryDetail)
{
  setMap(&map);
  setStartPoint(start_point);
  setEndPoint(end_point);
  setSearchRadius(search_radius);
  setQueryFlags(query_flags);
}


LineQuery::~LineQuery()
{
  LineQueryDetail *d = imp();
  if (d)
  {
    delete d->clearance_calculator;
    delete d;
  }
  // Clear pointer for base class.
  imp_ = nullptr;
}


glm::dvec3 LineQuery::startPoint() const
{
  const LineQueryDetail *d = imp();
  return d->start_point;
}


void LineQuery::setStartPoint(const glm::dvec3 &point)
{
  LineQueryDetail *d = imp();
  d->start_point = point;
}


glm::dvec3 LineQuery::endPoint() const
{
  const LineQueryDetail *d = imp();
  return d->end_point;
}


void LineQuery::setEndPoint(const glm::dvec3 &point)
{
  LineQueryDetail *d = imp();
  d->end_point = point;
}


float LineQuery::searchRadius() const
{
  const LineQueryDetail *d = imp();
  return d->search_radius;
}


void LineQuery::setSearchRadius(float radius)
{
  LineQueryDetail *d = imp();
  d->search_radius = radius;
}


float LineQuery::defaultRangeValue() const
{
  const LineQueryDetail *d = imp();
  return d->default_range;
}


void LineQuery::setDefaultRangeValue(float range)
{
  LineQueryDetail *d = imp();
  d->default_range = range;
}


glm::vec3 LineQuery::axisScaling() const
{
  const LineQueryDetail *d = imp();
  return d->axis_scaling;
}


void LineQuery::setAxisScaling(const glm::vec3 &scaling)
{
  LineQueryDetail *d = imp();
  d->axis_scaling = scaling;
}


bool LineQuery::onExecute()
{
  LineQueryDetail *d = imp();

  if (!d->map)
  {
    return false;
  }

  bool used_cache_clearance = !(d->query_flags & kQfNoCache);
  ClosestResult closest;
  if (d->query_flags & kQfGpuEvaluate)
  {
    // GPU evaluation requested. Use the ClearanceProcess to do so.
    glm::dvec3 min_ext, max_ext;
    for (int i = 0; i < 3; ++i)
    {
      min_ext[i] = std::min(d->start_point[i], d->end_point[i]);
      max_ext[i] = std::max(d->start_point[i], d->end_point[i]);
    }
    unsigned clearance_flags = kQfGpuEvaluate;
    if (d->query_flags & kQfUnknownAsOccupied)
    {
      clearance_flags |= kQfUnknownAsOccupied;
    }
    if (!d->clearance_calculator)
    {
      d->clearance_calculator = new ClearanceProcess();
    }
    d->clearance_calculator->setSearchRadius(d->search_radius);
    d->clearance_calculator->setQueryFlags(clearance_flags);
    d->clearance_calculator->setAxisScaling(d->axis_scaling);
    // Force recalculation if not using cached values. Otherwise we'll only calculate dirty regions.
    const bool force = (d->query_flags & kQfNoCache);
    d->clearance_calculator->calculateForExtents(*d->map, min_ext, max_ext, force);
    // QF_NoCache behaviour differs slightly for GPU calculation. The GPU only pushes
    // into the voxel clearance layer so set usedCacheClearance to read that information.
    used_cache_clearance = true;
  }

  if (used_cache_clearance)
  {
    // Calculate the voxels the line intersects.
    d->map->calculateSegmentKeys(d->segment_keys, d->start_point, d->end_point);

    // Populate results.
    d->intersected_voxels.resize(d->segment_keys.size());
    d->ranges.resize(d->segment_keys.size());
    VoxelConst voxel;

    if (!d->segment_keys.empty())
    {
      float range;
      closest.index = 0;
      closest.range = -1;
      for (size_t i = 0; i < d->segment_keys.size(); ++i)
      {
        d->intersected_voxels[i] = d->segment_keys[i];
        voxel = d->map->voxel(d->segment_keys[i]);

        if (voxel.isValid())
        {
          range = voxel.clearance((d->query_flags & kQfUnknownAsOccupied) != 0);
          // Range will be -1 from ClearanceProcess for unobstructed voxels (to the search radius).
          // Override the result with d->default_range, which defaults to -1 as well.
          if (range < 0)
          {
            range = d->default_range;
          }
        }
        else
        {
          range = d->default_range;
        }

        d->ranges[i] = range;
        if (i == 0 || range >= 0 && (range < closest.range || closest.range < 0))
        {
          closest.index = i;
          closest.range = range;
        }
      }
    }
  }
  else
  {
    occupancyLineQueryCpu(*d->map, *d, closest);
  }

  if ((d->query_flags & kQfNearestResult) && !d->intersected_voxels.empty())
  {
    d->intersected_voxels[0] = d->intersected_voxels[closest.index];
    d->intersected_voxels.resize(1);
    d->ranges[0] = d->ranges[closest.index];
    d->number_of_results = 1u;
    d->ranges.resize(1);
  }
  else
  {
    d->number_of_results = d->intersected_voxels.size();
  }

  return true;
}


bool LineQuery::onExecuteAsync()
{
  return false;
}


void LineQuery::onReset(bool /*hard_reset*/)
{
  //LineQueryDetail *d = imp();
}


LineQueryDetail *LineQuery::imp()
{
  return static_cast<LineQueryDetail *>(imp_);
}


const LineQueryDetail *LineQuery::imp() const
{
  return static_cast<const LineQueryDetail *>(imp_);
}
