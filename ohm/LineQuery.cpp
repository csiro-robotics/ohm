// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "LineQuery.h"

#include "Key.h"
#include "MapCache.h"
#include "OccupancyMap.h"
#include "QueryFlag.h"
#include "private/LineQueryDetail.h"
#include "private/OccupancyMapDetail.h"
#include "private/OccupancyQueryAlg.h"
#include "private/VoxelAlgorithms.h"

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
  void calculateNearestNeighboursRange(LineQueryDetail &query,  // NOLINT(google-runtime-references)
                                       size_t start_index, size_t end_index, const OccupancyMap &map,
                                       const glm::ivec3 &voxel_search_half_extents)
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

  unsigned occupancyLineQueryCpu(const OccupancyMap &map, LineQueryDetail &query,  // NOLINT(google-runtime-references)
                                 ClosestResult &closest)                           // NOLINT(google-runtime-references)
  {
    glm::ivec3 voxel_search_half_extents = calculateVoxelSearchHalfExtents(map, query.search_radius);
    map.calculateSegmentKeys(query.segment_keys, query.start_point, query.end_point);

    // Allocate results.
    query.intersected_voxels.resize(query.segment_keys.size());
    query.ranges.resize(query.segment_keys.size());

    // Perform query.
#ifdef OHM_THREADS
    const auto parallel_query_func = [&query, &map,
                                      voxel_search_half_extents](const tbb::blocked_range<size_t> &range) {
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
}  // namespace


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
  delete d;
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

  ClosestResult closest;
  occupancyLineQueryCpu(*d->map, *d, closest);

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
  // LineQueryDetail *d = imp();
}


LineQueryDetail *LineQuery::imp()
{
  return static_cast<LineQueryDetail *>(imp_);
}


const LineQueryDetail *LineQuery::imp() const
{
  return static_cast<const LineQueryDetail *>(imp_);
}
