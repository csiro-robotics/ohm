// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancylinequery.h"

#include "mapcache.h"
#include "mapregion.h"
#include "occupancygpumap.h"
#include "occupancykey.h"
#include "occupancymap.h"
#include "occupancyqueryflag.h"
#include "ohmclearanceprocess.h"
#include "ohmdefaultlayers.h"
#include "private/occupancylinequerydetail.h"
#include "private/occupancymapdetail.h"
#include "private/occupancynodealgorithms.h"
#include "private/occupancyqueryalg.h"

#include "gpucache.h"
#include "gpulayercache.h"

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
  void calculateNearestNeighboursRange(LineQueryDetail &query, size_t startIndex, size_t endIndex,
                                       const OccupancyMap &map, const glm::ivec3 &voxelSearchHalfExtents)
  {
    const OccupancyMapDetail *data = map.detail();
    float range;

    for (size_t i = startIndex; i < endIndex; ++i)
    {
      const OccupancyKey &key = query.segmentKeys[i];
      range =
        calculateNearestNeighbour(key, map, voxelSearchHalfExtents, (query.queryFlags & QF_UnknownAsOccupied) != 0,
                                  false, query.searchRadius, query.axisScaling);
      // if (range < 0)
      // {
      //   range = query.defaultRange;
      // }
      query.intersectedVoxels[i] = key;
      query.ranges[i] = range;
    }
  }

  unsigned occupancyLineQueryCpu(const OccupancyMap &map, LineQueryDetail &query, ClosestResult &closest)
  {
    const OccupancyMapDetail *data = map.detail();
    const OccupancyKey startKey = map.voxelKey(query.startPoint);
    const OccupancyKey endKey = map.voxelKey(query.endPoint);
    glm::dvec3 lineMinExtents, lineMaxExtents, extentsAdjustment;
    glm::i16vec3 regionMin, regionMax, regionKey;

    glm::ivec3 voxelSearchHalfExtents = calculateVoxelSearchHalfExtents(map, query.searchRadius);
    map.calculateSegmentKeys(query.segmentKeys, query.startPoint, query.endPoint);

    // Allocate results.
    query.intersectedVoxels.resize(query.segmentKeys.size());
    query.ranges.resize(query.segmentKeys.size());

    // Perform query.
#ifdef OHM_THREADS
    auto parallelQueryFunc = [&query, &map, voxelSearchHalfExtents](const tbb::blocked_range<size_t> &range) {
      calculateNearestNeighboursRange(query, range.begin(), range.end(), map, voxelSearchHalfExtents);
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0u, query.segmentKeys.size()), parallelQueryFunc);

#else   // OHM_THREADS
    calculateNearestNeighboursRange(query, 0u, query.segmentKeys.size(), map, voxelSearchHalfExtents);
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

    return (unsigned)query.segmentKeys.size();
  }
}


LineQuery::LineQuery(LineQueryDetail *detail)
  : Query(detail)
{}


LineQuery::LineQuery()
  : LineQuery(new LineQueryDetail)
{}


LineQuery::LineQuery(OccupancyMap &map, const glm::dvec3 &startPoint, const glm::dvec3 &endPoint, float searchRadius,
                     unsigned queryFlags)
  : LineQuery(new LineQueryDetail)
{
  setMap(&map);
  setStartPoint(startPoint);
  setEndPoint(endPoint);
  setSearchRadius(searchRadius);
  setQueryFlags(queryFlags);
  LineQueryDetail *d = imp();
}


LineQuery::~LineQuery()
{
  LineQueryDetail *d = imp();
  if (d)
  {
    delete d->clearanceCalculator;
    delete d;
  }
  // Clear pointer for base class.
  _imp = nullptr;
}


glm::dvec3 LineQuery::startPoint() const
{
  const LineQueryDetail *d = imp();
  return d->startPoint;
}


void LineQuery::setStartPoint(const glm::dvec3 &point)
{
  LineQueryDetail *d = imp();
  d->startPoint = point;
}


glm::dvec3 LineQuery::endPoint() const
{
  const LineQueryDetail *d = imp();
  return d->endPoint;
}


void LineQuery::setEndPoint(const glm::dvec3 &point)
{
  LineQueryDetail *d = imp();
  d->endPoint = point;
}


float LineQuery::searchRadius() const
{
  const LineQueryDetail *d = imp();
  return d->searchRadius;
}


void LineQuery::setSearchRadius(float radius)
{
  LineQueryDetail *d = imp();
  d->searchRadius = radius;
}


float LineQuery::defaultRangeValue() const
{
  const LineQueryDetail *d = imp();
  return d->defaultRange;
}


void LineQuery::setDefaultRangeValue(float range)
{
  LineQueryDetail *d = imp();
  d->defaultRange = range;
}


bool LineQuery::onExecute()
{
  LineQueryDetail *d = imp();

  if (!d->map)
  {
    return false;
  }

  bool usedCacheClearance = !(d->queryFlags & QF_NoCache);
  ClosestResult closest;
  if (d->queryFlags & QF_GpuEvaluate)
  {
    // GPU evaluation requested. Use the ClearanceProcess to do so.
    glm::dvec3 minExt, maxExt;
    for (int i = 0; i < 3; ++i)
    {
      minExt[i] = std::min(d->startPoint[i], d->endPoint[i]);
      maxExt[i] = std::max(d->startPoint[i], d->endPoint[i]);
    }
    unsigned clearanceFlags = QF_GpuEvaluate;
    if (d->queryFlags & QF_UnknownAsOccupied)
    {
      clearanceFlags |= QF_UnknownAsOccupied;
    }
    if (!d->clearanceCalculator)
    {
      d->clearanceCalculator = new ClearanceProcess();
    }
    d->clearanceCalculator->setSearchRadius(d->searchRadius);
    d->clearanceCalculator->setQueryFlags(clearanceFlags);
    // Force recalculation if not using cached values. Otherwise we'll only calculate dirty regions.
    const bool force = (d->queryFlags & QF_NoCache);
    d->clearanceCalculator->calculateForExtents(*d->map, minExt, maxExt, force);
    // QF_NoCache behaviour differs slightly for GPU calculation. The GPU only pushes
    // into the voxel clearance layer so set usedCacheClearance to read that information.
    usedCacheClearance = true;
  }

  if (usedCacheClearance)
  {
    // Calculate the voxels the line intersects.
    d->map->calculateSegmentKeys(d->segmentKeys, d->startPoint, d->endPoint);

    // Populate results.
    d->intersectedVoxels.resize(d->segmentKeys.size());
    d->ranges.resize(d->segmentKeys.size());
    OccupancyNodeConst node;

    if (!d->segmentKeys.empty())
    {
      float range;
      closest.index = 0;
      closest.range = -1;
      for (size_t i = 0; i < d->segmentKeys.size(); ++i)
      {
        d->intersectedVoxels[i] = d->segmentKeys[i];
        node = d->map->node(d->segmentKeys[i]);

        if (node.isValid())
        {
          range = node.clearance((d->queryFlags & QF_UnknownAsOccupied) != 0);
          // Range will be -1 from ClearanceProcess for unobstructed voxels (to the search radius).
          // Override the result with d->defaultRange, which defaults to -1 as well.
          if (range < 0)
          {
            range = d->defaultRange;
          }
        }
        else
        {
          range = d->defaultRange;
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

  if ((d->queryFlags & QF_NearestResult) && !d->intersectedVoxels.empty())
  {
    d->intersectedVoxels[0] = d->intersectedVoxels[closest.index];
    d->intersectedVoxels.resize(1);
    d->ranges[0] = d->ranges[closest.index];
    d->numberOfResults = 1u;
    d->ranges.resize(1);
  }
  else
  {
    d->numberOfResults = d->intersectedVoxels.size();
  }

  return true;
}


bool LineQuery::onExecuteAsync()
{
  return false;
}


void LineQuery::onReset(bool hardReset)
{
  LineQueryDetail *d = imp();
}


LineQueryDetail *LineQuery::imp()
{
  return static_cast<LineQueryDetail *>(_imp);
}


const LineQueryDetail *LineQuery::imp() const
{
  return static_cast<const LineQueryDetail *>(_imp);
}
