// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "LineQueryGpu.h"

#include "ClearanceProcess.h"
#include "GpuMap.h"

#include "private/LineQueryDetailGpu.h"

#include <ohm/Key.h>
#include <ohm/OccupancyMap.h>
#include <ohm/QueryFlag.h>
#include <ohm/VoxelData.h>
#include <ohm/private/OccupancyMapDetail.h>
#include <ohm/private/OccupancyQueryAlg.h>
#include <ohm/private/VoxelAlgorithms.h>

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


LineQueryGpu::LineQueryGpu(LineQueryDetailGpu *detail)
  : LineQuery(detail)
{}


LineQueryGpu::LineQueryGpu()
  : LineQuery(new LineQueryDetailGpu)
{}


LineQueryGpu::LineQueryGpu(OccupancyMap &map, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                           float search_radius, unsigned query_flags)
  : LineQuery(new LineQueryDetailGpu)
{
  setMap(&map);
  setStartPoint(start_point);
  setEndPoint(end_point);
  setSearchRadius(search_radius);
  setQueryFlags(query_flags);
}


LineQueryGpu::~LineQueryGpu()
{
  LineQueryDetailGpu *d = imp();
  if (d)
  {
    delete d->clearance_calculator;
    delete d;
  }
  // Clear pointer for base class.
  imp_ = nullptr;
}


bool LineQueryGpu::onExecute()
{
  LineQueryDetailGpu *d = imp();

  if (!d->map)
  {
    return false;
  }

  if (!(d->query_flags & kQfGpuEvaluate))
  {
    return LineQuery::onExecute();
  }

  ClosestResult closest;

  ClearanceProcess::ensureClearanceLayer(*d->map);
  gpumap::enableGpu(*d->map);

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

  // Calculate the voxels the line intersects.
  d->map->calculateSegmentKeys(d->segment_keys, d->start_point, d->end_point);

  // Populate results.
  d->intersected_voxels.resize(d->segment_keys.size());
  d->ranges.resize(d->segment_keys.size());

  Voxel<const float> clearance(d->map, d->map->layout().clearanceLayer());

  if (!d->segment_keys.empty())
  {
    float range;
    closest.index = 0;
    closest.range = -1;
    for (size_t i = 0; i < d->segment_keys.size(); ++i)
    {
      d->intersected_voxels[i] = d->segment_keys[i];
      clearance.setKey(d->segment_keys[i]);

      if (clearance.isValid())
      {
        clearance.read(&range);
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


bool LineQueryGpu::onExecuteAsync()
{
  return false;
}


void LineQueryGpu::onReset(bool /*hard_reset*/)
{
  // LineQueryDetailGpu *d = imp();
}


LineQueryDetailGpu *LineQueryGpu::imp()
{
  return static_cast<LineQueryDetailGpu *>(imp_);
}


const LineQueryDetailGpu *LineQueryGpu::imp() const
{
  return static_cast<const LineQueryDetailGpu *>(imp_);
}
