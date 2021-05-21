// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RaysQueryGpu.h"

#include "private/RaysQueryDetailGpu.h"

#include "GpuKey.h"
#include "GpuLayerCache.h"

#include <ohm/private/OccupancyMapDetail.h>

namespace ohm
{
RaysQueryGpu::RaysQueryGpu(RaysQueryDetailGpu *detail)
  : RaysQuery(detail)
{}


RaysQueryGpu::RaysQueryGpu()
  : RaysQuery(new RaysQueryDetailGpu)
{
  RaysQueryDetailGpu *d = imp();
  d->gpu_interface = std::make_unique<RaysQueryMapWrapper>();
  d->query_flags |= kQfGpu;
}


RaysQueryGpu::~RaysQueryGpu()
{}


void RaysQueryGpu::onSetMap()
{
  RaysQuery::onSetMap();
  RaysQueryDetailGpu *d = imp();
  d->gpu_interface->setMap(d->map);
}


bool RaysQueryGpu::onExecute()
{
  RaysQueryDetailGpu *d = imp();

  if (!d->map)
  {
    return false;
  }

  if (!onExecuteAsync())
  {
    return false;
  }

  // Sync.
  sync();

  return true;
}


bool RaysQueryGpu::onExecuteAsync()
{
  RaysQueryDetailGpu *d = imp();

  if (!d->map || d->rays_in.empty())
  {
    return false;
  }

  if (!(d->query_flags & kQfGpuEvaluate))
  {
    return RaysQuery::onExecute();
  }

  d->gpu_interface->setVolumeCoefficient(volumeCoefficient());

  return d->gpu_interface->integrateRays(d->rays_in.data(), d->rays_in.size()) == d->rays_in.size();
}


void RaysQueryGpu::onReset(bool hard_reset)
{
  RaysQuery::onReset(hard_reset);
  // Need to wait on the GPU program.
  sync();
}

void RaysQueryGpu::sync()
{
  RaysQueryDetailGpu *d = imp();
  if (d->query_flags & kQfGpuEvaluate)
  {
    RaysQueryDetailGpu *d = imp();
    d->gpu_interface->syncVoxels();

    d->ranges.clear();
    d->unobserved_volumes_out.clear();
    d->terminal_states_out.clear();

    const auto number_of_results = d->gpu_interface->results().size();

    d->ranges.reserve(number_of_results);
    d->unobserved_volumes_out.reserve(number_of_results);
    d->terminal_states_out.reserve(number_of_results);

    // Copy to split output buffers.
    for (const auto &result : d->gpu_interface->results())
    {
      d->ranges.emplace_back(result.range);
      d->unobserved_volumes_out.emplace_back(result.unobserved_volume);
      d->terminal_states_out.emplace_back(OccupancyType(result.voxel_type));
    }
    d->number_of_results = number_of_results;
  }
}


RaysQueryDetailGpu *RaysQueryGpu::imp()
{
  return static_cast<RaysQueryDetailGpu *>(imp_);
}


const RaysQueryDetailGpu *RaysQueryGpu::imp() const
{
  return static_cast<const RaysQueryDetailGpu *>(imp_);
}
}  // namespace ohm
