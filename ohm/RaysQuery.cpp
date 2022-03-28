// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RaysQuery.h"

#include "private/RaysQueryDetail.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "LineWalk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"
#include "VoxelBuffer.h"
#include "VoxelMean.h"
#include "VoxelOccupancy.h"

namespace ohm
{
RaysQuery::RaysQuery(RaysQueryDetail *detail)
  : Query(detail)
{}


RaysQuery::RaysQuery()
  : RaysQuery(new RaysQueryDetail)
{}


RaysQuery::~RaysQuery() = default;


void RaysQuery::setVolumeCoefficient(double coefficient)
{
  RaysQueryDetail *d = imp();
  d->volume_coefficient = coefficient;
}


double RaysQuery::volumeCoefficient() const
{
  const RaysQueryDetail *d = imp();
  return d->volume_coefficient;
}


void RaysQuery::setRays(const glm::dvec3 *rays, size_t element_count)
{
  RaysQueryDetail *d = imp();
  d->rays_in.clear();
  addRays(rays, element_count);
}


void RaysQuery::addRays(const glm::dvec3 *rays, size_t element_count)
{
  RaysQueryDetail *d = imp();
  // Ensure we add in pairs.
  for (size_t i = 0; i < element_count; i += 2)
  {
    d->rays_in.emplace_back(rays[i]);
    d->rays_in.emplace_back(rays[i + 1]);
  }
}


void RaysQuery::addRay(const glm::dvec3 &origin, const glm::dvec3 &end_point)
{
  RaysQueryDetail *d = imp();
  d->rays_in.emplace_back(origin);
  d->rays_in.emplace_back(end_point);
}

void RaysQuery::clearRays()
{
  RaysQueryDetail *d = imp();
  d->rays_in.clear();
}


const glm::dvec3 *RaysQuery::rays(size_t *count) const
{
  const RaysQueryDetail *d = imp();
  if (count)
  {
    *count = d->rays_in.size();
  }
  return d->rays_in.data();
}


size_t RaysQuery::numberOfRays() const
{
  const RaysQueryDetail *d = imp();
  return d->rays_in.size() / 2;
}


bool RaysQuery::onExecute()
{
  RaysQueryDetail *d = imp();

  if (!d->map && !d->valid_layers)
  {
    return false;
  }

  KeyList keys;
  MapChunk *last_chunk = nullptr;
  VoxelBuffer<const VoxelBlock> occupancy_buffer;
  double unobserved_volume = 0;
  float range = 0;
  OccupancyType terminal_state = OccupancyType::kNull;
  Key terminal_key(nullptr);

  auto map = d->map;
  const RayFilterFunction ray_filter = map->rayFilter();
  const bool use_filter = bool(ray_filter);
  const auto occupancy_layer = d->occupancy_layer;
  const auto occupancy_dim = d->occupancy_dim;
  const auto occupancy_threshold_value = map->occupancyThresholdValue();
  const auto volume_coefficient = d->volume_coefficient;

  const auto visit_func = [&](const Key &key, double enter_range, double exit_range) -> bool  //
  {
    // Work out the index of the voxel in it's region.
    const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
    float occupancy_value = unobservedOccupancyValue();
    // Ensure the MapChunk pointer is up to date.
    MapChunk *chunk =
      (last_chunk && key.regionKey() == last_chunk->region.coord) ? last_chunk : map->region(key.regionKey(), false);
    if (chunk)
    {
      if (chunk != last_chunk)
      {
        occupancy_buffer = VoxelBuffer<const VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
      }
      occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
    }
    last_chunk = chunk;
    // Check voxel occupancy status.
    const bool is_unobserved = occupancy_value == unobservedOccupancyValue();
    const bool is_occupied = !is_unobserved && occupancy_value > occupancy_threshold_value;
    unobserved_volume +=
      is_unobserved ?
        (volume_coefficient * (exit_range * exit_range * exit_range - enter_range * enter_range * enter_range)) :
        0.0f;
      range = (!is_occupied) ? float(exit_range) : range;
    // Resolve the voxel state.
    terminal_state =
      is_unobserved ? OccupancyType::kUnobserved : (is_occupied ? OccupancyType::kOccupied : OccupancyType::kFree);
    terminal_key = key;

    return !is_occupied;
  };

  // Size the output arrays.
  d->ranges.reserve(d->rays_in.size() / 2);
  d->intersected_voxels.reserve(d->rays_in.size() / 2);
  d->unobserved_volumes_out.reserve(d->rays_in.size() / 2);
  d->terminal_states_out.reserve(d->rays_in.size() / 2);

  glm::dvec3 start;
  glm::dvec3 end;
  unsigned filter_flags;
  for (size_t i = 0; i < d->rays_in.size(); i += 2)
  {
    filter_flags = 0;
    start = d->rays_in[i];
    end = d->rays_in[i + 1];

    unobserved_volume = 0.0f;
    range = 0.0f;

    if (use_filter && !ray_filter(&start, &end, &filter_flags))
    {
      // Filtered ray.
      d->ranges.emplace_back(range);
      d->unobserved_volumes_out.emplace_back(unobserved_volume);
      d->terminal_states_out.emplace_back(OccupancyType::kNull);
      d->intersected_voxels.emplace_back(Key::kNull);
      continue;
    }

    walkSegmentKeys(LineWalkContext(*map, visit_func), start, end);

    d->ranges.emplace_back(range);
    d->unobserved_volumes_out.emplace_back(unobserved_volume);
    d->terminal_states_out.emplace_back(terminal_state);
    d->intersected_voxels.emplace_back(terminal_key);
  }

  d->number_of_results = d->ranges.size();

  return true;
}


const double *RaysQuery::unobservedVolumes() const
{
  const RaysQueryDetail *d = imp();
  return d->unobserved_volumes_out.data();
}


const OccupancyType *RaysQuery::terminalOccupancyTypes() const
{
  const RaysQueryDetail *d = imp();
  return d->terminal_states_out.data();
}


void RaysQuery::onSetMap()
{
  RaysQueryDetail *d = imp();
  auto map = d->map;
  if (!map)
  {
    d->occupancy_layer = -1;
    d->valid_layers = false;
    return;
  }

  d->occupancy_layer = map->layout().occupancyLayer();

  // Use Voxel to validate the layers.
  // In processing we use VoxelBuffer instead of Voxel objects. While Voxel makes for a neater API, using VoxelBuffer
  // makes for less overhead and yields better performance.
  Voxel<const float> occupancy(map, d->occupancy_layer);

  d->occupancy_dim = occupancy.isLayerValid() ? occupancy.layerDim() : d->occupancy_dim;

  // Validate we only have an occupancy layer.
  d->valid_layers = occupancy.isLayerValid();
}


bool RaysQuery::onExecuteAsync()
{
  return false;
}


void RaysQuery::onReset(bool hard_reset)
{
  RaysQueryDetail *d = imp();
  d->unobserved_volumes_out.clear();
  d->terminal_states_out.clear();
  if (hard_reset)
  {
    d->rays_in.clear();
  }
}


RaysQueryDetail *RaysQuery::imp()
{
  return static_cast<RaysQueryDetail *>(imp_);
}


const RaysQueryDetail *RaysQuery::imp() const
{
  return static_cast<const RaysQueryDetail *>(imp_);
}
}  // namespace ohm
