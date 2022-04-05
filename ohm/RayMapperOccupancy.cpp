// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RayMapperOccupancy.h"

#include "DefaultLayer.h"
#include "LineWalk.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"
#include "VoxelBuffer.h"
#include "VoxelIncident.h"
#include "VoxelMean.h"
#include "VoxelOccupancy.h"
#include "VoxelTouchTime.h"

// TODO (KS): RayMapperOccupancy::lookupRays() is deprecated. Use RaysQuery for less code maintenance, but it creates
// a poor dependency.
#include "RaysQuery.h"

namespace ohm
{
RayMapperOccupancy::RayMapperOccupancy(OccupancyMap *map)
  : map_(map)
  , occupancy_layer_(map_->layout().occupancyLayer())
  , mean_layer_(map_->layout().meanLayer())
  , traversal_layer_(map_->layout().traversalLayer())
  , touch_time_layer_(map_->layout().layerIndex(default_layer::touchTimeLayerName()))
  , incident_normal_layer_(map_->layout().layerIndex(default_layer::incidentNormalLayerName()))
{
  // Use Voxel to validate the layers.
  // In processing we use VoxelBuffer instead of Voxel objects. While Voxel makes for a neater API, using VoxelBuffer
  // makes for less overhead and yields better performance.
  Voxel<const float> occupancy(map_, occupancy_layer_);
  Voxel<const VoxelMean> mean(map_, mean_layer_);
  Voxel<const float> traversal(map_, traversal_layer_);
  Voxel<const uint32_t> touch_time_layer(map_, touch_time_layer_);
  Voxel<const uint32_t> incident_normal_layer(map_, incident_normal_layer_);

  occupancy_dim_ = occupancy.isLayerValid() ? occupancy.layerDim() : occupancy_dim_;

  // Validate we only have an occupancy layer or we also have a mean layer and the layer dimesions match.
  valid_ = occupancy.isLayerValid() && !mean.isLayerValid() ||
           occupancy.isLayerValid() && mean.isLayerValid() && occupancy.layerDim() == mean.layerDim();
  // Validate the traversal layer in a simliar fashion.
  valid_ = occupancy.isLayerValid() && !traversal.isLayerValid() ||
           occupancy.isLayerValid() && traversal.isLayerValid() && occupancy.layerDim() == traversal.layerDim();

  if (touch_time_layer.isLayerValid())
  {
    valid_ = valid_ && occupancy.layerDim() == touch_time_layer.layerDim();
  }

  if (incident_normal_layer.isLayerValid())
  {
    valid_ = valid_ && occupancy.layerDim() == incident_normal_layer.layerDim();
  }
}


RayMapperOccupancy::~RayMapperOccupancy() = default;


size_t RayMapperOccupancy::integrateRays(const glm::dvec3 *rays, size_t element_count, const float * /*intensities*/,
                                         const double *timestamps, unsigned ray_update_flags)
{
  KeyList keys;
  MapChunk *last_chunk = nullptr;
  MapChunk *last_mean_chunk = nullptr;
  VoxelBuffer<VoxelBlock> occupancy_buffer;
  VoxelBuffer<VoxelBlock> mean_buffer;
  VoxelBuffer<VoxelBlock> traversal_buffer;
  VoxelBuffer<VoxelBlock> touch_time_buffer;
  VoxelBuffer<VoxelBlock> incidents_buffer;
  double last_exit_range = 0;
  bool stop_adjustments = false;

  const RayFilterFunction ray_filter = map_->rayFilter();
  const bool use_filter = bool(ray_filter);
  const auto occupancy_layer = occupancy_layer_;
  const auto mean_layer = mean_layer_;
  const auto traversal_layer = traversal_layer_;
  const auto occupancy_dim = occupancy_dim_;
  const auto occupancy_threshold_value = map_->occupancyThresholdValue();
  const auto miss_value = map_->missValue();
  const auto hit_value = map_->hitValue();
  const auto resolution = map_->resolution();
  const auto voxel_min = map_->minVoxelValue();
  const auto voxel_max = map_->maxVoxelValue();
  const auto saturation_min = map_->saturateAtMinValue() ? voxel_min : std::numeric_limits<float>::lowest();
  const auto saturation_max = map_->saturateAtMaxValue() ? voxel_max : std::numeric_limits<float>::max();
  // Touch the map to flag changes.
  const auto touch_stamp = map_->touch();

  if (timestamps)
  {
    // Update first ray time if not yet set.
    map_->updateFirstRayTime(*timestamps);
  }

  const auto visit_func = [&](const Key &key, double enter_range, double exit_range) -> bool  //
  {
    // The update logic here is a little unclear as it tries to avoid outright branches.
    // The intended logic is described as follows:
    // 1. Select direct write or additive adjustment.
    //    - Make a direct, non-additive adjustment if one of the following conditions are met:
    //      - stop_adjustments is true
    //      - the voxel is uncertain
    //      - ray_update_flags and kRfExclude<Type> flags pass.
    //      - voxel is saturated
    //    - Otherwise add to present value.
    // 2. Select the value adjustment
    //    - current_value if one of the following conditions are met:
    //      - stop_adjustments is true (no longer making adjustments)
    //      - ray_update_flags and kRfExclude<Type> flags pass.
    //    - miss_value otherwise
    // 3. Calculate new value
    // 4. Apply saturation logic: only min saturation relevant
    //    -
    MapChunk *chunk =
      (last_chunk && key.regionKey() == last_chunk->region.coord) ? last_chunk : map_->region(key.regionKey(), true);
    if (chunk != last_chunk)
    {
      occupancy_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
      if (traversal_layer >= 0)
      {
        traversal_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[traversal_layer]);
      }
      if (touch_time_layer_ >= 0 && timestamps)
      {
        // Touch time not required for miss update, but we need it in sync for the update later.
        touch_time_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[touch_time_layer_]);
      }
      if (incident_normal_layer_ >= 0)
      {
        // Incidents not required for miss update, but we need it in sync for the update later.
        incidents_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[incident_normal_layer_]);
      }
    }
    last_chunk = chunk;
    const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
    float occupancy_value;
    occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
    const float initial_value = occupancy_value;

    const bool initially_unobserved = initial_value == unobservedOccupancyValue();
    const bool initially_free = !initially_unobserved && initial_value < occupancy_threshold_value;
    const bool initially_occupied = !initially_unobserved && initial_value >= occupancy_threshold_value;

    // Calculate the adjustment to make based on the initial occupancy value, various exclusion flags and the configured
    // value adjustment.
    float miss_adjustment = miss_value;
    // The next series of statements are designed to modify the miss_adjustment according to the current voxel state
    // and the kRfExclude<Type> values. Note that for kRfExcludeUnobserved we set the miss_adjustment such that it keeps
    // the observesed value, whereas in other cases we set to zero to make for no change. This is because unobserved
    // values have a value written, whereas other voxels have use addition to adjust the value.
    miss_adjustment = (initially_unobserved && (ray_update_flags & kRfExcludeUnobserved)) ? unobservedOccupancyValue() :
                                                                                            miss_adjustment;
    miss_adjustment = (initially_free && (ray_update_flags & kRfExcludeFree)) ? 0.0f : miss_adjustment;
    miss_adjustment = (initially_occupied && (ray_update_flags & kRfExcludeOccupied)) ? 0.0f : miss_adjustment;

    occupancyAdjustMiss(&occupancy_value, initial_value, miss_adjustment, unobservedOccupancyValue(), voxel_min,
                        saturation_min, saturation_max, stop_adjustments);
    occupancy_buffer.writeVoxel(voxel_index, occupancy_value);

    // Accumulate traversal
    if (traversal_layer >= 0)
    {
      float traversal;
      traversal_buffer.readVoxel(voxel_index, &traversal);
      traversal += float(exit_range - enter_range);
      traversal_buffer.writeVoxel(voxel_index, traversal);
    }

    // Lint(KS): The analyser takes some branches which are not possible in practice.
    // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
    chunk->updateFirstValid(voxel_index);

    stop_adjustments = stop_adjustments || ((ray_update_flags & kRfStopOnFirstOccupied) && initially_occupied);
    chunk->dirty_stamp = touch_stamp;
    // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
    // not so much the sequencing. We really don't want to synchronise here.
    chunk->touched_stamps[occupancy_layer].store(touch_stamp, std::memory_order_relaxed);

    // Store last exit range for final traversal accumulation.
    last_exit_range = exit_range;

    return true;
  };

  glm::dvec3 start;
  glm::dvec3 end;
  unsigned filter_flags;
  double time_base = 0;

  if (timestamps)
  {
    // Update first ray time if not yet set.
    map_->updateFirstRayTime(*timestamps);
  }
  time_base = map_->firstRayTime();

  for (size_t i = 0; i < element_count; i += 2)
  {
    filter_flags = 0;
    start = rays[i];
    end = rays[i + 1];

    if (use_filter)
    {
      if (!ray_filter(&start, &end, &filter_flags))
      {
        // Bad ray.
        continue;
      }
    }

    const bool include_sample_in_ray = (filter_flags & kRffClippedEnd) || (ray_update_flags & kRfEndPointAsFree);

    if (!(ray_update_flags & kRfExcludeRay))
    {
      stop_adjustments = false;
      walkSegmentKeys(LineWalkContext(*map_, visit_func), start, end, include_sample_in_ray);
    }

    if (!stop_adjustments && !include_sample_in_ray && !(ray_update_flags & kRfExcludeSample))
    {
      // Like the miss logic, we have similar obfuscation here to avoid branching. It's a little simpler though,
      // because we do have a branch above, which will filter some of the conditions catered for in miss integration.
      const ohm::Key key = map_->voxelKey(end);
      MapChunk *chunk =
        (last_chunk && key.regionKey() == last_chunk->region.coord) ? last_chunk : map_->region(key.regionKey(), true);
      if (chunk != last_chunk)
      {
        occupancy_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
        if (traversal_layer >= 0)
        {
          traversal_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[traversal_layer]);
        }
        if (touch_time_layer_ >= 0 && timestamps)
        {
          touch_time_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[touch_time_layer_]);
        }
        if (incident_normal_layer_ >= 0)
        {
          incidents_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[incident_normal_layer_]);
        }
      }
      last_chunk = chunk;
      const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);

      float occupancy_value;
      occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
      const float initial_value = occupancy_value;

      const bool initially_unobserved = initial_value == unobservedOccupancyValue();
      const bool initially_free = !initially_unobserved && initial_value < occupancy_threshold_value;
      const bool initially_occupied = !initially_unobserved && initial_value >= occupancy_threshold_value;

      // Calculate the adjustment to make based on the initial occupancy value, various exclusion flags and the
      // configured value adjustment (see the equivalent section for the miss update). Note the adjustment for skipping
      // an initially_unobserved voxel is not zero - it's unobservedOccupancyValue()/infinity to keep the state
      // unchanged.
      float hit_adjustment = hit_value;
      hit_adjustment = (initially_unobserved && (ray_update_flags & kRfExcludeUnobserved)) ?
                         unobservedOccupancyValue() :
                         hit_adjustment;
      hit_adjustment = (initially_free && (ray_update_flags & kRfExcludeFree)) ? 0.0f : hit_adjustment;
      hit_adjustment = (initially_occupied && (ray_update_flags & kRfExcludeOccupied)) ? 0.0f : hit_adjustment;

      occupancyAdjustHit(&occupancy_value, initial_value, hit_adjustment, unobservedOccupancyValue(), voxel_max,
                         saturation_min, saturation_max, stop_adjustments);

      // update voxel mean if present.
      unsigned sample_count = 0;
      if (mean_layer >= 0)
      {
        if (chunk != last_mean_chunk)
        {
          mean_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[mean_layer]);
        }
        last_mean_chunk = chunk;
        VoxelMean voxel_mean;
        mean_buffer.readVoxel(voxel_index, &voxel_mean);
        voxel_mean.coord =
          subVoxelUpdate(voxel_mean.coord, voxel_mean.count, end - map_->voxelCentreGlobal(key), resolution);
        sample_count = voxel_mean.count;
        ++voxel_mean.count;
        mean_buffer.writeVoxel(voxel_index, voxel_mean);
        // Lint(KS): The analyser takes some branches which are not possible in practice.
        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
        chunk->touched_stamps[mean_layer].store(touch_stamp, std::memory_order_relaxed);
      }
      occupancy_buffer.writeVoxel(voxel_index, occupancy_value);

      // Accumulate traversal
      if (traversal_layer >= 0)
      {
        float traversal;
        traversal_buffer.readVoxel(voxel_index, &traversal);
        traversal += float(glm::length(end - start) - last_exit_range);
        traversal_buffer.writeVoxel(voxel_index, traversal);
      }

      if (touch_time_layer_ >= 0 && timestamps)
      {
        const unsigned touch_time = encodeVoxelTouchTime(time_base, timestamps[i >> 1]);
        touch_time_buffer.writeVoxel(voxel_index, touch_time);
      }

      if (incident_normal_layer_ >= 0)
      {
        unsigned packed_normal{};
        incidents_buffer.readVoxel(voxel_index, &packed_normal);
        packed_normal = updateIncidentNormal(packed_normal, start - end, sample_count);
        incidents_buffer.writeVoxel(voxel_index, packed_normal);
      }

      // Lint(KS): The analyser takes some branches which are not possible in practice.
      // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
      chunk->updateFirstValid(voxel_index);

      chunk->dirty_stamp = touch_stamp;
      // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
      // not so much the sequencing. We really don't want to synchronise here.
      chunk->touched_stamps[occupancy_layer].store(touch_stamp, std::memory_order_relaxed);
    }
  }

  return element_count / 2;
}


size_t RayMapperOccupancy::lookupRays(const glm::dvec3 *rays, size_t element_count, float *newly_observed_volumes,
                                      float *ranges, OccupancyType *terminal_states)
{
  RaysQuery query;
  query.setMap(map_);
  query.setVolumeCoefficient((float)(1.0 / 180.0 * M_PI * 1.0 / 180.0 * M_PI));
  query.setRays(rays, element_count);

  query.execute();

  if (newly_observed_volumes)
  {
    memcpy(newly_observed_volumes, query.unobservedVolumes(),
           query.numberOfResults() * sizeof(*newly_observed_volumes));
  }

  if (ranges)
  {
    memcpy(ranges, query.ranges(), query.numberOfResults() * sizeof(*ranges));
  }

  if (terminal_states)
  {
    memcpy(terminal_states, query.terminalOccupancyTypes(), query.numberOfResults() * sizeof(*terminal_states));
  }

  return query.numberOfResults();
}
}  // namespace ohm
