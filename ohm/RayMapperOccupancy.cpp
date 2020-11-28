//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#include "RayMapperOccupancy.h"

#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"
#include "VoxelBuffer.h"
#include "VoxelMean.h"
#include "VoxelOccupancy.h"

#include <ohmutil/LineWalk.h>

namespace ohm
{
RayMapperOccupancy::RayMapperOccupancy(OccupancyMap *map)
  : map_(map)
  , occupancy_layer_(map_->layout().occupancyLayer())
  , mean_layer_(map_->layout().meanLayer())
{
  // Use Voxel to validate the layers.
  // In processing we use VoxelBuffer instead of Voxel objects. While Voxel mapes for a neader API, using VoxelBuffer
  // makes for less overhead and yields better performance.
  Voxel<const float> occupancy(map_, occupancy_layer_);
  Voxel<const VoxelMean> mean(map_, mean_layer_);

  occupancy_dim_ = occupancy.isLayerValid() ? occupancy.layerDim() : occupancy_dim_;

  // Validate we only have an occupancy layer or we also have a mean layer and the layer dimesions match.
  valid_ = occupancy.isLayerValid() && !mean.isLayerValid() ||
           occupancy.isLayerValid() && mean.isLayerValid() && occupancy.layerDim() == mean.layerDim();
}


RayMapperOccupancy::~RayMapperOccupancy() = default;


size_t RayMapperOccupancy::integrateRays(const glm::dvec3 *rays, size_t element_count, unsigned ray_update_flags)
{
  KeyList keys;
  MapChunk *last_chunk = nullptr;
  MapChunk *last_mean_chunk = nullptr;
  VoxelBuffer<VoxelBlock> occupancy_buffer;
  VoxelBuffer<VoxelBlock> mean_buffer;
  bool stop_adjustments = false;

  const RayFilterFunction ray_filter = map_->rayFilter();
  const bool use_filter = bool(ray_filter);
  const auto occupancy_layer = occupancy_layer_;
  const auto mean_layer = mean_layer_;
  const auto occupancy_dim = occupancy_dim_;
  const auto occupancy_threshold_value = map_->occupancyThresholdValue();
  const auto map_origin = map_->origin();
  const auto miss_value = map_->missValue();
  const auto hit_value = map_->hitValue();
  const auto resolution = map_->resolution();
  const auto voxel_min = map_->minVoxelValue();
  const auto voxel_max = map_->maxVoxelValue();
  const auto saturation_min = map_->saturateAtMinValue() ? voxel_min : std::numeric_limits<float>::lowest();
  const auto saturation_max = map_->saturateAtMaxValue() ? voxel_max : std::numeric_limits<float>::max();
  // Touch the map to flag changes.
  const auto touch_stamp = map_->touch();

  const auto visit_func = [&](const Key &key, float /*enter_range*/, float /*exit_range*/)  //
  {                                                                                 //
    // The update logic here is a little unclear as it tries to avoid outright branches.
    // The intended logic is described as follows:
    // 1. Select direct write or additive adjustment.
    //    - Make a direct, non-additive adjustment if one of the following conditions are met:
    //      - stop_adjustments is true
    //      - the voxel is uncertain
    //      - (ray_update_flags & kRfClearOnly) and not is_occupied - we only want to adjust occupied voxels.
    //      - voxel is saturated
    //    - Otherwise add to present value.
    // 2. Select the value adjustment
    //    - current_value if one of the following conditions are met:
    //      - stop_adjustments is true (no longer making adjustments)
    //      - (ray_update_flags & kRfClearOnly) and not is_occupied (only looking to affect occupied voxels)
    //    - miss_value otherwise
    // 3. Calculate new value
    // 4. Apply saturation logic: only min saturation relevant
    //    -
    MapChunk *chunk =
      (last_chunk && key.regionKey() == last_chunk->region.coord) ? last_chunk : map_->region(key.regionKey(), true);
    if (chunk != last_chunk)
    {
      occupancy_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
    }
    last_chunk = chunk;
    const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
    float occupancy_value;
    occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
    const float initial_value = occupancy_value;
    const bool is_occupied = (initial_value != unobservedOccupancyValue() && initial_value > occupancy_threshold_value);
    occupancyAdjustMiss(&occupancy_value, initial_value, miss_value, unobservedOccupancyValue(), voxel_min,
                        saturation_min, saturation_max, stop_adjustments);
    occupancy_buffer.writeVoxel(voxel_index, occupancy_value);
    // Lint(KS): The analyser takes some branches which are not possible in practice.
    // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
    chunk->updateFirstValid(voxel_index);

    stop_adjustments = stop_adjustments || ((ray_update_flags & kRfStopOnFirstOccupied) && is_occupied);
    chunk->dirty_stamp = touch_stamp;
    // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
    // not so much the sequencing. We really don't want to synchronise here.
    chunk->touched_stamps[occupancy_layer].store(touch_stamp, std::memory_order_relaxed);
  };

  glm::dvec3 start;
  glm::dvec3 end;
  unsigned filter_flags;
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

    const bool include_sample_in_ray =
      (filter_flags & kRffClippedEnd) || (ray_update_flags & kRfEndPointAsFree) || (ray_update_flags & kRfClearOnly);

    if (!(ray_update_flags & kRfExcludeRay))
    {
      // Calculate line key for the last voxel if the end point has been clipped
      const glm::dvec3 start_point_local = glm::dvec3(start - map_origin);
      const glm::dvec3 end_point_local = glm::dvec3(end - map_origin);

      stop_adjustments = false;
      ohm::walkSegmentKeys<Key>(visit_func, start_point_local, end_point_local, include_sample_in_ray,
                                WalkKeyAdaptor(*map_));
    }

    if (!stop_adjustments && !include_sample_in_ray && !(ray_update_flags & (kRfClearOnly | kRfExcludeSample)) &&
        !(ray_update_flags & kRfExcludeSample))
    {
      // Like the miss logic, we have similar obfuscation here to avoid branching. It's a little simpler though,
      // because we do have a branch above, which will filter some of the conditions catered for in miss integration.
      const ohm::Key key = map_->voxelKey(end);
      MapChunk *chunk =
        (last_chunk && key.regionKey() == last_chunk->region.coord) ? last_chunk : map_->region(key.regionKey(), true);
      if (chunk != last_chunk)
      {
        occupancy_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
      }
      last_chunk = chunk;
      const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);

      float occupancy_value;
      occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
      const float initial_value = occupancy_value;
      occupancyAdjustHit(&occupancy_value, initial_value, hit_value, unobservedOccupancyValue(), voxel_max,
                         saturation_min, saturation_max, stop_adjustments);

      // update voxel mean if present.
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
        ++voxel_mean.count;
        mean_buffer.writeVoxel(voxel_index, voxel_mean);
        // Lint(KS): The analyser takes some branches which are not possible in practice.
        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
        chunk->touched_stamps[mean_layer].store(touch_stamp, std::memory_order_relaxed);
      }
      occupancy_buffer.writeVoxel(voxel_index, occupancy_value);
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
  KeyList keys;
  MapChunk *last_chunk = nullptr;
  VoxelBuffer<VoxelBlock> occupancy_buffer;
  bool stop_adjustments = false;
  float newly_observed_volume;
  float range;
  OccupancyType terminal_state;

  const RayFilterFunction ray_filter = map_->rayFilter();
  const bool use_filter = bool(ray_filter);
  const auto occupancy_layer = occupancy_layer_;
  const auto occupancy_dim = occupancy_dim_;
  const auto occupancy_threshold_value = map_->occupancyThresholdValue();
  const auto map_origin = map_->origin();
  const float ray_solid_angle = (float)(1 / 180 * M_PI * 1 / 180 * M_PI); // TODO: parameterise
  // Touch the map to flag changes.

  const auto visit_func = [&](const Key &key, float enter_range, float exit_range)  //
  {                                                                                 //
    // The update logic here is a little unclear as it tries to avoid outright branches.
    // The intended logic is described as follows:
    // 1. Select direct write or additive adjustment.
    //    - Make a direct, non-additive adjustment if one of the following conditions are met:
    //      - stop_adjustments is true
    //      - the voxel is uncertain
    //      - (ray_update_flags & kRfClearOnly) and not is_occupied - we only want to adjust occupied voxels.
    //      - voxel is saturated
    //    - Otherwise add to present value.
    // 2. Select the value adjustment
    //    - current_value if one of the following conditions are met:
    //      - stop_adjustments is true (no longer making adjustments)
    //      - (ray_update_flags & kRfClearOnly) and not is_occupied (only looking to affect occupied voxels)
    //    - miss_value otherwise
    // 3. Calculate new value
    // 4. Apply saturation logic: only min saturation relevant
    //    -
    MapChunk *chunk =
      (last_chunk && key.regionKey() == last_chunk->region.coord) ? last_chunk : map_->region(key.regionKey(), true);
    if (chunk != last_chunk)
    {
      occupancy_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
    }
    last_chunk = chunk;
    const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
    float occupancy_value;
    occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
    const bool is_unobserved = occupancy_value == unobservedOccupancyValue();
    const bool is_occupied = !is_unobserved && occupancy_value > occupancy_threshold_value;
    if (!stop_adjustments)
    {
      newly_observed_volume += is_unobserved * ray_solid_angle *
                               (exit_range * exit_range * exit_range - enter_range * enter_range * enter_range);
      range = exit_range;
      terminal_state =
        is_unobserved ? OccupancyType::kUnobserved : (is_occupied ? OccupancyType::kOccupied : OccupancyType::kFree);
    }
    // Lint(KS): The analyser takes some branches which are not possible in practice.
    // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
    chunk->updateFirstValid(voxel_index); // Question(JW): what does this do

    stop_adjustments = stop_adjustments || is_occupied;
  };

  glm::dvec3 start;
  glm::dvec3 end;
  unsigned filter_flags;
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

    // Calculate line key for the last voxel if the end point has been clipped
    const glm::dvec3 start_point_local = glm::dvec3(start - map_origin);
    const glm::dvec3 end_point_local = glm::dvec3(end - map_origin);

    stop_adjustments = false;
    newly_observed_volume = 0.0f;
    range = 0.0f;
    terminal_state = OccupancyType::kNull;
    ohm::walkSegmentKeys<Key>(visit_func, start_point_local, end_point_local, true, WalkKeyAdaptor(*map_));

    if (newly_observed_volumes)
    {
      newly_observed_volumes[i >> 1] = newly_observed_volume;
    }
    if (ranges)
    {
      ranges[i >> 1] = range;
    }
    if (terminal_states)
    {
      terminal_states[i >> 1] = terminal_state;
    }
  }

  return element_count / 2;
}


}  // namespace ohm
