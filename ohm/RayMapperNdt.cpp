//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#include "RayMapperNdt.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "CovarianceVoxel.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "NdtMap.h"
#include "OccupancyMap.h"
#include "RayFilter.h"
#include "VoxelBuffer.h"
#include "VoxelData.h"

#include <ohmutil/LineWalk.h>

#include <iostream>

namespace ohm
{
RayMapperNdt::RayMapperNdt(NdtMap *map)
  : map_(map)
  , occupancy_layer_(map_->map().layout().occupancyLayer())
  , mean_layer_(map_->map().layout().meanLayer())
  , traversal_layer_(map_->map().layout().traversalLayer())
  , covariance_layer_(map_->map().layout().covarianceLayer())
  , intensity_layer_(map_->map().layout().intensityLayer())
  , hit_miss_count_layer_(map_->map().layout().hitMissCountLayer())
  , ndt_tm_(map->mode() == NdtMode::kTraversability)
{
  OccupancyMap *map_ptr = &map_->map();

  Voxel<const float> occupancy(map_ptr, occupancy_layer_);
  Voxel<const VoxelMean> mean(map_ptr, mean_layer_);
  Voxel<const CovarianceVoxel> cov(map_ptr, covariance_layer_);
  Voxel<const float> traversal(map_ptr, traversal_layer_);

  occupancy_dim_ = (occupancy.isLayerValid()) ? occupancy.layerDim() : occupancy_dim_;

  // Validate we have occupancy, mean and covariance layers and their dimensions match.
  valid_ = occupancy.isLayerValid() && mean.isLayerValid() && cov.isLayerValid() &&
           occupancy.layerDim() == mean.layerDim() && occupancy.layerDim() == cov.layerDim();
  // Validate the traversal layer in a simliar fashion.
  valid_ = occupancy.isLayerValid() && !traversal.isLayerValid() ||
           occupancy.isLayerValid() && traversal.isLayerValid() && occupancy.layerDim() == traversal.layerDim();

  if (ndt_tm_)
  {
    Voxel<const IntensityMeanCov> intensity(map_ptr, intensity_layer_);
    Voxel<const HitMissCount> hit_miss(map_ptr, hit_miss_count_layer_);
    valid_ = valid_ && intensity.isLayerValid() && occupancy.layerDim() == intensity.layerDim() &&
             hit_miss.isLayerValid() && occupancy.layerDim() == hit_miss.layerDim();
  }
}


RayMapperNdt::~RayMapperNdt() = default;


size_t RayMapperNdt::integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                                   unsigned ray_update_flags)
{
  KeyList keys;
  MapChunk *last_chunk = nullptr;
  VoxelBuffer<VoxelBlock> occupancy_buffer;
  VoxelBuffer<VoxelBlock> mean_buffer;
  VoxelBuffer<VoxelBlock> cov_buffer;
  VoxelBuffer<VoxelBlock> intensity_buffer;
  VoxelBuffer<VoxelBlock> hit_miss_count_buffer;
  VoxelBuffer<VoxelBlock> traversal_buffer;
  double last_exit_range = 0;
  bool stop_adjustments = false;

  OccupancyMap &occupancy_map = map_->map();
  const RayFilterFunction ray_filter = occupancy_map.rayFilter();
  const bool use_filter = bool(ray_filter);
  const auto occupancy_layer = occupancy_layer_;
  const auto occupancy_dim = occupancy_dim_;
  const auto miss_value = occupancy_map.missValue();
  const auto hit_value = occupancy_map.hitValue();
  const auto resolution = occupancy_map.resolution();
  const auto voxel_min = occupancy_map.minVoxelValue();
  const auto voxel_max = occupancy_map.maxVoxelValue();
  const auto saturation_min = occupancy_map.saturateAtMinValue() ? voxel_min : std::numeric_limits<float>::lowest();
  const auto saturation_max = occupancy_map.saturateAtMaxValue() ? voxel_max : std::numeric_limits<float>::max();
  const auto sensor_noise = map_->sensorNoise();
  const auto ndt_adaptation_rate = map_->adaptationRate();
  const auto ndt_sample_threshold = map_->ndtSampleThreshold();

  // Mean and covariance layers must exists.
  const auto mean_layer = mean_layer_;
  const auto traversal_layer = traversal_layer_;
  const auto covariance_layer = covariance_layer_;

  // Compulsory if using NdtMode::kTraversability
  const auto intensity_layer = intensity_layer_;
  const auto hit_miss_count_layer = hit_miss_count_layer_;

  // Touch the map to flag changes.
  const auto touch_stamp = occupancy_map.touch();

  glm::dvec3 start;
  glm::dvec3 sample;
  float intensity = 0.0f;

  const auto visit_func = [&](const Key &key, double enter_range, double exit_range) -> bool  //
  {
    //
    // The update logic here is a little unclear as it tries to avoid outright branches.
    // The intended logic is described as follows:
    // 1. Select direct write or additive adjustment.
    //    - Make a direct, non-additive adjustment if one of the following conditions are met:
    //      - stop_adjustments is true
    //      - the voxel is uncertain
    //      - voxel is saturated
    //    - Otherwise add to present value.
    // 2. Select the value adjustment
    //    - current_value if one of the following conditions are met:
    //      - stop_adjustments is true (no longer making adjustments)
    //    - miss_value otherwise
    // 3. Calculate new value
    // 4. Apply saturation logic: only min saturation relevant
    //    -
    MapChunk *chunk = (last_chunk && key.regionKey() == last_chunk->region.coord) ?
                        last_chunk :
                        occupancy_map.region(key.regionKey(), true);
    if (chunk != last_chunk)
    {
      occupancy_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
      mean_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[mean_layer]);
      cov_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[covariance_layer]);
      if (ndt_tm_)
      {
        // Intensity not required for miss update, but we need it in sync for the update later.
        intensity_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[intensity_layer_]);
        hit_miss_count_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[hit_miss_count_layer]);
      }
      if (traversal_layer >= 0)
      {
        traversal_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[traversal_layer]);
      }
    }
    last_chunk = chunk;
    const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
    float occupancy_value;
    CovarianceVoxel cov;
    VoxelMean voxel_mean;
    occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
    cov_buffer.readVoxel(voxel_index, &cov);
    mean_buffer.readVoxel(voxel_index, &voxel_mean);
    const glm::dvec3 mean =
      subVoxelToLocalCoord<glm::dvec3>(voxel_mean.coord, resolution) + occupancy_map.voxelCentreGlobal(key);
    const float initial_value = occupancy_value;
    float adjusted_value = initial_value;

    bool is_miss = false;
    calculateMissNdt(&cov, &adjusted_value, &is_miss, start, sample, mean, voxel_mean.count, unobservedOccupancyValue(),
                     miss_value, ndt_adaptation_rate, sensor_noise, ndt_sample_threshold);

    if (ndt_tm_)
    {
      // Note we don't need hit count in miss calculation.
      HitMissCount hit_miss_count_voxel;
      hit_miss_count_buffer.readVoxel(voxel_index, &hit_miss_count_voxel);
      hit_miss_count_voxel.miss_count += (is_miss) ? 1u : 0u;
      hit_miss_count_buffer.writeVoxel(voxel_index, hit_miss_count_voxel);
    }

    occupancyAdjustDown(&occupancy_value, initial_value, adjusted_value, unobservedOccupancyValue(), voxel_min,
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

    stop_adjustments = stop_adjustments;
    chunk->dirty_stamp = touch_stamp;
    // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
    // not so much the sequencing. We really don't want to synchronise here.
    chunk->touched_stamps[occupancy_layer].store(touch_stamp, std::memory_order_relaxed);
    if (ndt_tm_)
    {
      chunk->touched_stamps[hit_miss_count_layer].store(touch_stamp, std::memory_order_relaxed);
    }

    // Store last exit range for final traversal accumulation.
    last_exit_range = exit_range;

    return true;
  };

  unsigned filter_flags;
  float min_int = 1e6f, max_int = 0.0f;
  for (size_t i = 0; i < element_count; i += 2)
  {
    filter_flags = 0;
    start = rays[i];
    sample = rays[i + 1];
    if (intensities)
    {
      intensity = intensities[i >> 1];
    }

    if (use_filter)
    {
      if (!ray_filter(&start, &sample, &filter_flags))
      {
        // Bad ray.
        continue;
      }
    }

    const bool include_sample_in_ray = (filter_flags & kRffClippedEnd);

    if (!(ray_update_flags & kRfExcludeRay))
    {
      stop_adjustments = false;
      ohm::walkSegmentKeys<Key>(visit_func, start, sample, include_sample_in_ray, WalkKeyAdaptor(occupancy_map));
    }

    if (!stop_adjustments && !include_sample_in_ray)
    {
      // Like the miss logic, we have similar obfuscation here to avoid branching. It's a little simpler though,
      // because we do have a branch above, which will filter some of the conditions catered for in miss integration.
      const ohm::Key key = occupancy_map.voxelKey(sample);
      MapChunk *chunk = (last_chunk && key.regionKey() == last_chunk->region.coord) ?
                          last_chunk :
                          occupancy_map.region(key.regionKey(), true);
      if (chunk != last_chunk)
      {
        occupancy_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[occupancy_layer]);
        mean_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[mean_layer]);
        cov_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[covariance_layer_]);
        if (ndt_tm_)
        {
          intensity_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[intensity_layer_]);
          hit_miss_count_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[hit_miss_count_layer_]);
        }
        if (traversal_layer >= 0)
        {
          traversal_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[traversal_layer]);
        }
      }
      last_chunk = chunk;
      const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
      const glm::dvec3 voxel_centre = occupancy_map.voxelCentreGlobal(key);
      float occupancy_value;
      CovarianceVoxel cov;
      VoxelMean voxel_mean;
      occupancy_buffer.readVoxel(voxel_index, &occupancy_value);
      cov_buffer.readVoxel(voxel_index, &cov);
      mean_buffer.readVoxel(voxel_index, &voxel_mean);
      const glm::dvec3 mean = subVoxelToLocalCoord<glm::dvec3>(voxel_mean.coord, resolution) + voxel_centre;
      const float initial_value = occupancy_value;
      float adjusted_value = initial_value;

      IntensityMeanCov intensity_voxel;
      HitMissCount hit_miss_count_voxel;
      if (ndt_tm_)
      {
        intensity_buffer.readVoxel(voxel_index, &intensity_voxel);
        hit_miss_count_buffer.readVoxel(voxel_index, &hit_miss_count_voxel);

        const bool reinitialise_permeability_with_covariance = true;  // TODO: make a parameter of map
        calculateHitMissUpdateOnHit(&cov, adjusted_value, &hit_miss_count_voxel, start, sample, mean, voxel_mean.count,
                                    unobservedOccupancyValue(), reinitialise_permeability_with_covariance,
                                    ndt_adaptation_rate, sensor_noise, map_->reinitialiseCovarianceThreshold(),
                                    map_->reinitialiseCovariancePointCount(), ndt_sample_threshold);

        calculateIntensityUpdateOnHit(&intensity_voxel, adjusted_value, intensity, map_->initialIntensityCovariance(),
                                      voxel_mean.count, map_->reinitialiseCovarianceThreshold(),
                                      map_->reinitialiseCovariancePointCount());

        min_int = std::fmin(min_int, intensity_voxel.intensity_mean);
        max_int = std::fmax(max_int, intensity_voxel.intensity_mean);
      }

      const bool reset_mean = calculateHitWithCovariance(
        &cov, &adjusted_value, sample, mean, voxel_mean.count, hit_value, unobservedOccupancyValue(), float(resolution),
        map_->reinitialiseCovarianceThreshold(), map_->reinitialiseCovariancePointCount());
      occupancyAdjustUp(&occupancy_value, initial_value, adjusted_value, unobservedOccupancyValue(), voxel_max,
                        saturation_min, saturation_max, stop_adjustments);

      voxel_mean.count = (!reset_mean) ? voxel_mean.count : 0;
      voxel_mean.coord = subVoxelUpdate(voxel_mean.coord, voxel_mean.count, sample - voxel_centre, resolution);
      ++voxel_mean.count;

      occupancy_buffer.writeVoxel(voxel_index, occupancy_value);
      cov_buffer.writeVoxel(voxel_index, cov);
      mean_buffer.writeVoxel(voxel_index, voxel_mean);
      if (ndt_tm_)
      {
        intensity_buffer.writeVoxel(voxel_index, intensity_voxel);
        hit_miss_count_buffer.writeVoxel(voxel_index, hit_miss_count_voxel);
      }

      // Accumulate traversal
      if (traversal_layer >= 0)
      {
        float traversal;
        traversal_buffer.readVoxel(voxel_index, &traversal);
        traversal += float(glm::length(sample - start) - last_exit_range);
        traversal_buffer.writeVoxel(voxel_index, traversal);
      }

      // Lint(KS): The analyser takes some branches which are not possible in practice.
      // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
      chunk->updateFirstValid(voxel_index);

      chunk->dirty_stamp = touch_stamp;
      // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
      // not so much the sequencing. We really don't want to synchronise here.
      chunk->touched_stamps[occupancy_layer].store(touch_stamp, std::memory_order_relaxed);
      chunk->touched_stamps[mean_layer].store(touch_stamp, std::memory_order_relaxed);
      chunk->touched_stamps[covariance_layer].store(touch_stamp, std::memory_order_relaxed);
      if (ndt_tm_)
      {
        chunk->touched_stamps[intensity_layer].store(touch_stamp, std::memory_order_relaxed);
        chunk->touched_stamps[hit_miss_count_layer].store(touch_stamp, std::memory_order_relaxed);
      }
    }
  }

  return element_count / 2;
}
}  // namespace ohm
