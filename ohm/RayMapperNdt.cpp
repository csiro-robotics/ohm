//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#include "RayMapperNdt.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtx/norm.hpp>

#include "CovarianceVoxel.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "OccupancyMap.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "NdtMap.h"
#include "RayFilter.h"
#include "VoxelData.h"

#include <ohmutil/LineWalk.h>

using namespace ohm;

RayMapperNdt::RayMapperNdt(NdtMap *map)
  : map_(map)
  , occupancy_layer_(map_->map().layout().occupancyLayer())
  , mean_layer_(map_->map().layout().meanLayer())
  , covariance_layer_(map_->map().layout().covarianceLayer())
{
  OccupancyMap &omap = map_->map();
  occupancy_dim_ = omap.layout().layer(occupancy_layer_).dimensions(omap.regionVoxelDimensions());
  valid_ = true;
  // Validate the mean layer size.
  if (mean_layer_ >= 0)
  {
    if (omap.layout().layer(mean_layer_).voxelByteSize() != sizeof(VoxelMean))
    {
      // Unexpected layer size.
      valid_ = false;
    }
    else if (omap.layout().layer(mean_layer_).dimensions(omap.regionVoxelDimensions()) != occupancy_dim_)
    {
      // Unexpected layer dimensions.
      valid_ = false;
    }
  }
  else
  {
    // Missing layer
    valid_ = false;
  }

  // Validate the covariance layer size.
  if (covariance_layer_ >= 0)
  {
    if (omap.layout().layer(covariance_layer_).voxelByteSize() != sizeof(CovarianceVoxel))
    {
      // Unexpected layer size.
      valid_ = false;
    }
    else if (omap.layout().layer(covariance_layer_).dimensions(omap.regionVoxelDimensions()) != occupancy_dim_)
    {
      // Unexpected layer dimensions.
      valid_ = false;
    }
  }
  else
  {
    // Missing layer
    valid_ = false;
  }
}


RayMapperNdt::~RayMapperNdt() = default;


size_t RayMapperNdt::integrateRays(const glm::dvec3 *rays, size_t element_count, unsigned ray_update_flags)
{
  KeyList keys;
  MapChunk *last_chunk = nullptr;
  bool stop_adjustments = false;

  OccupancyMap &occupancy_map = map_->map();
  const RayFilterFunction ray_filter = occupancy_map.rayFilter();
  const bool use_filter = bool(ray_filter);
  const auto occupancy_layer = occupancy_layer_;
  const auto occupancy_dim = occupancy_dim_;
  // const auto occupancy_threshold_value = occupancy_map.occupancyThresholdValue();
  const auto map_origin = occupancy_map.origin();
  const auto miss_value = occupancy_map.missValue();
  const auto hit_value = occupancy_map.hitValue();
  const auto resolution = occupancy_map.resolution();
  const auto voxel_min = occupancy_map.minVoxelValue();
  const auto voxel_max = occupancy_map.maxVoxelValue();
  const auto saturation_min = occupancy_map.saturateAtMinValue() ? voxel_min : std::numeric_limits<float>::lowest();
  const auto saturation_max = occupancy_map.saturateAtMaxValue() ? voxel_max : std::numeric_limits<float>::max();
  const auto adaptation_rate = map_->adaptationRate();
  const auto sensor_noise = map_->sensorNoise();
  const auto ndt_sample_threshold = map_->ndtSampleThreshold();

  // Mean and covariance layers must exists.
  const auto mean_layer = mean_layer_;
  const auto covariance_layer = covariance_layer_;

  // Touch the map to flag changes.
  const auto touch_stamp = occupancy_map.touch();

  glm::dvec3 start, sample;

  const auto visit_func = [&](const Key &key)  //
  {
    //
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
    MapChunk *chunk = (last_chunk && key.regionKey() == last_chunk->region.coord) ?
                        last_chunk :
                        occupancy_map.region(key.regionKey(), true);
    last_chunk = chunk;
    const unsigned voxel_index = ::voxelIndex(key, occupancy_dim);
    float *occupancy_value = reinterpret_cast<float *>(chunk->voxel_maps[occupancy_layer]) + +voxel_index;
    const CovarianceVoxel *cov = reinterpret_cast<CovarianceVoxel *>(chunk->voxel_maps[covariance_layer]) + voxel_index;
    const VoxelMean *voxel_mean = reinterpret_cast<VoxelMean *>(chunk->voxel_maps[mean_layer]) + voxel_index;
    const glm::dvec3 mean =
      subVoxelToLocalCoord<glm::dvec3>(voxel_mean->coord, resolution) + occupancy_map.voxelCentreGlobal(key);
    const float initial_value = *occupancy_value;
    float adjusted_value = initial_value;

    calculateMissNdt(cov, &adjusted_value, start, sample, mean, voxel_mean->count, unobservedOccupancyValue(),
                     miss_value, adaptation_rate, sensor_noise, ndt_sample_threshold);
    occupancyAdjustDown(occupancy_value, initial_value, adjusted_value, unobservedOccupancyValue(), voxel_min,
                        saturation_min, saturation_max, stop_adjustments);
    chunk->updateFirstValid(voxel_index);

    // stop_adjustments = stop_adjustments || ((ray_update_flags & kRfStopOnFirstOccupied) && is_occupied);
    chunk->dirty_stamp = touch_stamp;
    // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
    // not so much the sequencing. We really don't want to synchronise here.
    chunk->touched_stamps[occupancy_layer].store(touch_stamp, std::memory_order_relaxed);
  };

  unsigned filter_flags;
  for (size_t i = 0; i < element_count; i += 2)
  {
    filter_flags = 0;
    start = rays[i];
    sample = rays[i + 1];

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
      // Calculate line key for the last voxel if the sample point has been clipped
      const glm::dvec3 start_point_local = glm::dvec3(start - map_origin);
      const glm::dvec3 end_point_local = glm::dvec3(sample - map_origin);

      stop_adjustments = false;
      ohm::walkSegmentKeys<Key>(visit_func, start_point_local, end_point_local, include_sample_in_ray,
                                WalkKeyAdaptor(occupancy_map));
    }

    if (!stop_adjustments && !include_sample_in_ray)
    {
      // Like the miss logic, we have similar obfuscation here to avoid branching. It's a little simpler though,
      // because we do have a branch above, which will filter some of the conditions catered for in miss integration.
      const ohm::Key key = occupancy_map.voxelKey(sample);
      MapChunk *chunk = (last_chunk && key.regionKey() == last_chunk->region.coord) ?
                          last_chunk :
                          occupancy_map.region(key.regionKey(), true);
      last_chunk = chunk;
      const unsigned voxel_index = ::voxelIndex(key, occupancy_dim);
      const glm::dvec3 voxel_centre = occupancy_map.voxelCentreGlobal(key);
      float *occupancy_value = reinterpret_cast<float *>(chunk->voxel_maps[occupancy_layer]) + voxel_index;
      CovarianceVoxel *cov = reinterpret_cast<CovarianceVoxel *>(chunk->voxel_maps[covariance_layer]) + voxel_index;
      VoxelMean *voxel_mean = reinterpret_cast<VoxelMean *>(chunk->voxel_maps[mean_layer]) + voxel_index;
      const glm::dvec3 mean = subVoxelToLocalCoord<glm::dvec3>(voxel_mean->coord, resolution) + voxel_centre;
      const float initial_value = *occupancy_value;
      float adjusted_value = initial_value;

      const bool reset_mean = calculateHitWithCovariance(
        cov, &adjusted_value, sample, mean, voxel_mean->count, hit_value, unobservedOccupancyValue(), float(resolution),
        map_->reinitialiseCovarianceTheshold(), map_->reinitialiseCovariancePointCount());
      occupancyAdjustUp(occupancy_value, initial_value, adjusted_value, unobservedOccupancyValue(), voxel_max,
                        saturation_min, saturation_max, stop_adjustments);

      voxel_mean->count = (!reset_mean) ? voxel_mean->count : 0;
      voxel_mean->coord = subVoxelUpdate(voxel_mean->coord, voxel_mean->count, sample - voxel_centre, resolution);
      ++voxel_mean->count;

      chunk->updateFirstValid(voxel_index);

      chunk->dirty_stamp = touch_stamp;
      // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
      // not so much the sequencing. We really don't want to synchronise here.
      chunk->touched_stamps[occupancy_layer].store(touch_stamp, std::memory_order_relaxed);
      chunk->touched_stamps[mean_layer].store(touch_stamp, std::memory_order_relaxed);
      chunk->touched_stamps[covariance_layer].store(touch_stamp, std::memory_order_relaxed);
    }
  }

  return element_count / 2;
}
