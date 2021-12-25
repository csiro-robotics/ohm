// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RayMapperTsdf.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"
#include "VoxelBuffer.h"
#include "VoxelTsdf.h"

#include <ohmutil/LineWalk.h>

namespace ohm
{
RayMapperTsdf::RayMapperTsdf(OccupancyMap *map)
  : map_(map)
  , tsdf_layer_(map_->layout().layerIndex(default_layer::tsdfLayerName()))
{
  // Use Voxel to validate the layers.
  // In processing we use VoxelBuffer instead of Voxel objects. While Voxel makes for a neater API, using VoxelBuffer
  // makes for less overhead and yields better performance.
  Voxel<const VoxelTsdf> tsdf(map_, tsdf_layer_);

  tsdf_dim_ = tsdf.isLayerValid() ? tsdf.layerDim() : tsdf_dim_;
  valid_ = tsdf.isLayerValid();
}


RayMapperTsdf::~RayMapperTsdf() = default;


void RayMapperTsdf::setTsdfOptions(const TsdfOptions &options)
{
  tsdf_options_ = options;
  if (map_)
  {
    updateMapInfo(map_->mapInfo(), tsdf_options_);
  }
}


void RayMapperTsdf::setMaxWeight(float max_weight)
{
  tsdf_options_.max_weight = max_weight;
  if (map_)
  {
    updateMapInfo(map_->mapInfo(), tsdf_options_);
  }
}


void RayMapperTsdf::setDefaultTruncationDistance(float default_truncation_distance)
{
  tsdf_options_.default_truncation_distance = default_truncation_distance;
  if (map_)
  {
    updateMapInfo(map_->mapInfo(), tsdf_options_);
  }
}


void RayMapperTsdf::setDropoffEpsilon(float dropoff_epsilon)
{
  tsdf_options_.dropoff_epsilon = dropoff_epsilon;
  if (map_)
  {
    updateMapInfo(map_->mapInfo(), tsdf_options_);
  }
}


void RayMapperTsdf::setSparsityCompensationFactor(float sparsity_compensation_factor)
{
  tsdf_options_.sparsity_compensation_factor = sparsity_compensation_factor;
  if (map_)
  {
    updateMapInfo(map_->mapInfo(), tsdf_options_);
  }
}


size_t RayMapperTsdf::integrateRays(const glm::dvec3 *rays, size_t element_count, const float * /*intensities*/,
                                    const double *timestamps, unsigned ray_update_flags)
{
  KeyList keys;
  MapChunk *last_chunk = nullptr;
  MapChunk *last_mean_chunk = nullptr;
  VoxelBuffer<VoxelBlock> tsdf_buffer;

  const RayFilterFunction ray_filter = map_->rayFilter();
  const bool use_filter = bool(ray_filter);
  const auto tsdf_layer = tsdf_layer_;
  const auto tsdf_dim = tsdf_dim_;
  // Touch the map to flag changes.
  const auto touch_stamp = map_->touch();

  // Cached values for current ray to use in the update closure.
  glm::dvec3 sensor{};
  glm::dvec3 sample{};

  if (timestamps)
  {
    // Update first ray time if not yet set.
    map_->updateFirstRayTime(*timestamps);
  }

  const auto visit_func = [&](const Key &key, double /*enter_range*/, double /*exit_range*/) -> bool  //
  {                                                                                                   //
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
      tsdf_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[tsdf_layer]);
    }
    last_chunk = chunk;
    const unsigned voxel_index = ohm::voxelIndex(key, tsdf_dim);
    VoxelTsdf tsdf_voxel;
    tsdf_buffer.readVoxel(voxel_index, &tsdf_voxel);

    calculateTsdf(sensor, sample, map_->voxelCentreGlobal(key), tsdf_options_.default_truncation_distance,
                  tsdf_options_.max_weight, tsdf_options_.dropoff_epsilon, tsdf_options_.sparsity_compensation_factor,
                  &tsdf_voxel.weight, &tsdf_voxel.distance);
    tsdf_buffer.writeVoxel(voxel_index, tsdf_voxel);

    // Lint(KS): The analyser takes some branches which are not possible in practice.
    // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
    chunk->updateFirstValid(voxel_index);

    chunk->dirty_stamp = touch_stamp;
    // Update the touched_stamps with relaxed memory ordering. The important thing is to have an update,
    // not so much the sequencing. We really don't want to synchronise here.
    chunk->touched_stamps[tsdf_layer].store(touch_stamp, std::memory_order_relaxed);

    return true;
  };

  for (size_t i = 0; i < element_count; i += 2)
  {
    unsigned filter_flags = 0;
    sensor = rays[i];
    sample = rays[i + 1];

    if (use_filter)
    {
      if (!ray_filter(&sensor, &sample, &filter_flags))
      {
        // Bad ray.
        continue;
      }
    }

    ohm::walkSegmentKeys<Key>(visit_func, sensor, sample, true, WalkKeyAdaptor(*map_));
  }

  return element_count / 2;
}
}  // namespace ohm
