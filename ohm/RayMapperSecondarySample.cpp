// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RayMapperSecondarySample.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"
#include "VoxelBuffer.h"
#include "VoxelSecondarySample.h"

namespace ohm
{
RayMapperSecondarySample::RayMapperSecondarySample(OccupancyMap *map)
  : map_(map)
  , secondary_samples_layer_(map_->layout().layerIndex(default_layer::secondarySamplesLayerName()))
{
  // Use Voxel to validate the layers.
  // In processing we use VoxelBuffer instead of Voxel objects. While Voxel makes for a neater API, using VoxelBuffer
  // makes for less overhead and yields better performance.
  Voxel<const VoxelSecondarySample> secondary_sample(map_, secondary_samples_layer_);

  layer_dim_ = secondary_sample.isLayerValid() ? secondary_sample.layerDim() : layer_dim_;
  valid_ = secondary_sample.isLayerValid();
}


RayMapperSecondarySample::~RayMapperSecondarySample() = default;


size_t RayMapperSecondarySample::integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                                               const double *timestamps, unsigned ray_update_flags)
{
  (void)intensities;
  (void)timestamps;
  (void)ray_update_flags;

  MapChunk *last_chunk = nullptr;
  VoxelBuffer<VoxelBlock> secondary_sample_buffer;
  VoxelSecondarySample voxel{};

  const auto secondary_samples_layer = secondary_samples_layer_;
  const auto layer_dim = layer_dim_;
  // Touch the map to flag changes.
  const auto touch_stamp = map_->touch();

  for (size_t i = 0; i < element_count; i += 2)
  {
    const double range = glm::length(rays[i + 1] - rays[i]);
    const ohm::Key key = map_->voxelKey(rays[i + 1]);
    MapChunk *chunk =
      (last_chunk && key.regionKey() == last_chunk->region.coord) ? last_chunk : map_->region(key.regionKey(), true);

    if (chunk != last_chunk)
    {
      secondary_sample_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[secondary_samples_layer]);
    }
    last_chunk = chunk;
    const unsigned voxel_index = ohm::voxelIndex(key, layer_dim);

    secondary_sample_buffer.readVoxel(voxel_index, &voxel);
    addSecondarySample(voxel, range);
    secondary_sample_buffer.writeVoxel(voxel_index, voxel);

    assert(chunk);
    chunk->dirty_stamp = touch_stamp;
    chunk->touched_stamps[secondary_samples_layer].store(touch_stamp, std::memory_order_relaxed);
  }

  return element_count / 2;
}
}  // namespace ohm
