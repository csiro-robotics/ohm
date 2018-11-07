//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "Voxel.h"

#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include "DefaultLayers.h"
#include "MapChunk.h"
#include "VoxelLayout.h"

#include <algorithm>
#include <cassert>
#include <cstdio>

using namespace ohm;

namespace
{
  // Get a voxel pointer as uint8_t * or const uint8_t *. No other types for T are supported.
  template <typename T, typename MAPCHUNK>
  T getVoxelBytePtr(const Key &key, MAPCHUNK *chunk, const OccupancyMapDetail *map, int layer_index, size_t expected_size)
  {
    if (chunk && map && key.regionKey() == chunk->region.coord)
    {
      // Validate layer.
      assert(layer_index < int(chunk->layout->layerCount()));
      const MapLayer *layer = chunk->layout->layerPtr(layer_index);
      if (expected_size > 0 && layer->voxelByteSize() != expected_size)
      {
        return nullptr;
      }
      // Account for sub sampling.
      const glm::u8vec3 layer_dim = layer->dimensions(map->region_voxel_dimensions);
      // Resolve voxel index within this layer.
      const unsigned index = ::voxelIndex(key, layer_dim);
      T voxels = layer->voxels(*chunk);
      assert(index < unsigned(layer_dim.x * layer_dim.y * layer_dim.z));
      voxels += layer->voxelByteSize() * index;
      return voxels;
    }

    return nullptr;
  }
}  // namespace

namespace ohm
{
  namespace voxel
  {
    uint8_t *voxelPtr(const Key &key, MapChunk *chunk, OccupancyMapDetail *map, int layer_index, size_t expected_size)
    {
      uint8_t *ptr = ::getVoxelBytePtr<uint8_t *>(key, chunk, map, layer_index, expected_size);
      return ptr;
    }

    const uint8_t *voxelPtr(const Key &key, const MapChunk *chunk, OccupancyMapDetail *map, int layer_index,
                            size_t expected_size)
    {
      const uint8_t *ptr = ::getVoxelBytePtr<const uint8_t *>(key, chunk, map, layer_index, expected_size);
      return ptr;
    }


    float occupancyThreshold(const OccupancyMapDetail &map) { return map.occupancy_threshold_value; }


    glm::u8vec3 regionVoxelDimensions(const OccupancyMapDetail &map) { return map.region_voxel_dimensions; }
  }  // namespace voxel
}  // namespace ohm


Voxel VoxelConst::makeMutable() const
{
  // Two approaches here:
  // 1. Find the chunk again in the map has.
  // 2. Const cast the chunk pointer.
  // Whilst 1. is more technically correct, 2. is much faster...
  return Voxel(key_, const_cast<MapChunk *>(chunk_), map_);
}


void Voxel::setValue(float value, bool force)
{
  if (float *voxel_ptr = ohm::voxel::voxelPtrAs<float *>(key_, chunk_, map_, kDlOccupancy))
  {
    if (value != voxel::invalidMarkerValue())
    {
      // Clamp the value to the allowed voxel range.
      value = std::max(map_->min_voxel_value, std::min(value, map_->max_voxel_value));
      // Honour saturation. Once saturated a voxel value could not change.
      if (force || *voxel_ptr == voxel::invalidMarkerValue() ||
          (value < *voxel_ptr && (!map_->saturate_at_max_value || *voxel_ptr < map_->max_voxel_value)) ||
          (value > *voxel_ptr && (!map_->saturate_at_min_value || *voxel_ptr > map_->min_voxel_value)))
      {
        *voxel_ptr = value;
        // This voxel is now valid. Update the chunk's first valid key as required.
        chunk_->updateFirstValid(key_.localKey(), map_->region_voxel_dimensions);
      }
    }
    else
    {
      *voxel_ptr = value;
      // This voxel is now invalid. If it was the first valid voxel, then it no longer is and
      // we need to update it (brute force).
      if (chunk_->first_valid_index == key_.localKey())
      {
        // Search for a new first valid index. We can start from the current first valid value.
        chunk_->searchAndUpdateFirstValid(map_->region_voxel_dimensions, key_.localKey());
      }
#ifdef OHM_VALIDATION
      _chunk->validateFirstValid(_map->region_voxel_dimensions);
#endif  // OHM_VALIDATION
    }
    touchMap();
  }
#ifdef OHM_VALIDATION
  else
  {
    fprintf(stderr, "Attempting to modify null voxel\n");
  }
#endif  // OHM_VALIDATION
}


void Voxel::setClearance(float range)
{
  if (float *voxel_ptr = voxel::voxelPtrAs<float *>(key_, chunk_, map_, kDlClearance))
  {
    *voxel_ptr = range;
    // touchMap();
  }
}


void Voxel::touchRegion(double timestamp)
{
  if (chunk_)
  {
    chunk_->touched_time = timestamp;
  }
}


void Voxel::touchMap()
{
  if (map_ && chunk_)
  {
    ++map_->stamp;
    chunk_->dirty_stamp = chunk_->touched_stamps[kDlOccupancy] = map_->stamp;
  }
}
