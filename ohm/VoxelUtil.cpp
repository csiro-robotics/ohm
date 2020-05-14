// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelUtil.h"

#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include "DefaultLayer.h"
#include "Key.h"
#include "MapChunk.h"
#include "VoxelLayout.h"
#include "VoxelMean.h"

using namespace ohm;

namespace
{
  // Get a voxel pointer as uint8_t * or const uint8_t *. No other types for T are supported.
  template <typename T, typename MAPCHUNK>
  T getVoxelBytePtr(const Key &key, MAPCHUNK *chunk, const OccupancyMapDetail *map, int layer_index,
                    size_t expected_size)
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
    uint8_t *voxelPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map, int layer_index,
                      size_t expected_size)
    {
      uint8_t *ptr = ::getVoxelBytePtr<uint8_t *>(key, chunk, map, layer_index, expected_size);
      return ptr;
    }

    const uint8_t *voxelPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map, int layer_index,
                            size_t expected_size)
    {
      const uint8_t *ptr = ::getVoxelBytePtr<const uint8_t *>(key, chunk, map, layer_index, expected_size);
      return ptr;
    }


    const float *voxelOccupancyPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map)
    {
      return voxelPtrAs<const float *>(key, chunk, map, chunk->layout->occupancyLayer());
    }


    float *voxelOccupancyPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map)
    {
      return voxelPtrAs<float *>(key, chunk, map, chunk->layout->occupancyLayer());
    }


    float occupancyThreshold(const OccupancyMapDetail &map) { return map.occupancy_threshold_value; }


    glm::u8vec3 regionVoxelDimensions(const OccupancyMapDetail &map) { return map.region_voxel_dimensions; }


    glm::dvec3 centreLocal(const Key &key, const OccupancyMapDetail &map)
    {
      glm::dvec3 centre;
      // Region centre
      centre = glm::vec3(key.regionKey());
      centre.x *= map.region_spatial_dimensions.x;
      centre.y *= map.region_spatial_dimensions.y;
      centre.z *= map.region_spatial_dimensions.z;
      // Offset to the lower extents of the region.
      centre -= 0.5 * map.region_spatial_dimensions;
      // Local offset.
      centre += glm::dvec3(key.localKey()) * map.resolution;
      centre += glm::dvec3(0.5 * map.resolution);
      return centre;
    }


    glm::dvec3 centreGlobal(const Key &key, const OccupancyMapDetail &map)
    {
      glm::dvec3 centre;
      // Region centre
      centre = glm::dvec3(key.regionKey());
      centre.x *= map.region_spatial_dimensions.x;
      centre.y *= map.region_spatial_dimensions.y;
      centre.z *= map.region_spatial_dimensions.z;
      // Offset to the lower extents of the region.
      centre -= 0.5 * glm::dvec3(map.region_spatial_dimensions);
      // Map offset.
      centre += map.origin;
      // Local offset.
      centre += glm::dvec3(key.localKey()) * double(map.resolution);
      centre += glm::dvec3(0.5 * map.resolution);
      return centre;
    }


    glm::dvec3 position(const Key &key, const MapChunk &chunk, const OccupancyMapDetail &map)
    {
      // Start with the voxel centre.
      glm::dvec3 pos = centreGlobal(key, map);
      // Now resolve the voxel mean value.
      int mean_layer = map.layout.meanLayer();
      if (mean_layer >= 0)
      {
        const VoxelMean *voxel_mean = voxelPtrAs<const VoxelMean *>(key, &chunk, &map, mean_layer);
        if (voxel_mean)
        {
          const glm::dvec3 mean_offset = subVoxelToLocalCoord<glm::dvec3>(voxel_mean->coord, map.resolution);
          pos += mean_offset;
        }
      }
      return pos;
    }
  }  // namespace voxel
}  // namespace ohm
