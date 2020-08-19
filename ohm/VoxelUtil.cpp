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

#include <type_traits>

using namespace ohm;

namespace
{
  // Get a voxel pointer as uint8_t * or const uint8_t *. No other types for T are supported.
  template <typename T, typename MAPCHUNK>
  T getVoxelBytePtr(const Key &key, MAPCHUNK *chunk, int layer_index, const glm::u8vec3 &layer_dim,
                    size_t voxel_byte_stride)
  {
    static_assert(sizeof(typename std::remove_pointer<T>::type) == 1, "Only byte types are supported");
    // Resolve voxel index within this layer.
    const unsigned index = ::voxelIndex(key, layer_dim);
    T voxels = chunk->voxel_maps[layer_index];
    voxels += voxel_byte_stride * index;
    return voxels;
  }
}  // namespace

namespace ohm
{
  namespace voxel
  {
    uint8_t *voxelPtr(const Key &key, MapChunk *chunk, int layer_index, const glm::u8vec3 &layer_dim,
                      size_t voxel_byte_stride)
    {
      return ::getVoxelBytePtr<uint8_t *>(key, chunk, layer_index, layer_dim, voxel_byte_stride);
    }

    uint8_t *voxelPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map, int layer_index,
                      size_t expected_size)
    {
      const MapLayer &layer = map->layout.layer(layer_index);
      if (expected_size == layer.voxelByteSize())
      {
        return ::getVoxelBytePtr<uint8_t *>(key, chunk, layer.layerIndex(),
                                            layer.dimensions(map->region_voxel_dimensions), expected_size);
      }
      return nullptr;
    }

    uint8_t *voxelPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map, const MapLayer &layer,
                      size_t expected_size)
    {
      if (expected_size == layer.voxelByteSize())
      {
        return ::getVoxelBytePtr<uint8_t *>(key, chunk, layer.layerIndex(),
                                            layer.dimensions(map->region_voxel_dimensions), expected_size);
      }
      return nullptr;
    }

    const uint8_t *voxelPtr(const Key &key, const MapChunk *chunk, int layer_index, const glm::u8vec3 &layer_dim,
                            size_t voxel_byte_stride)
    {
      return ::getVoxelBytePtr<const uint8_t *>(key, chunk, layer_index, layer_dim, voxel_byte_stride);
    }

    const uint8_t *voxelPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map, int layer_index,
                            size_t expected_size)
    {
      const MapLayer &layer = map->layout.layer(layer_index);
      if (expected_size == layer.voxelByteSize())
      {
        return ::getVoxelBytePtr<const uint8_t *>(key, chunk, layer.layerIndex(),
                                                  layer.dimensions(map->region_voxel_dimensions), expected_size);
      }
      return nullptr;
    }

    const uint8_t *voxelPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map, const MapLayer &layer,
                            size_t expected_size)
    {
      if (expected_size == layer.voxelByteSize())
      {
        return ::getVoxelBytePtr<const uint8_t *>(key, chunk, layer.layerIndex(),
                                                  layer.dimensions(map->region_voxel_dimensions), expected_size);
      }
      return nullptr;
    }


    const float *voxelOccupancyPtr(const Key &key, const MapChunk *chunk, int occupancy_layer,
                                   const glm::u8vec3 &layer_dim)
    {
      return reinterpret_cast<const float *>(voxelPtr(key, chunk, occupancy_layer, layer_dim, sizeof(float)));
    }


    float *voxelOccupancyPtr(const Key &key, MapChunk *chunk, int occupancy_layer, const glm::u8vec3 &layer_dim)
    {
      return reinterpret_cast<float *>(voxelPtr(key, chunk, occupancy_layer, layer_dim, sizeof(float)));
    }


    const float *voxelOccupancyPtr(const Key &key, const MapChunk *chunk, const OccupancyMapDetail *map)
    {
      return reinterpret_cast<const float *>(
        voxelPtr(key, chunk, map->layout.occupancyLayer(), map->region_voxel_dimensions, sizeof(float)));
    }


    float *voxelOccupancyPtr(const Key &key, MapChunk *chunk, const OccupancyMapDetail *map)
    {
      return reinterpret_cast<float *>(
        voxelPtr(key, chunk, map->layout.occupancyLayer(), map->region_voxel_dimensions, sizeof(float)));
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
