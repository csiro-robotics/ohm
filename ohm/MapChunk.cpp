// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapChunk.h"

#include "DefaultLayer.h"
#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include <cassert>
#include <cstdio>
#include "Voxel.h"

using namespace ohm;

MapChunk::MapChunk(const MapRegion &region, const MapLayout &layout, const glm::uvec3 &region_dim)
{
  this->region = region;
  this->layout = &layout;

  voxel_maps = new uint8_t *[layout.layerCount()];
  touched_stamps = new std::atomic_uint64_t[layout.layerCount()];
  for (size_t i = 0; i < layout.layerCount(); ++i)
  {
    const MapLayer &layer = layout.layer(i);
    voxel_maps[i] = layer.allocate(region_dim);
    layer.clear(voxel_maps[i], region_dim);
    touched_stamps[i] = 0u;
  }
}


MapChunk::MapChunk(const MapLayout &layout, const glm::uvec3 &region_dim)
  : MapChunk(MapRegion(), layout, region_dim)
{}


MapChunk::MapChunk(MapChunk &&other) noexcept
  : region(other.region)
  , layout(other.layout)
  , first_valid_index(other.first_valid_index)
  , touched_time(other.touched_time)
  , dirty_stamp(other.dirty_stamp)
  , touched_stamps(other.touched_stamps)
  , voxel_maps(other.voxel_maps)
  , flags(other.flags)
{
  other.layout = nullptr;
  other.voxel_maps = nullptr;
  other.touched_stamps = nullptr;
}


MapChunk::~MapChunk()
{
  if (layout)
  {
    for (unsigned i = 0; i < layout->layerCount(); ++i)
    {
      layout->layer(i).release(voxel_maps[i]);
    }
  }
  delete[] voxel_maps;
  delete[] touched_stamps;
}


Key MapChunk::keyForIndex(size_t voxel_index, const glm::ivec3 &region_voxel_dimensions,
                                   const glm::i16vec3 &region_coord)
{
  Key key;

  if (voxel_index < unsigned(region_voxel_dimensions.x * region_voxel_dimensions.y * region_voxel_dimensions.z))
  {
    key.setRegionKey(region_coord);

    size_t local_coord = voxel_index % region_voxel_dimensions.x;
    key.setLocalAxis(0, uint8_t(local_coord));
    voxel_index /= region_voxel_dimensions.x;
    local_coord = voxel_index % region_voxel_dimensions.y;
    key.setLocalAxis(1, uint8_t(local_coord));
    voxel_index /= region_voxel_dimensions.y;
    local_coord = voxel_index;
    key.setLocalAxis(2, uint8_t(local_coord));
  }
  else
  {
    key = Key::kNull;
  }

  return key;
}


bool MapChunk::hasValidNodes() const
{
  return first_valid_index.x != 255 && first_valid_index.y != 255 && first_valid_index.z != 255;
}


void MapChunk::updateFirstValid(const glm::u8vec3 &local_index, const glm::ivec3 &region_voxel_dimensions)
{
  const unsigned current_first = voxelIndex(first_valid_index.x, first_valid_index.y, first_valid_index.z, region_voxel_dimensions.x,
                                    region_voxel_dimensions.y, region_voxel_dimensions.z);
  const unsigned test_first = voxelIndex(local_index.x, local_index.y, local_index.z, region_voxel_dimensions.x,
                                 region_voxel_dimensions.y, region_voxel_dimensions.z);
  if (test_first < current_first)
  {
    first_valid_index = local_index;
#ifdef OHM_VALIDATION
    validateFirstValid(regionVoxelDimensions);
#endif  // OHM_VALIDATION
  }
}


void MapChunk::searchAndUpdateFirstValid(const glm::ivec3 &region_voxel_dimensions, const glm::u8vec3 &search_from)
{
  size_t voxel_index;
  first_valid_index = search_from;

  size_t voxel_stride = layout->layer(layout->occupancyLayer()).voxelByteSize();
  const uint8_t *voxel_mem = voxel_maps[layout->occupancyLayer()];

  for (int z = 0; z < region_voxel_dimensions.z; ++z)
  {
    for (int y = 0; y < region_voxel_dimensions.y; ++y)
    {
      for (int x = 0; x < region_voxel_dimensions.x; ++x)
      {
        voxel_index = x + y * region_voxel_dimensions.x + z * region_voxel_dimensions.y * region_voxel_dimensions.x;
        const float occupancy = *reinterpret_cast<const float *>(voxel_mem + voxel_stride * voxel_index);
        if (occupancy != voxel::invalidMarkerValue())
        {
          first_valid_index.x = x;
          first_valid_index.y = y;
          first_valid_index.z = z;
          return;
        }
      }
    }
  }

  // first_valid_index.x = first_valid_index.y = first_valid_index.z = 255u;
}


bool MapChunk::validateFirstValid(const glm::ivec3 &region_voxel_dimensions) const
{
  size_t voxel_index = 0;

  size_t voxel_stride = layout->layer(layout->occupancyLayer()).voxelByteSize();
  const uint8_t *voxel_mem = voxel_maps[layout->occupancyLayer()];

  for (int z = 0; z < region_voxel_dimensions.z; ++z)
  {
    for (int y = 0; y < region_voxel_dimensions.y; ++y)
    {
      for (int x = 0; x < region_voxel_dimensions.x; ++x)
      {
        const float occupancy = *reinterpret_cast<const float *>(voxel_mem + voxel_stride * voxel_index);
        if (occupancy != voxel::invalidMarkerValue())
        {
          if (first_valid_index.x != x || first_valid_index.y != y || first_valid_index.z != z)
          {
            fprintf(stderr, "First valid validation failure. Current: (%d %d %d) actual: (%d %d %d)\n",
                    int(first_valid_index.x), int(first_valid_index.y), int(first_valid_index.z), x, y, z);
            return false;
          }
          return true;
        }
        ++voxel_index;
      }
    }
  }

  // No valid voxels.
  if (first_valid_index.x != 255u || first_valid_index.y != 255u || first_valid_index.z != 255u)
  {
    fprintf(stderr, "First valid validation failure. Current: (%d %d %d) actual: (%d %d %d) [no valid]\n",
            int(first_valid_index.x), int(first_valid_index.y), int(first_valid_index.z), 255, 255, 255);
    return false;
  }

  return true;
}


bool MapChunk::overlapsExtents(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext,
                               const glm::dvec3 &region_spatial_dimensions) const
{
  glm::dvec3 region_min, region_max;
  extents(region_min, region_max, region_spatial_dimensions);

  const bool min_fail = glm::any(glm::greaterThan(region_min, max_ext));
  const bool max_fail = glm::any(glm::greaterThan(min_ext, region_max));

  return !min_fail && !max_fail;
}

void MapChunk::extents(glm::dvec3 &min_ext, glm::dvec3 &max_ext, const glm::dvec3 &region_spatial_dimensions) const
{
  min_ext = max_ext = region.centre;
  min_ext -= 0.5 * region_spatial_dimensions;
  max_ext += 0.5 * region_spatial_dimensions;
}
