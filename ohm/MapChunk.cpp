// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapChunk.h"

#include "DefaultLayer.h"
#include "Voxel.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include <cassert>
#include <cstdio>

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


void MapChunk::updateLayout(const MapLayout *new_layout, const glm::uvec3 &region_dim,
                            const std::vector<std::pair<const MapLayer *, const MapLayer *>> &preserve_layer_mapping)
{
  // Allocate voxel pointer array.
  uint8_t **new_voxel_maps = new uint8_t *[new_layout->layerCount()];
  std::atomic_uint64_t *new_touched_stamps = new std::atomic_uint64_t[new_layout->layerCount()];

  // Initialise voxel maps to null so we can track what's missed by preserve_layer_mapping.
  for (size_t i = 0; i < new_layout->layerCount(); ++i)
  {
    new_voxel_maps[i] = nullptr;
  }

  for (const auto &mapping : preserve_layer_mapping)
  {
    new_voxel_maps[mapping.second->layerIndex()] = voxel_maps[mapping.first->layerIndex()];
    new_touched_stamps[mapping.second->layerIndex()] = touched_stamps[mapping.first->layerIndex()].load();
    // Memory ownership moved: nullify to prevent release.
    voxel_maps[mapping.first->layerIndex()] = nullptr;
  }

  // Now initialise any new or unmapped layers and release those not preserved.
  for (size_t i = 0; i < new_layout->layerCount(); ++i)
  {
    if (!new_voxel_maps[i])
    {
      // Initilised layer.
      const MapLayer &layer = new_layout->layer(i);
      new_voxel_maps[i] = layer.allocate(region_dim);
      layer.clear(new_voxel_maps[i], region_dim);
      new_touched_stamps[i] = 0u;
    }
  }

  // Release redundant layers.
  for (size_t i = 0; i < layout->layerCount(); ++i)
  {
    if (voxel_maps[i])
    {
      // Unmigrated/redudant layer. Release.
      layout->layer(i).release(voxel_maps[i]);
      voxel_maps[i] = nullptr;
    }
  }

  // Release pointer arrays.
  delete[] voxel_maps;
  delete[] touched_stamps;

  // Update pointers
  voxel_maps = new_voxel_maps;
  touched_stamps = new_touched_stamps;
  // We do not update the layout pointer to new_layout. This pointer is to the owning occupancy map's layout which we
  // assume is about to change internally. It's address will remain unchanged.
}


void MapChunk::searchAndUpdateFirstValid(const glm::ivec3 &region_voxel_dimensions, const glm::u8vec3 &search_from)
{
  unsigned voxel_index;

  size_t voxel_stride = layout->layer(layout->occupancyLayer()).voxelByteSize();
  const uint8_t *voxel_mem = voxel_maps[layout->occupancyLayer()];

  for (int z = 0; z < region_voxel_dimensions.z; ++z)
  {
    for (int y = 0; y < region_voxel_dimensions.y; ++y)
    {
      for (int x = 0; x < region_voxel_dimensions.x; ++x)
      {
        voxel_index =
          unsigned(x) + y * region_voxel_dimensions.x + z * region_voxel_dimensions.y * region_voxel_dimensions.x;
        const float occupancy = *reinterpret_cast<const float *>(voxel_mem + voxel_stride * voxel_index);
        if (occupancy != voxel::invalidMarkerValue())
        {
          first_valid_index = voxel_index;
          return;
        }
      }
    }
  }

  // first_valid_index = ~0u;
}


bool MapChunk::validateFirstValid(const glm::ivec3 &region_voxel_dimensions) const
{
  unsigned voxel_index = 0;

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
          if (first_valid_index != voxel_index)
          {
            fprintf(stderr, "First valid validation failure. Current: (%d) actual: (%d)\n", int(first_valid_index),
                    int(voxel_index));
            return false;
          }
          return true;
        }
        ++voxel_index;
      }
    }
  }

  // No valid voxels.
  if (first_valid_index != ~0u)
  {
    fprintf(stderr, "First valid validation failure. Current: (%d) actual: (%d)\n", int(first_valid_index),
            int(voxel_index));
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
