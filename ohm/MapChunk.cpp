// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapChunk.h"

#include "DefaultLayer.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "VoxelBuffer.h"
#include "VoxelOccupancy.h"

#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include <cassert>
#include <cstdio>

namespace ohm
{
MapChunk::MapChunk(const MapRegion &region, const OccupancyMapDetail &map)
{
  this->region = region;
  this->map = &map;

  const MapLayout &layout = this->layout();
  voxel_blocks.resize(layout.layerCount());
  touched_stamps = std::make_unique<std::atomic_uint64_t[]>(layout.layerCount());  // NOLINT(modernize-avoid-c-arrays)
  for (size_t i = 0; i < layout.layerCount(); ++i)
  {
    const MapLayer &layer = layout.layer(i);
    voxel_blocks[i].reset(new VoxelBlock(&map, layer));
    touched_stamps[i] = 0u;
  }
}


MapChunk::MapChunk(const OccupancyMapDetail &map)
  : MapChunk(MapRegion(), map)
{}


MapChunk::MapChunk(MapChunk &&other) noexcept
  : region(std::exchange(other.region, MapRegion()))
  , map(std::exchange(other.map, nullptr))
  , first_valid_index(std::exchange(other.first_valid_index, ~0u))
  , touched_time(std::exchange(other.touched_time, 0))
  , dirty_stamp(other.dirty_stamp.load())
  , touched_stamps(std::move(other.touched_stamps))
  , voxel_blocks(std::move(other.voxel_blocks))
  , flags(std::exchange(other.flags, 0))
{}


MapChunk::~MapChunk() = default;


const MapLayout &MapChunk::layout() const
{
  return map->layout;
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


void MapChunk::updateLayout(const MapLayout *new_layout,
                            const std::vector<std::pair<const MapLayer *, const MapLayer *>> &preserve_layer_mapping)
{
  // Allocate voxel pointer array.
  std::vector<VoxelBlock::Ptr> new_voxel_blocks(new_layout->layerCount());
  std::unique_ptr<std::atomic_uint64_t[]> new_touched_stamps =           // NOLINT(modernize-avoid-c-arrays)
    std::make_unique<std::atomic_uint64_t[]>(new_layout->layerCount());  // NOLINT(modernize-avoid-c-arrays)

  for (const auto &mapping : preserve_layer_mapping)
  {
    new_voxel_blocks[mapping.second->layerIndex()].swap(voxel_blocks[mapping.first->layerIndex()]);
    new_touched_stamps[mapping.second->layerIndex()] = touched_stamps[mapping.first->layerIndex()].load();
    // Memory ownership moved: nullify to prevent release.
    voxel_blocks[mapping.first->layerIndex()] = nullptr;
  }

  // Now initialise any new or unmapped layers and release those not preserved.
  for (size_t i = 0; i < new_layout->layerCount(); ++i)
  {
    const MapLayer &layer = new_layout->layer(i);
    if (!new_voxel_blocks[i])
    {
      // Initilised layer.
      new_voxel_blocks[i].reset(new VoxelBlock(map, layer));
      new_touched_stamps[i] = 0u;
    }
    else
    {
      // Preserving layer. Ensure it's index is updated.
      new_voxel_blocks[i]->updateLayerIndex(layer.layerIndex());
    }
  }

  // Release redundant layers.
  const MapLayout &old_layout = layout();
  for (size_t i = 0; i < old_layout.layerCount(); ++i)
  {
    if (voxel_blocks[i])
    {
      // Unmigrated/redudant layer. Release.
      voxel_blocks[i]->destroy();
      voxel_blocks[i] = nullptr;
    }
  }

  // Update pointers
  std::swap(voxel_blocks, new_voxel_blocks);
  std::swap(touched_stamps, new_touched_stamps);
  // We do nothing to update the layout() to new_layout. This object is owned by the occupancy map which we assume is
  // about to change internally. It's address will remain unchanged.
}


void MapChunk::searchAndUpdateFirstValid(const glm::ivec3 &region_voxel_dimensions, const glm::u8vec3 &search_from)
{
  const MapLayout &layout = this->layout();
  VoxelBuffer<const VoxelBlock> voxel_buffer(voxel_blocks[layout.occupancyLayer()]);
  const size_t voxel_stride = layout.layer(layout.occupancyLayer()).voxelByteSize();
  const uint8_t *voxel_mem = voxel_buffer.voxelMemory();

  unsigned voxel_index;
  float occupancy;
  for (int z = search_from.z; z < region_voxel_dimensions.z; ++z)
  {
    for (int y = search_from.y; y < region_voxel_dimensions.y; ++y)
    {
      for (int x = search_from.x; x < region_voxel_dimensions.x; ++x)
      {
        voxel_index =
          unsigned(x) + y * region_voxel_dimensions.x + z * region_voxel_dimensions.y * region_voxel_dimensions.x;
        memcpy(&occupancy, voxel_mem + voxel_stride * voxel_index, sizeof(occupancy));
        if (occupancy != unobservedOccupancyValue())
        {
          first_valid_index = voxel_index;
          return;
        }
      }
    }
  }

  // Failed to find a valid item (at least from search_from). Mark as unknown.
  first_valid_index = ~0u;
}


glm::u8vec3 MapChunk::firstValidKey() const
{
  return firstValidKey(map->region_voxel_dimensions);
}


bool MapChunk::validateFirstValid() const
{
  const MapLayout &layout = this->layout();
  VoxelBuffer<const VoxelBlock> voxel_buffer(voxel_blocks[layout.occupancyLayer()].get());
  const size_t voxel_stride = layout.layer(layout.occupancyLayer()).voxelByteSize();
  const uint8_t *voxel_mem = voxel_buffer.voxelMemory();

  unsigned voxel_index = 0;
  float occupancy;
  for (int z = 0; z < map->region_voxel_dimensions.z; ++z)
  {
    for (int y = 0; y < map->region_voxel_dimensions.y; ++y)
    {
      for (int x = 0; x < map->region_voxel_dimensions.x; ++x)
      {
        memcpy(&occupancy, voxel_mem + voxel_stride * voxel_index, sizeof(occupancy));
        if (occupancy != unobservedOccupancyValue())
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


bool MapChunk::overlapsExtents(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext) const
{
  glm::dvec3 region_min;
  glm::dvec3 region_max;
  extents(region_min, region_max);

  const bool min_fail = glm::any(glm::greaterThan(region_min, max_ext));
  const bool max_fail = glm::any(glm::greaterThan(min_ext, region_max));

  return !min_fail && !max_fail;
}

void MapChunk::extents(glm::dvec3 &min_ext, glm::dvec3 &max_ext) const
{
  min_ext = max_ext = region.centre;
  min_ext -= 0.5 * map->region_spatial_dimensions;
  max_ext += 0.5 * map->region_spatial_dimensions;
}
}  // namespace ohm
