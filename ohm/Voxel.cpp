//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "Voxel.h"

#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "VoxelLayout.h"
#include "VoxelMean.h"
#include "VoxelOccupancy.h"

#include <algorithm>
#include <cassert>
#include <cstdio>

using namespace ohm;

namespace
{
  template <typename VOXEL, typename MAPCHUNK, typename MAPDETAIL>
  VOXEL voxelNeighbour(int dx, int dy, int dz, const Key &key, MAPCHUNK *chunk, MAPDETAIL *map)
  {
    if (map == nullptr || key.isNull())
    {
      return VOXEL(key, nullptr, nullptr);
    }

    Key neighbour_key = key;
    map->moveKeyAlongAxis(neighbour_key, 0, dx);
    map->moveKeyAlongAxis(neighbour_key, 1, dy);
    map->moveKeyAlongAxis(neighbour_key, 2, dz);

    if (neighbour_key.regionKey() == key.regionKey())
    {
      // Same region.
      return VOXEL(neighbour_key, chunk, map);
    }

    // Different region. Need to resolve the region.
    std::unique_lock<decltype(map->mutex)> guard(map->mutex);
    const auto region_ref = map->chunks.find(neighbour_key.regionKey());
    if (region_ref != map->chunks.end())
    {
      // Valid neighbouring region.
      return VOXEL(neighbour_key, region_ref->second, map);
    }

    // Invalid neighbouring region.
    return VOXEL(neighbour_key, nullptr, map);
  }
}  // namespace

Voxel VoxelConst::makeMutable() const
{
  // Two approaches here:
  // 1. Find the chunk again in the map has.
  // 2. Const cast the chunk pointer.
  // Whilst 1. is more technically correct, 2. is much faster...
  return Voxel(key_, const_cast<MapChunk *>(chunk_), map_);  // NOLINT(cppcoreguidelines-pro-type-const-cast)
}


void Voxel::setValue(float value, bool force)
{
#ifdef OHM_VALIDATION
  if (!isValid())
  {
    fprintf(stderr, "Attempting to modify null voxel\n");
  }
#endif  // OHM_VALIDATION

  float *occupancy_ptr =
    voxel::voxelOccupancyPtr(key_, chunk_, map_->layout.occupancyLayer(), map_->region_voxel_dimensions);
  const float initial_value = *occupancy_ptr;
  if (value != voxel::invalidMarkerValue())
  {
    if (value >= initial_value || value > 0 && initial_value == voxel::invalidMarkerValue())
    {
      occupancyAdjustUp(
        occupancy_ptr, initial_value, value, voxel::invalidMarkerValue(), map_->max_voxel_value,
        (!force && map_->saturate_at_min_value) ? map_->min_voxel_value : std::numeric_limits<float>::lowest(),
        (!force && map_->saturate_at_max_value) ? map_->max_voxel_value : std::numeric_limits<float>::max(), false);
    }
    else
    {
      occupancyAdjustDown(
        occupancy_ptr, initial_value, value, voxel::invalidMarkerValue(), map_->min_voxel_value,
        (!force && map_->saturate_at_min_value) ? map_->min_voxel_value : std::numeric_limits<float>::lowest(),
        (!force && map_->saturate_at_max_value) ? map_->max_voxel_value : std::numeric_limits<float>::max(), false);
    }
    // This voxel is now valid. Update the chunk's first valid key as required.
    chunk_->updateFirstValid(key_.localKey(), map_->region_voxel_dimensions);
  }
  else
  {
    *occupancy_ptr = value;
  }

  touchMap(map_->layout.occupancyLayer());
}


void Voxel::setUncertain()
{
#ifdef OHM_VALIDATION
  if (!isValid())
  {
    fprintf(stderr, "Attempting to modify null voxel\n");
  }
#endif  // OHM_VALIDATION

  float *occupancy_ptr =
    voxel::voxelOccupancyPtr(key_, chunk_, map_->layout.occupancyLayer(), map_->region_voxel_dimensions);

  *occupancy_ptr = voxel::invalidMarkerValue();
  // This voxel is now invalid. If it was the first valid voxel, then it no longer is and
  // we need to update it (brute force).
  if (chunk_->first_valid_index == voxelIndex(key_.localKey(), map_->region_voxel_dimensions))
  {
    // Search for a new first valid index. We can start from the current first valid value.
    chunk_->searchAndUpdateFirstValid(map_->region_voxel_dimensions, key_.localKey());
  }
#ifdef OHM_VALIDATION
  _chunk->validateFirstValid(_map->region_voxel_dimensions);
#endif  // OHM_VALIDATION
}


void Voxel::setClearance(float range)
{
  if (isValid() && chunk_->layout->clearanceLayer() >= 0)
  {
    if (float *voxel_ptr = voxel::voxelPtrAs<float *>(key_, chunk_, map_, chunk_->layout->clearanceLayer()))
    {
      *voxel_ptr = range;
      touchMap(map_->layout.clearanceLayer());
    }
  }
}


bool Voxel::setPosition(const glm::dvec3 &position, unsigned point_count)
{
  if (isValid())
  {
    if (chunk_->layout->meanLayer() >= 0)
    {
      VoxelMean *voxel_mean = layerContent<VoxelMean *>(map_->layout.meanLayer());
      voxel_mean->coord = subVoxelCoord(position - centreGlobal(), map_->resolution);
      if (point_count)
      {
        voxel_mean->count = point_count;
      }
      touchMap(map_->layout.meanLayer());
      return true;
    }
  }

  return false;
}


bool Voxel::updatePosition(const glm::dvec3 &position)
{
  if (isValid())
  {
    if (chunk_->layout->meanLayer() >= 0)
    {
      VoxelMean *voxel_mean = layerContent<VoxelMean *>(map_->layout.meanLayer());
      voxel_mean->coord =
        subVoxelUpdate(voxel_mean->coord, voxel_mean->count, position - centreGlobal(), map_->resolution);
      ++voxel_mean->count;
      touchMap(map_->layout.meanLayer());
      return true;
    }
  }

  return false;
}


void Voxel::touchRegion(double timestamp)
{
  if (chunk_)
  {
    chunk_->touched_time = timestamp;
  }
}


void Voxel::touchMap(int layer)
{
  chunk_->dirty_stamp = ++map_->stamp;
  chunk_->touched_stamps[layer].store(chunk_->dirty_stamp, std::memory_order_relaxed);
}


Voxel Voxel::neighbour(int dx, int dy, int dz) const
{
  return voxelNeighbour<Voxel>(dx, dy, dz, key_, chunk_, map_);
}


VoxelConst VoxelConst::neighbour(int dx, int dy, int dz) const
{
  return voxelNeighbour<VoxelConst>(dx, dy, dz, key_, chunk_, map_);
}
