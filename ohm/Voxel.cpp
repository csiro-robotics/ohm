//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "Voxel.h"

#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "SubVoxel.h"
#include "VoxelLayout.h"

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
    const auto region_ref = map->findRegion(neighbour_key.regionKey());
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
  return Voxel(key_, const_cast<MapChunk *>(chunk_), map_);
}


void Voxel::setValue(float value, bool force)
{
  float *occupancy_ptr = nullptr;

  if (isValid())
  {
    occupancy_ptr = voxel::voxelOccupancyPtr(key_, chunk_, map_);
  }

  if (occupancy_ptr)
  {
    if (value != voxel::invalidMarkerValue())
    {
      // Clamp the value to the allowed voxel range.
      value = std::max(map_->min_voxel_value, std::min(value, map_->max_voxel_value));
      // Honour saturation. Once saturated a voxel value could not change.
      if (force || *occupancy_ptr == voxel::invalidMarkerValue() ||
          (value < *occupancy_ptr && (!map_->saturate_at_max_value || *occupancy_ptr < map_->max_voxel_value)) ||
          (value > *occupancy_ptr && (!map_->saturate_at_min_value || *occupancy_ptr > map_->min_voxel_value)))
      {
        *occupancy_ptr = value;
        // This voxel is now valid. Update the chunk's first valid key as required.
        chunk_->updateFirstValid(key_.localKey(), map_->region_voxel_dimensions);
      }
    }
    else
    {
      *occupancy_ptr = value;
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
    touchMap(chunk_->layout->occupancyLayer());
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
  if (isValid() && chunk_->layout->clearanceLayer() >= 0)
  {
    if (float *voxel_ptr = voxel::voxelPtrAs<float *>(key_, chunk_, map_, chunk_->layout->clearanceLayer()))
    {
      *voxel_ptr = range;
      touchMap(chunk_->layout->clearanceLayer());
    }
  }
}


bool Voxel::setPosition(const glm::dvec3 &position)
{
  if (isValid())
  {
    if (chunk_->layout->hasSubVoxelPattern())
    {
      OccupancyVoxel *voxel_occupancy = layerContent<OccupancyVoxel *>(map_->layout.occupancyLayer());
      voxel_occupancy->sub_voxel = subVoxelCoord(position - centreGlobal(), map_->resolution);

      return true;
    }
  }

  return false;
}


bool Voxel::updatePosition(const glm::dvec3 &position, double update_weighting)
{
  if (isValid())
  {
    if (chunk_->layout->hasSubVoxelPattern())
    {
      update_weighting = (update_weighting >= 0) ? update_weighting : map_->sub_voxel_weighting;
      OccupancyVoxel *voxel_occupancy = layerContent<OccupancyVoxel *>(map_->layout.occupancyLayer());
      voxel_occupancy->sub_voxel =
        subVoxelUpdate(voxel_occupancy->sub_voxel, position - centreGlobal(), map_->resolution, update_weighting);
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
  if (map_ && chunk_)
  {
    assert(0 <= layer && layer < chunk_->layout->layerCount());
    ++map_->stamp;
    chunk_->dirty_stamp = chunk_->touched_stamps[layer] = map_->stamp;
  }
}


Voxel Voxel::neighbour(int dx, int dy, int dz) const
{
  return voxelNeighbour<Voxel>(dx, dy, dz, key_, chunk_, map_);
}


VoxelConst VoxelConst::neighbour(int dx, int dy, int dz) const
{
  return voxelNeighbour<VoxelConst>(dx, dy, dz, key_, chunk_, map_);
}
