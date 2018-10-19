//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "Voxel.h"

#include "private/MapLayoutDetail.h"
#include "private/OccupancyMapDetail.h"

#include "MapChunk.h"
#include "DefaultLayers.h"
#include "VoxelLayout.h"

#include <algorithm>
#include <cassert>
#include <cstdio>

using namespace ohm;

namespace
{
  template <typename T, typename MAPCHUNK, typename MAP>
  T *simpleVoxelPtr(const OccupancyKey &key, MAPCHUNK *chunk, MAP *map, int layer_index)
  {
    if (chunk && map)
    {
      // Validate layer.
      assert(layer_index < int(chunk->layout->layerCount()));
      const MapLayer *layer = chunk->layout->layerPtr(layer_index);
      assert(layer->voxelByteSize() == sizeof(T));
      // Account for sub sampling.
      const glm::u8vec3 layer_dim = layer->dimensions(map->region_voxel_dimensions);
      // Resolve node index within this layer.
      const unsigned index = ::voxelIndex(key, layer_dim);
      T *voxels = reinterpret_cast<T *>(layer->voxels(*chunk));
      assert(index < unsigned(layer_dim.x * layer_dim.y * layer_dim.z));
      return &voxels[index];
    }

    return nullptr;
  }
}

float NodeBase::occupancy() const
{
  if (const float *occupancy = simpleVoxelPtr<const float>(key_, chunk_, map_, kDlOccupancy))
  {
    return *occupancy;
  }

  return invalidMarkerValue();
}


float NodeBase::clearance(bool invalid_as_obstructed) const
{
  if (const float *clearance = simpleVoxelPtr<const float>(key_, chunk_, map_, kDlClearance))
  {
    return *clearance;
  }

  return (invalid_as_obstructed) ? 0.0f : -1.0f;
}


double NodeBase::regionTimestamp() const
{
  return (chunk_) ? chunk_->touched_time : 0.0;
}


bool NodeBase::isOccupied() const
{
  const float val = value();
  return !isNull() && val >= map_->occupancy_threshold_value && val != invalidMarkerValue();
}


bool NodeBase::isFree() const
{
  const float val = value();
  return !isNull() && val < map_->occupancy_threshold_value && val != invalidMarkerValue();
}


bool NodeBase::nextInRegion()
{
  if (!chunk_)
  {
    return false;
  }

  const glm::u8vec3 region_dim = map_->region_voxel_dimensions;
  if (key_.localKey().x + 1 == region_dim.x)
  {
    if (key_.localKey().y + 1 == region_dim.y)
    {
      if (key_.localKey().z + 1 == region_dim.z)
      {
        return false;
      }

      key_.setLocalKey(glm::u8vec3(0, 0, key_.localKey().z + 1));
    }
    else
    {
      key_.setLocalKey(glm::u8vec3(0, key_.localKey().y + 1, key_.localKey().z));
    }
  }
  else
  {
    key_.setLocalAxis(0,  key_.localKey().x + 1);
  }

  return true;
}


void OccupancyNode::setValue(float value, bool force)
{
  if (float *voxel_ptr = simpleVoxelPtr<float>(key_, chunk_, map_, kDlOccupancy))
  {
    if (value != invalidMarkerValue())
    {
      // Clamp the value to the allowed node range.
      value = std::max(map_->min_node_value, std::min(value, map_->max_node_value));
      // Honour saturation. Once saturated a node value could not change.
      if (force || *voxel_ptr == invalidMarkerValue() ||
          (value < *voxel_ptr && (!map_->saturate_at_max_value || *voxel_ptr < map_->max_node_value)) ||
          (value > *voxel_ptr && (!map_->saturate_at_min_value || *voxel_ptr > map_->min_node_value)))
      {
        *voxel_ptr = value;
        // This node is now valid. Update the chunk's first valid key as required.
        chunk_->updateFirstValid(key_.localKey(), map_->region_voxel_dimensions);
      }
    }
    else
    {
      *voxel_ptr = value;
      // This node is now invalid. If it was the first valid node, then it no longer is and
      // we need to update it (brute force).
      if (chunk_->first_valid_index == key_.localKey())
      {
        // Search for a new first valid index. We can start from the current first valid value.
        chunk_->searchAndUpdateFirstValid(map_->region_voxel_dimensions, key_.localKey());
      }
#ifdef OHM_VALIDATION
      _chunk->validateFirstValid(_map->region_voxel_dimensions);
#endif // OHM_VALIDATION
    }
    touchMap();
  }
#ifdef OHM_VALIDATION
  else
  {
    fprintf(stderr, "Attempting to modify null node\n");
  }
#endif // OHM_VALIDATION
}


void OccupancyNode::setClearance(float range)
{
  if (float *voxel_ptr = simpleVoxelPtr<float>(key_, chunk_, map_, kDlClearance))
  {
    *voxel_ptr = range;
    // touchMap();
  }
}


void OccupancyNode::touchRegion(double timestamp)
{
  if (chunk_)
  {
    chunk_->touched_time = timestamp;
  }
}


void OccupancyNode::touchMap()
{
  if (map_ && chunk_)
  {
    ++map_->stamp;
    chunk_->dirty_stamp = chunk_->touched_stamps[kDlOccupancy] = map_->stamp;
  }
}


OccupancyNode OccupancyNodeConst::makeMutable() const
{
  return OccupancyNode(key_, chunk_, map_);
}
