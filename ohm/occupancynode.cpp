//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "occupancynode.h"

#include "private/maplayoutdetail.h"
#include "private/occupancymapdetail.h"

#include "mapchunk.h"
#include "ohmdefaultlayers.h"
#include "ohmvoxellayout.h"

#include <algorithm>
#include <cassert>
#include <cstdio>

using namespace ohm;

namespace
{
  template <typename T, typename MAPCHUNK, typename MAP>
  T *simpleVoxelPtr(const OccupancyKey &key, MAPCHUNK *chunk, MAP *map, int layerIndex)
  {
    if (chunk && map)
    {
      // Validate layer.
      assert(layerIndex < chunk->layout->layerCount());
      const MapLayer *layer = chunk->layout->layerPtr(layerIndex);
      assert(layer->voxelByteSize() == sizeof(T));
      // Account for sub sampling.
      const glm::u8vec3 layerDim = layer->dimensions(map->regionVoxelDimensions);
      // Resolve node index within this layer.
      const unsigned index = ::voxelIndex(key, layerDim);
      const unsigned maxIndex = layerDim.x * layerDim.y * layerDim.z;
      T *voxels = (T *)layer->voxels(*chunk);
      assert(index < maxIndex);
      return &voxels[index];
    }

    return nullptr;
  }
}

float NodeBase::occupancy() const
{
  if (const float *occupancy = simpleVoxelPtr<const float>(_key, _chunk, _map, DL_Occupancy))
  {
    return *occupancy;
  }

  return invalidMarkerValue();
}


float NodeBase::clearance(bool invalidAsObstructed) const
{
  if (const float *clearance = simpleVoxelPtr<const float>(_key, _chunk, _map, DL_Clearance))
  {
    return *clearance;
  }

  return (invalidAsObstructed) ? 0.0f : -1.0f;
}


double NodeBase::regionTimestamp() const
{
  return (_chunk) ? _chunk->touchedTime : 0.0;
}


bool NodeBase::isOccupied() const
{
  const float val = value();
  return !isNull() && val >= _map->occupancyThresholdValue && val != invalidMarkerValue();
}


bool NodeBase::isFree() const
{
  const float val = value();
  return !isNull() && val < _map->occupancyThresholdValue && val != invalidMarkerValue();
}


bool NodeBase::nextInRegion()
{
  if (!_chunk)
  {
    return false;
  }

  const glm::u8vec3 regionDim = _map->regionVoxelDimensions;
  if (_key.localKey().x + 1 == regionDim.x)
  {
    if (_key.localKey().y + 1 == regionDim.y)
    {
      if (_key.localKey().z + 1 == regionDim.z)
      {
        return false;
      }

      _key.setLocalKey(glm::u8vec3(0, 0, _key.localKey().z + 1));
    }
    else
    {
      _key.setLocalKey(glm::u8vec3(0, _key.localKey().y + 1, _key.localKey().z));
    }
  }
  else
  {
    _key.setLocalAxis(0,  _key.localKey().x + 1);
  }

  return true;
}


void OccupancyNode::setValue(float value, bool force)
{
  if (float *voxelPtr = simpleVoxelPtr<float>(_key, _chunk, _map, DL_Occupancy))
  {
    if (value != invalidMarkerValue())
    {
      // Clamp the value to the allowed node range.
      value = std::max(_map->minNodeValue, std::min(value, _map->maxNodeValue));
      // Honour saturation. Once saturated a node value could not change.
      if (force || *voxelPtr == invalidMarkerValue() ||
          (value < *voxelPtr && (!_map->saturateAtMaxValue || *voxelPtr < _map->maxNodeValue)) ||
          (value > *voxelPtr && (!_map->saturateAtMinValue || *voxelPtr > _map->minNodeValue)))
      {
        *voxelPtr = value;
        // This node is now valid. Update the chunk's first valid key as required.
        _chunk->updateFirstValid(_key.localKey(), _map->regionVoxelDimensions);
      }
    }
    else
    {
      *voxelPtr = value;
      // This node is now invalid. If it was the first valid node, then it no longer is and
      // we need to update it (brute force).
      if (_chunk->firstValidIndex == _key.localKey())
      {
        // Search for a new first valid index. We can start from the current first valid value.
        _chunk->searchAndUpdateFirstValid(_map->regionVoxelDimensions, _key.localKey());
      }
#ifdef OHM_VALIDATION
      _chunk->validateFirstValid(_map->regionVoxelDimensions);
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
  if (float *voxelPtr = simpleVoxelPtr<float>(_key, _chunk, _map, DL_Clearance))
  {
    *voxelPtr = range;
    // touchMap();
  }
}


void OccupancyNode::touchRegion(double timestamp)
{
  if (_chunk)
  {
    _chunk->touchedTime = timestamp;
  }
}


void OccupancyNode::touchMap()
{
  if (_map && _chunk)
  {
    ++_map->stamp;
    _chunk->dirtyStamp = _chunk->touchedStamps[DL_Occupancy] = _map->stamp;
  }
}


OccupancyNode OccupancyNodeConst::makeMutable() const
{
  return OccupancyNode(_key, _chunk, _map);
}
