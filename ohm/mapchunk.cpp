// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "mapchunk.h"

#include "ohmdefaultlayers.h"
#include "private/maplayoutdetail.h"
#include "private/occupancymapdetail.h"

#include <cassert>
#include <cstdio>

using namespace ohm;

MapChunk::MapChunk(const MapRegion &region, const MapLayout &layout, const glm::uvec3 &regionDim)
{
  this->region = region;
  this->layout = &layout;

  voxelMaps = new uint8_t *[layout.layerCount()];
  touchedStamps = new std::atomic_uint64_t[layout.layerCount()];
  for (size_t i = 0; i < layout.layerCount(); ++i)
  {
    const MapLayer &layer = layout.layer(i);
    voxelMaps[i] = layer.allocate(regionDim);
    layer.clear(voxelMaps[i], regionDim);
    touchedStamps[i] = 0u;
  }
}


MapChunk::MapChunk(const MapLayout &layout, const glm::uvec3 &regionDim)
  : MapChunk(MapRegion(), layout, regionDim)
{}


MapChunk::MapChunk(MapChunk &&other)
  : region(other.region)
  , layout(other.layout)
  , firstValidIndex(other.firstValidIndex)
  , touchedTime(other.touchedTime)
  , dirtyStamp(other.dirtyStamp)
  , touchedStamps(other.touchedStamps)
  , voxelMaps(other.voxelMaps)
  , flags(other.flags)
{
  other.layout = nullptr;
  other.voxelMaps = nullptr;
  other.touchedStamps = nullptr;
}


MapChunk::~MapChunk()
{
  if (layout)
  {
    for (unsigned i = 0; i < layout->layerCount(); ++i)
    {
      layout->layer(i).release(voxelMaps[i]);
    }
  }
  delete[] voxelMaps;
  delete[] touchedStamps;
}


OccupancyKey MapChunk::keyForIndex(size_t voxelIndex, const glm::ivec3 &regionVoxelDimensions,
                                   const glm::i16vec3 &regionCoord)
{
  OccupancyKey key;
  size_t localCoord;

  if (voxelIndex < regionVoxelDimensions.x * regionVoxelDimensions.y * regionVoxelDimensions.z)
  {
    key.setRegionKey(regionCoord);

    localCoord = voxelIndex % regionVoxelDimensions.x;
    key.setLocalAxis(0, (uint8_t)localCoord);
    voxelIndex /= regionVoxelDimensions.x;
    localCoord = voxelIndex % regionVoxelDimensions.y;
    key.setLocalAxis(1, (uint8_t)localCoord);
    voxelIndex /= regionVoxelDimensions.y;
    localCoord = voxelIndex;
    key.setLocalAxis(2, (uint8_t)localCoord);
  }
  else
  {
    key = OccupancyKey::null;
  }

  return key;
}


bool MapChunk::hasValidNodes() const
{
  return firstValidIndex.x != 255 && firstValidIndex.y != 255 && firstValidIndex.z != 255;
}


void MapChunk::updateFirstValid(const glm::u8vec3 &localIndex, const glm::ivec3 &regionVoxelDimensions)
{
  unsigned currentFirst = voxelIndex(firstValidIndex.x, firstValidIndex.y, firstValidIndex.z, regionVoxelDimensions.x,
                                    regionVoxelDimensions.y, regionVoxelDimensions.z);
  unsigned testFirst = voxelIndex(localIndex.x, localIndex.y, localIndex.z, regionVoxelDimensions.x,
                                 regionVoxelDimensions.y, regionVoxelDimensions.z);
  if (testFirst < currentFirst)
  {
    firstValidIndex = localIndex;
#ifdef OHM_VALIDATION
    validateFirstValid(regionVoxelDimensions);
#endif  // OHM_VALIDATION
  }
}


void MapChunk::searchAndUpdateFirstValid(const glm::ivec3 &regionVoxelDimensions, const glm::u8vec3 &searchFrom)
{
  size_t voxelIndex;
  firstValidIndex = searchFrom;

  const float *occupancy = layout->layer(DL_Occupancy).voxelsAs<float>(*this);
  for (int z = 0; z < regionVoxelDimensions.z; ++z)
  {
    for (int y = 0; y < regionVoxelDimensions.y; ++y)
    {
      for (int x = 0; x < regionVoxelDimensions.x; ++x)
      {
        voxelIndex = x + y * regionVoxelDimensions.x + z * regionVoxelDimensions.y * regionVoxelDimensions.x;
        if (occupancy[voxelIndex] != NodeBase::invalidMarkerValue())
        {
          firstValidIndex.x = x;
          firstValidIndex.y = y;
          firstValidIndex.z = z;
          return;
        }
      }
    }
  }

  // firstValidIndex.x = firstValidIndex.y = firstValidIndex.z = 255u;
}


bool MapChunk::validateFirstValid(const glm::ivec3 &regionVoxelDimensions) const
{
  size_t voxelIndex = 0;

  const float *occupancy = layout->layer(DL_Occupancy).voxelsAs<float>(*this);
  for (int z = 0; z < regionVoxelDimensions.z; ++z)
  {
    for (int y = 0; y < regionVoxelDimensions.y; ++y)
    {
      for (int x = 0; x < regionVoxelDimensions.x; ++x)
      {
        if (occupancy[voxelIndex] != NodeBase::invalidMarkerValue())
        {
          if (firstValidIndex.x != x || firstValidIndex.y != y || firstValidIndex.z != z)
          {
            fprintf(stderr, "First valid validation failure. Current: (%d %d %d) actual: (%d %d %d)\n",
                    int(firstValidIndex.x), int(firstValidIndex.y), int(firstValidIndex.z), x, y, z);
            return false;
          }
          return true;
        }
        ++voxelIndex;
      }
    }
  }

  // No valid voxels.
  if (firstValidIndex.x != 255u || firstValidIndex.y != 255u || firstValidIndex.z != 255u)
  {
    fprintf(stderr, "First valid validation failure. Current: (%d %d %d) actual: (%d %d %d) [no valid]\n",
            int(firstValidIndex.x), int(firstValidIndex.y), int(firstValidIndex.z), 255, 255, 255);
    return false;
  }

  return true;
}


bool MapChunk::overlapsExtents(const glm::dvec3 &minExt, const glm::dvec3 &maxExt,
                               const glm::dvec3 &regionSpatialDimensions) const
{
  glm::dvec3 regionMin, regionMax;
  extents(regionMin, regionMax, regionSpatialDimensions);

  const bool minFail = glm::any(glm::greaterThan(regionMin, maxExt));
  const bool maxFail = glm::any(glm::greaterThan(minExt, regionMax));

  return !minFail && !maxFail;
}

void MapChunk::extents(glm::dvec3 &minExt, glm::dvec3 &maxExt, const glm::dvec3 &regionSpatialDimensions) const
{
  minExt = maxExt = region.centre;
  minExt -= 0.5 * regionSpatialDimensions;
  maxExt += 0.5 * regionSpatialDimensions;
}
