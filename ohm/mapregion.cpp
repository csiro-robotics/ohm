//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "mapregion.h"

#include "occupancykey.h"
#include "mapnode.h"
#include <vectorhash.h>

#ifdef OHM_VALIDATION
#include <cstdio>
#endif // OHM_VALIDATION

using namespace ohm;

unsigned MapRegion::Hash::calculate(const MapRegion &key)
{
  const glm::i32vec3 hashCoord(key.coord);
  return vhash::hashBits(hashCoord.x, hashCoord.y, hashCoord.z);
}


unsigned MapRegion::Hash::calculate(const glm::i16vec3 &regionCoord)
{
  const glm::i32vec3 hashCoord(regionCoord);
  return vhash::hashBits(hashCoord.x, hashCoord.y, hashCoord.z);
}


MapRegion::MapRegion(const glm::dvec3 &point, const glm::dvec3 &mapOrigin, const glm::dvec3 &regionDimensions)
{
  // Quantise.
  coord.x = pointToRegionCoord(double(point.x - mapOrigin.x), regionDimensions.x);
  coord.y = pointToRegionCoord(double(point.y - mapOrigin.y), regionDimensions.y);
  coord.z = pointToRegionCoord(double(point.z - mapOrigin.z), regionDimensions.z);

  // Hash.
  hash = Hash::calculate(*this);

  // Centre
  centre.x = regionCentreCoord(coord.x, regionDimensions.x);
  centre.y = regionCentreCoord(coord.y, regionDimensions.y);
  centre.z = regionCentreCoord(coord.z, regionDimensions.z);
}


bool MapRegion::voxelKey(OccupancyKey &key,
                         const glm::dvec3 &point,
                         const glm::dvec3 &mapOrigin,
                         const glm::dvec3 &regionDimensions,
                         const glm::ivec3 &voxelCounts,
                         const double voxelResolution) const
{
  // Localise.
  const glm::dvec3 regionMin = centre - 0.5 * regionDimensions;
  const glm::dvec3 p = point - mapOrigin - regionMin;
  // Quantise.
  const glm::ivec3 q(pointToRegionVoxel(p.x, voxelResolution, regionDimensions.x),
                     pointToRegionVoxel(p.y, voxelResolution, regionDimensions.y),
                     pointToRegionVoxel(p.z, voxelResolution, regionDimensions.z));

  if (0 <= q.x && q.x < voxelCounts.x &&
      0 <= q.y && q.y < voxelCounts.y &&
      0 <= q.z && q.z < voxelCounts.z)
  {
    key.setRegionKey(coord);
    key.setLocalKey(q);
    return true;
  }

  // Invalidate the key.
  key = OccupancyKey::null;
  // Out of range.
  return false;
}
