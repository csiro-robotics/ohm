//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "MapRegion.h"

#include <ohmutil/VectorHash.h>

#include "Key.h"
#include "MapCoord.h"

#ifdef OHM_VALIDATION
#include <cstdio>
#endif  // OHM_VALIDATION

using namespace ohm;

unsigned MapRegion::Hash::calculate(const MapRegion &key)
{
  const glm::i32vec3 hash_coord(key.coord);
  return vhash::hashBits(hash_coord.x, hash_coord.y, hash_coord.z);
}


unsigned MapRegion::Hash::calculate(const glm::i16vec3 &region_coord)
{
  const glm::i32vec3 hash_coord(region_coord);
  return vhash::hashBits(hash_coord.x, hash_coord.y, hash_coord.z);
}


MapRegion::MapRegion(const glm::dvec3 &point, const glm::dvec3 &map_origin, const glm::dvec3 &region_dimensions)
{
  // Quantise.
  coord.x = pointToRegionCoord(double(point.x - map_origin.x), region_dimensions.x);
  coord.y = pointToRegionCoord(double(point.y - map_origin.y), region_dimensions.y);
  coord.z = pointToRegionCoord(double(point.z - map_origin.z), region_dimensions.z);

  // Centre
  centre.x = regionCentreCoord(coord.x, region_dimensions.x);
  centre.y = regionCentreCoord(coord.y, region_dimensions.y);
  centre.z = regionCentreCoord(coord.z, region_dimensions.z);
}


bool MapRegion::voxelKey(Key &key, const glm::dvec3 &point, const glm::dvec3 &map_origin,
                         const glm::dvec3 &region_dimensions, const glm::ivec3 &voxel_counts,
                         double voxel_resolution) const
{
  // Localise.
  const glm::dvec3 region_min = centre - 0.5 * region_dimensions;
  const glm::dvec3 p = point - map_origin - region_min;
  // Quantise.
  const glm::ivec3 q(pointToRegionVoxel(p.x, voxel_resolution, region_dimensions.x),
                     pointToRegionVoxel(p.y, voxel_resolution, region_dimensions.y),
                     pointToRegionVoxel(p.z, voxel_resolution, region_dimensions.z));

  if (0 <= q.x && q.x < voxel_counts.x && 0 <= q.y && q.y < voxel_counts.y && 0 <= q.z && q.z < voxel_counts.z)
  {
    key.setRegionKey(coord);
    key.setLocalKey(q);
    return true;
  }

  // Invalidate the key.
  key = Key::kNull;
  // Out of range.
  return false;
}
