//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef MAPREGION_H_
#define MAPREGION_H_

#include "ohmconfig.h"

#include <glm/glm.hpp>

namespace ohm
{
  class OccupancyKey;

  /// Represents a large scale spatial region within an @c OccupancyMap.
  ///
  /// This is usually associated with a set of voxels contained in a @c MapChunk.
  struct ohm_API MapRegion
  {
    /// Centre of the map region local to the map origin.
    glm::dvec3 centre;
    /// Quantised integer indexing of the region within the map.
    glm::i16vec3 coord;
    /// A hash of the @p coord value.
    unsigned hash;

    /// Hashing function converting a region key or region coordinates into a 32-bit hash value.
    ///
    /// Supports various data sources.
    struct Hash
    {
      /// Hash a @c MapRegion.
      /// @param key The region to hash.
      /// @return The 32-bit hash for @p key.
      inline unsigned operator()(const MapRegion &key) const { return calculate(key); }
      /// Hash a @c MapRegion.
      /// @param key The region to hash.
      /// @return The 32-bit hash for @p key.
      static unsigned calculate(const MapRegion &key);
      /// Hash quantised interger indexing coordinates for a region.
      /// @param regionCoord The region coordinates to hash.
      /// @return The 32-bit hash for @p key.
      inline unsigned operator()(const glm::i16vec3 &regionCoord) const { return calculate(regionCoord); }
      /// Hash quantised interger indexing coordinates for a region.
      /// @param regionCoord The region coordinates to hash.
      /// @return The 32-bit hash for @p key.
      static unsigned calculate(const glm::i16vec3 &regionCoord);
    };
    friend struct Hash;

    /// Default constructor: member initialisation is undefined.
    MapRegion() = default;

    /// Initialise a region containing the given @p point, based on the given map details.
    /// @param point A spatial point contained by the region.
    /// @param mapOrigin The origin of the map containing the region. The @p point internally is made relative to this origin.
    /// @param regionDimensions Spatial expanse of a region within the map of interest.
    MapRegion(const glm::dvec3 &point, const glm::dvec3 &mapOrigin, const glm::dvec3 &regionDimensions);

    /// Attempt to generate an @c OccupancyKey for a voxel containing @p point if it lies in this region.
    ///
    /// Fails if the @p point does not lie within this region.
    ///
    /// @param[out] key Set to the key for the voxel containing @p point. A null key on failure.
    /// @param point The spatial point for which to generate a voxel key.
    /// @param mapOrigin The origin of the map containing this region. The @p point is internally
    ///   made relative to this origin.
    /// @param regionDimensions Spatial expanse of a region within the map of interest.
    /// @param voxelCounts The number of voxels in each region along each axis.
    /// @param voxelResolution The length of a voxel cube edge.
    /// @return True if the @p point lies within this region and @p key has been set to reference
    ///   the containing voxel. Otherwise @p key is a @c OccupancyMap::null key.
    bool voxelKey(OccupancyKey &key,
                  const glm::dvec3 &point,
                  const glm::dvec3 &mapOrigin,
                  const glm::dvec3 &regionDimensions,
                  const glm::ivec3 &voxelCounts,
                  const double voxelResolution) const;
  };
}

#endif // MAPREGION_H_
