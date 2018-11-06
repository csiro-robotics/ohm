// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

namespace ohm
{
  struct HeightmapDetail;
  class OccupancyMap;

  /// A 2D voxel map variation which contains height values in each voxel.
  ///
  /// The heightmap is tied to an @c OccupancyMap from which it builds a heightmap. It never takes ownership of the
  /// @c OccupancyMap.
  ///
  /// @todo Is this a @c Query?
  class Heightmap
  {
  public:
    static const unsigned kDefaultRegionSize = 128;

    /// Construct a new heightmap optionally tied to a specific @p map.
    /// @param grid_resolution The grid resolution for the heightmap.
    /// @param min_clearance The minimum clearance value to accept the lowest voxel.
    /// @param region_size Grid size of each region in the heightmap.
    Heightmap(double grid_resolution, double min_clearance, unsigned region_size = 0);

    /// Destructor.
    ~Heightmap();

    /// Set the occupancy map on which to base the heightmap.
    void setOccupancyMap(OccupancyMap *map);

    /// Access the current occupancy map.
    OccupancyMap *occupancyMap() const;

    /// Access the current heightmap.
    OccupancyMap &heightmap() const;

    void setMinClearance(double clearance);
    double minClearance() const;

    /// The layer number which contains @c HeightmapVoxel structures.
    unsigned heightmapVoxelLayer() const;

    /// Get the current heightmap plane. This is the last plane used to calculate the heightmap in @c update().
    const glm::dvec4 &plane() const;

    /// Update the heightmap using the current @c plane();
    ///
    /// Voxels are project onto this plane to calculate each voxels' closest two clusters to the plane.
    ///
    /// @param plane The heightmap plane. Currently must be horizontal in X/Y.
    /// @return true on success.
    bool update(const glm::dvec4 &plane);

  private:
    std::unique_ptr<HeightmapDetail> imp_;
  };
} // namespace ohm

#endif // HEIGHTMAP_H
