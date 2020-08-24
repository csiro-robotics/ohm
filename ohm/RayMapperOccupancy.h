//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPEROCCUPANCY_H
#define RAYMAPPEROCCUPANCY_H

#include "OhmConfig.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "MapCache.h"
#include "RayFilter.h"
#include "RayFlag.h"
#include "RayMapper.h"
#include "Voxel.h"

#include <glm/vec3.hpp>

namespace ohm
{
  /// A @c RayMapper implementation built around updating a map in CPU. This mapper supports basic occupancy population
  /// and @c VoxelMean update (if enabled by the map) - @c MayLayout::occupancyLayer() and @c MapLayout::meanLayer()
  /// respetively.
  ///
  /// The @c integrateRays() implementation performs a single threaded walk of the voxels to update and touches
  /// those voxels one at a time, updating their occupancy value. The given @c OccupancyMap must have an occupancy
  /// occupancy layer and may have a @c VoxelMean layer.
  class RayMapperOccupancy : public RayMapper
  {
  public:
    /// Constructor, wrapping the interface around the given @p map .
    ///
    /// @param map The target map. Must outlive this class.
    RayMapperOccupancy(OccupancyMap *map);

    /// Destructor
    ~RayMapperOccupancy();

    /// Has the map been successfully validated?
    /// @return True if valid and @c integrateRays() is safe to call.
    inline bool valid() const override { return valid_; }

    /// Performs the ray integration.
    ///
    /// This is updated in a single threaded fashion. For each ray we walk the affected voxel @c Key set and
    /// update those voxels. Voxels along each line segment have their occupancy probabilty diminished, while
    /// the end voxel of each segment has the probability increase. The end voxel will also have its @c VoxelMean
    /// updated if the map has a @c MapLayout::meanLayer() . This behaviour may be modified by the @p RayFlag
    /// bits in @p ray_update_flags .
    ///
    /// Should only be called if @c valid() is true.
    ///
    /// @param rays The array of start/end point pairs to integrate.
    /// @param element_count The number of @c glm::dvec3 elements in @p rays, which is twice the ray count.
    /// @param ray_update_flags @c RayFlag bitset used to modify the behaviour of this function. All flags are
    /// implemeted.
    size_t integrateRays(const glm::dvec3 *rays, size_t element_count, unsigned ray_update_flags = kRfDefault) override;

  protected:
    OccupancyMap *map_ = nullptr;           ///< Target map.
    int occupancy_layer_ = -1;              ///< Cached occupancy layer index.
    int mean_layer_ = -1;                   ///< Cached voxel mean layer index.
    glm::u8vec3 occupancy_dim_{ 0, 0, 0 };  ///< Cached occupancy layer voxel dimensions. Voxel mean must exactly match.
    bool valid_ = false;                    ///< Has layer validation passed?
  };

}  // namespace ohm


#endif  // RAYMAPPEROCCUPANCY_H
