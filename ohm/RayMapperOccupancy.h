//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPEROCCUPANCY_H
#define RAYMAPPEROCCUPANCY_H

#include "OhmConfig.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "RayFilter.h"
#include "RayFlag.h"
#include "RayMapper.h"
#include "Voxel.h"

#include <glm/vec3.hpp>

namespace ohm
{
/// A @c RayMapper implementation built around updating a map in CPU. This mapper supports basic occupancy population
/// and @c VoxelMean update (if enabled by the map) - @c MayLayout::occupancyLayer() and @c MapLayout::meanLayer()
/// respectively.
///
/// The @c integrateRays() implementation performs a single threaded walk of the voxels to update and touches
/// those voxels one at a time, updating their occupancy value. The given @c OccupancyMap must have an occupancy
/// layer and may have a @c VoxelMean layer.
class ohm_API RayMapperOccupancy : public RayMapper
{
public:
  /// Constructor, wrapping the interface around the given @p map .
  ///
  /// @param map The target map. Must outlive this class.
  explicit RayMapperOccupancy(OccupancyMap *map);

  /// Destructor
  ~RayMapperOccupancy() override;

  /// Has the map been successfully validated?
  /// @return True if valid and @c integrateRays() is safe to call.
  inline bool valid() const override { return valid_; }

  /// Performs the ray integration.
  ///
  /// This is updated in a single threaded fashion. For each ray we walk the affected voxel @c Key set and
  /// update those voxels. Voxels along each line segment have their occupancy probability diminished, while
  /// the end voxel of each segment has the probability increase. The end voxel will also have its @c VoxelMean
  /// updated if the map has a @c MapLayout::meanLayer() . This behaviour may be modified by the @p RayFlag
  /// bits in @p ray_update_flags .
  ///
  /// Should only be called if @c valid() is true.
  ///
  /// @param rays The array of start/end point pairs to integrate.
  /// @param element_count The number of @c glm::dvec3 elements in @p rays , which is twice the ray count.
  /// @param intensities An array of intensity values matching the @p rays items. There is one intensity value per ray
  ///   so there are @c element_count/2 items. May be null to omit intensity values.
  /// @param timestamps An array of timestamp values matching the @p rays items. There is one timestamp value per ray
  ///   so there are @c element_count/2 items. May be null to omit timestamp values in which case the touch time layer
  ///   will not be updated.
  /// @param ray_update_flags @c RayFlag bitset used to modify the behaviour of this function. All flags are
  /// implemented.
  size_t integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities, const double *timestamps,
                       unsigned ray_update_flags) override;

  /// Lookup the given @p rays in the map. The @p rays form a list of origin/sample pairs, where the sample represents
  /// the maximum range to scan.
  ///
  /// @deprecated This functionality can now be found in @c RaysQuery .
  ///
  /// @param rays Array of origin/sample point pairs.
  /// @param element_count The number of points in @p rays. The ray count is half this value.
  /// @param[out] newly_observed_volumes Array to be populated with previously unobserved volume observed in each ray.
  /// Optional; pass in nullptr if not required. Length element_count/2.
  /// @param[out] ranges Array to be populated with range traced for each ray. Will be length of origin sample/pair
  /// unless an occupied voxel was encountered. Optional; pass in nullptr if not required. Length element_count/2.
  /// @param[out] terminal_states Array to be populated with the state of the final voxel traced. Optional; pass in
  /// nullptr if not required. Length element_count/2.
  size_t lookupRays(const glm::dvec3 *rays, size_t element_count, float *newly_observed_volumes, float *ranges,
                    OccupancyType *terminal_states);

  using RayMapper::integrateRays;

protected:
  OccupancyMap *map_ = nullptr;           ///< Target map.
  int occupancy_layer_ = -1;              ///< Cached occupancy layer index.
  int mean_layer_ = -1;                   ///< Cached voxel mean layer index.
  int traversal_layer_ = -1;              ///< The traversal layer index.
  int touch_time_layer_ = -1;             ///< Cache touch time layer index.
  int incident_normal_layer_ = -1;        ///< Cache incident normal layer index.
  glm::u8vec3 occupancy_dim_{ 0, 0, 0 };  ///< Cached occupancy layer voxel dimensions. Voxel mean must exactly match.
  bool valid_ = false;                    ///< Has layer validation passed?
};

}  // namespace ohm


#endif  // RAYMAPPEROCCUPANCY_H
