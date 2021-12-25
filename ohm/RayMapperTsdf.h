// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef RAYMAPPERTSDF_H
#define RAYMAPPERTSDF_H

#include "OhmConfig.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "RayFilter.h"
#include "RayFlag.h"
#include "RayMapper.h"
#include "Voxel.h"
#include "VoxelTsdf.h"

#include <glm/vec3.hpp>

namespace ohm
{
/// A @c RayMapper implementation built around updating a TSDF map in CPU. This mapper supports TSDF
/// population of @c VoxelTsdf .
///
/// The @c integrateRays() implementation performs a single threaded walk of the voxels to update and touches
/// those voxels one at a time, updating their tsdf values. The given @c OccupancyMap must have a @c VoxelTsdf
/// layer.
class ohm_API RayMapperTsdf : public RayMapper
{
public:
  /// Constructor, wrapping the interface around the given @p map .
  ///
  /// @param map The target map. Must outlive this class.
  explicit RayMapperTsdf(OccupancyMap *map);

  /// Destructor
  ~RayMapperTsdf() override;

  /// Has the map been successfully validated?
  /// @return True if valid and @c integrateRays() is safe to call.
  inline bool valid() const override { return valid_; }

  void setTsdfOptions(const TsdfOptions &options);
  const TsdfOptions &tsdfOptions() const { return tsdf_options_; }

  void setMaxWeight(float max_weight);
  inline float maxWeight() const { return tsdf_options_.max_weight; }
  void setDefaultTruncationDistance(float default_truncation_distance);
  inline float defaultTruncationDistance() const { return tsdf_options_.default_truncation_distance; }
  void setDropoffEpsilon(float dropoff_epsilon);
  inline float dropoffEpsilon() const { return tsdf_options_.dropoff_epsilon; }
  void setSparsityCompensationFactor(float sparsity_compensation_factor);
  inline float sparsityCompensationFactor() const { return tsdf_options_.sparsity_compensation_factor; }

  /// Performs the ray integration.
  ///
  /// This is updated in a single threaded fashion. For each ray we walk the affected voxel @c Key set and
  /// update those voxels. Voxels along each line segment have their tsdf value updated.
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

  using RayMapper::integrateRays;

protected:
  OccupancyMap *map_ = nullptr;      ///< Target map.
  int tsdf_layer_ = -1;              ///< Cached tsdf layer index.
  glm::u8vec3 tsdf_dim_{ 0, 0, 0 };  ///< Cached tsdf layer voxel dimensions. Voxel mean must exactly match.
  TsdfOptions tsdf_options_;         ///< TSDF options.
  bool valid_ = false;               ///< Has layer validation passed?
};

}  // namespace ohm

#endif  // RAYMAPPERTSDF_H
