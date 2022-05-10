//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef OHM_RAYMAPPERSECONDARYSAMPLE_H
#define OHM_RAYMAPPERSECONDARYSAMPLE_H

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
/// A @c RayMapper implementation built to update the secondary sample voxel layer. Secondary samples are generally
/// used for lidar dual returns.
///
/// This mapper performs no ray tracing, instead only updating the @c VoxelSecondarySample data for each sample voxel.
/// A ray is still required for each voxel in order to calculate the required ranges for the update. The provided rays
/// are expected to originate from the previous/primary voxel to the secondary voxel instead of from the sensor.
/// Intensity and timestamp values are ignored. @c RayFlag values have no effect.
///
/// The @c integrateRays() implementation performs a single threaded walk of just the sample voxels.
class ohm_API RayMapperSecondarySample : public RayMapper
{
public:
  /// Constructor, wrapping the interface around the given @p map .
  ///
  /// @param map The target map. Must outlive this class.
  explicit RayMapperSecondarySample(OccupancyMap *map);

  /// Destructor
  ~RayMapperSecondarySample() override;

  /// Has the map been successfully validated?
  /// @return True if valid and @c integrateRays() is safe to call.
  inline bool valid() const override { return valid_; }

  /// Performs the ray integration.
  ///
  /// This is updated in a single threaded fashion. For each ray we update the sample voxel based on the distance
  /// between the ray origin and sample. Each ray origin is expected to be the location of the primary/previous sample
  /// along the ray, and the second point is the secondary sample location of interest.
  ///
  /// Should only be called if @c valid() is true.
  ///
  /// @param rays The array of start/end or primary/secondary sample pairs to integrate.
  /// @param element_count The number of @c glm::dvec3 elements in @p rays , which is twice the ray count.
  /// @param intensities Ignored.
  /// @param timestamps Ignored.
  /// @param ray_update_flags Ignored.
  /// @return The number of samples processed.
  size_t integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities, const double *timestamps,
                       unsigned ray_update_flags) override;

  using RayMapper::integrateRays;

protected:
  OccupancyMap *map_ = nullptr;       ///< Target map.
  int secondary_samples_layer_ = -1;  ///< Cached secondary samples layer index.
  glm::u8vec3 layer_dim_{ 0, 0, 0 };  ///< Cached layer voxel dimensions.
  bool valid_ = false;                ///< Has layer validation passed?
};

}  // namespace ohm

#endif  // OHM_RAYMAPPERSECONDARYSAMPLE_H
