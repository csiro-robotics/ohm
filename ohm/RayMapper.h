//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPER_H
#define RAYMAPPER_H

#include "OhmConfig.h"

#include "RayFlag.h"

#include <glm/fwd.hpp>

namespace ohm
{
class OccupancyMap;
class KeyList;

/// A @c RayMapper serves to provide a unified interface for integrating rays into an @c OccupancyMap .
///
/// This interfaces solely serves to define the @p integrateRays() function.
class RayMapper
{
public:
  /// Default constructor.
  RayMapper();
  /// Default, virtual destructor.
  virtual ~RayMapper();

  /// Has the map been successfully validated?
  /// @return True if validated and @c integrateRays() is safe to call.
  virtual bool valid() const = 0;

  /// This function integrates the set of @p rays into the underlying map interface. The details of how this is
  /// achieved is entirely up to the dervied implementation. Some examples include probabilistic occupancy update
  /// only, probably with update with voxel mean, occupancy update with covariance using a normal distribution
  /// transform or GPU implementations of the previous methods
  ///
  /// The @p rays array is expected to consist of ray origin, ray end point pairs. These correlate with a range sensor
  /// position and sample point pair. The @p element_count identifies the number of @c glm::dvec3 elements in @p rays
  /// thus is expected to be an even number with the ray count being `element_count / 2`.
  ///
  /// The @p ray_update_flags are used to modify the behaviour as par @c RayFlag values. However, not all
  /// implementations will respect all flags.
  ///
  /// Should only be called if @c validated() is true.
  ///
  /// @param rays The array of start/end point pairs to integrate.
  /// @param element_count The number of @c glm::dvec3 elements in @p rays, which is twice the ray count.
  /// @param intensities An array of intensity values matching the @p rays items. There is one intensity value per ray
  ///   so there are @c element_count/2 items. May be null to omit intensity values.
  /// @param timestamps An array of timestamp values matching the @p rays items. There is one timestamp value per ray
  ///   so there are @c element_count/2 items. May be null to omit timestamp values in which case the touch time layer
  ///   will not be updated.
  /// @param ray_update_flags @c RayFlag bitset used to modify the behaviour of this function.
  virtual size_t integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                               const double *timestamps, unsigned ray_update_flags) = 0;

  /// @overload
  virtual inline size_t integrateRays(const glm::dvec3 *rays, size_t element_count)
  {
    return integrateRays(rays, element_count, nullptr, nullptr, kRfDefault);
  }
};
}  // namespace ohm


#endif  // RAYMAPPER_H
