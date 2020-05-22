//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPERBASE_H
#define RAYMAPPERBASE_H

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
    /// @param rays The array of start/end point pairs to integrate.
    /// @param element_count The number of @c glm::dvec3 elements in @p rays, which is twice the ray count.
    /// @param ray_update_flags @c RayFlag bitset used to modify the behaviour of this function.
    virtual size_t integrateRays(const glm::dvec3 *rays, size_t element_count,
                                 unsigned ray_update_flags = kRfDefault) = 0;
  };
}  // namespace ohm


#endif  // RAYMAPPERBASE_H
