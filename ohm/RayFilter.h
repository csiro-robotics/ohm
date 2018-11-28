// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef RAYFILTER_H
#define RAYFILTER_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

#include <functional>

namespace ohm
{
  class Aabb;


  /// Flags used with @c RayFilterFunction.
  enum RayFilterFlag
  {
    /// Ray is invalid: should be skipped.
    kRffInvalid = (1 << 0),
    /// Sample ray start point has been clipped (and moved).
    kRffClippedStart = (1 << 1),
    /// Sample ray end point has been clipped (and moved).
    kRffClippedEnd = (1 << 2),
  };

  /// Function prototype used in ray filtering for @c OccupancyMap::integrateRays().
  /// @param start A pointer to the sample ray start coordinate. May be modified.
  /// @param end A pointer to the sample ray end coordinate. May be modified.
  /// @param filter_flags A pointer to the flags. May be added to using flags from @c RayFilterFlag.
  /// @return True if the sample ray may be processed, false to skip this sample.
  typedef std::function<bool (glm::dvec3 *, glm::dvec3 *, unsigned *)> RayFilterFunction;

  /// A helper function to validate a sample ray. Fails on rays which have start or end point as infinite or NaN or
  /// if the ray length exceeds @p max_range (skipped if @p max_range <= 0).
  /// @param start The sample ray start coordinate.
  /// @param end The sample ray end coordinate.
  /// @param max_range Optional maximum range limit for the ray.
  /// @return True if the ray is ok to be included.
  bool ohm_API goodRay(const glm::dvec3 &start, const glm::dvec3 &end, double max_range = 0);

  /// A wrapper around @c goodRay() for use with @c RayFilterFunction.
  /// On failure, @c filter_flags has the @c kRffInvalid flag set.
  ///
  /// This is the default ray filter applied.
  ///
  /// @param start A pointer to the sample ray start coordinate. May be modified.
  /// @param end A pointer to the sample ray end coordinate. May be modified.
  /// @param filter_flags A pointer to the flags. May be added to using flags from @c RayFilterFlag.
  /// @param max_range Optional maximum range limit for the ray.
  /// @return True if the sample ray may be processed, false to skip this sample.
  bool ohm_API goodRayFilter(glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags, double max_range);

  /// A @c RayFilterFunction which clips rays a bounding box.
  ///
  /// May set the folling bits of @p filter_flags:
  /// - @c kRffClippedStart if @p start is modified.
  /// - @c kRffClippedEnd if @p end is modified.
  ///
  /// @param start A pointer to the sample ray start coordinate. May be modified.
  /// @param end A pointer to the sample ray end coordinate. May be modified.
  /// @param filter_flags A pointer to the flags. May be added to using flags from @c RayFilterFlag.
  /// @param clip_box The axis aligned box to clip rays to.
  /// @return True if the sample ray may be processed, false to skip this sample.
  bool ohm_API clipBounded(glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags, const ohm::Aabb &clip_box);

  /// A @c RayFilterFunction which test sample end points ensuring they do not lie within @c clip_box
  ///
  /// May set the folling bits of @p filter_flags:
  /// - @c kRffClippedEnd if @p start falls within @p clip_box.
  ///
  /// Never modifies @p start or @p end and never returns false.
  ///
  /// @param start A pointer to the sample ray start coordinate. May be modified.
  /// @param end A pointer to the sample ray end coordinate. May be modified.
  /// @param filter_flags A pointer to the flags. May be added to using flags from @c RayFilterFlag.
  /// @param max_range Optional maximum range limit for the ray.
  /// @return True
  bool ohm_API clipNear(glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags, const ohm::Aabb &clip_box);

} // namespace ohm

#endif // RAYFILTER_H
