// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef RAYPATTERN_H
#define RAYPATTERN_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

#include <memory>
#include <vector>

namespace ohm
{
struct RayPatternDetail;

/// A @c RayPattern defines a set of ray end points with a common origin. The class may be used to define a custom
/// point set or derived to define a pre-determined pattern.
///
/// The @c RayPattern is intended for use with the @c ClearingPattern utility.
class ohm_API RayPattern
{
protected:
  /// Constructor for sub-classes to derive the detail.
  /// @param detail Custom implementation or null to use the default..
  RayPattern(ohm::RayPatternDetail *detail);

public:
  /// Create an empty ray pattern.
  RayPattern();
  /// Virtual destructor.
  virtual ~RayPattern();

  /// Add a set of points to the pattern. This implicitly adds a ray origin for each @p point at (0, 0, 0).
  /// @param points The array of points to add.
  /// @param point_count Number of elements in @p points.
  void addPoints(const glm::dvec3 *points, size_t point_count);

  /// Add a single point to the pattern.
  /// @param point The new point to add.
  inline void addPoint(const glm::dvec3 &point) { addPoints(&point, 1); }

  /// Add a set of ray start/end point pairs to the pattern.
  /// @param ray_pairs An array of ray start/end point pairs in sensor space.
  /// @param elements Number of elements in @p ray_pairs. Must be even for well defined behaviour.
  void addRays(const glm::dvec3 *ray_pairs, size_t elements);

  /// Add a single ray to the pattern.
  /// @param ray_start Origin of the ray in sensor space.
  /// @param ray_end End point of the ray in sensor space.
  void addRay(const glm::dvec3 &ray_start, const glm::dvec3 &ray_end);

  /// Query the number of rays in the pattern. This is the number of start/end point pairs.
  /// @return The number of ray pairs in the pattern.
  size_t rayCount() const;

  /// Access the ray sample array. This is an array of start/end point pairs (in that order).
  /// @return A pointer to the start of the array of ray pairs in the pattern. There are @c rayCount() pairs
  ///   meaning there are <tt>2 * rayCount()</tt> elements.
  const glm::dvec3 *rayPoints() const;

  /// Build the ray set from the base pattern of points. The @p rays container is populated with pairs of start/end
  /// points which can be used with @c OccupancyMap::intergratePoints(). The first item of every pair is equal to
  /// @p position, while the second is a point from the pattern, rotated by @p rotation and translated by @p position.
  /// @param rays The ray set to populate. Cleared before use.
  /// @param position The translation for the pattern application.
  /// @param rotation The rotation for the pattern application.
  /// @param scaling Optional uniform scaling to apply to the pattern.
  /// @return The number of elements added to @p rays (twice the @c pointCount()).
  size_t buildRays(std::vector<glm::dvec3> *rays, const glm::dvec3 &position, const glm::dquat &rotation,
                   double scaling = 1.0) const;

  /// @overload
  size_t buildRays(std::vector<glm::dvec3> *rays, const glm::dmat4 &pattern_transform) const;

private:
  std::unique_ptr<ohm::RayPatternDetail> imp_;
};
}  // namespace ohm

// 0. Pre-build pattern.
// 1. Position and rotate pattern.

#endif  // RAYPATTERN_H
