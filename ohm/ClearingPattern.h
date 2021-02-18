// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CLEARINGPATTERN_H
#define CLEARINGPATTERN_H

#include "OhmConfig.h"

#include "ohm/RayFlag.h"
#include "ohm/RayMapper.h"
#include "ohm/RayPattern.h"

#include <glm/glm.hpp>

#include <vector>

namespace ohm
{
struct ClearingPatternDetail;
class OccupancyMap;

/// A helper class for applying a @c RayPattern as a clearing pattern to an @c OccupancyMap. The @p apply() method is
/// templated so that it may also be used to apply a clearing pattern to a @c GpuMap.
///
/// The class is constructed with a @c RayPattern, optionally taking ownership of the pointer. The @c apply() method
/// is called to generate transformed rays from the @p RayPattern and integrate them into the map as clearing rays.
/// The rays are applied using the configure @c rayFlag() bit set, which defaults to @c kDefaultRayFlags. The flags
/// are designed to effect rays which only degrade the first occupied voxel struck, then halt traversal. Intervening
/// voxels are left unchanged.
///
/// Technically the @c rayFlags() can be modified to generate different behaviour even non-clearing behaviour.
///
/// @todo Rename this class to @c RayPattern and configure a selection of default flags including @c kClearingFlags
/// for the current default behaviour.
class ClearingPattern
{
public:
  /// Default flags used for a clearing pattern. The flags are set to ensure rays only affect occupied voxels
  /// (`kRfExcludeFree | kRfExcludeUnobserved`), reducing their occupancy and we stop on the first occupied voxel
  /// (`kRfStopOnFirstOccupied`) and the voxel at the end of the ray is treated as a clear voxel rather than a sample
  /// voxel (`kRfEndPointAsFree`).
  static const unsigned kDefaultRayFlags =
    kRfEndPointAsFree | kRfStopOnFirstOccupied | kRfExcludeFree | kRfExcludeUnobserved;

  /// Create a clearing pattern.
  /// @param pattern The ray pattern to use.
  /// @param take_ownership Should the @c ClearingPattern own the @p pattern and delete the pointer when done?
  ClearingPattern(const RayPattern *pattern, bool take_ownership);

  /// Destructor: deletes @c pattern() if it @c hasPatternOwnership().
  ~ClearingPattern();

  /// Retrieve the actual pattern.
  /// @return The @c RayPattern.
  const RayPattern *pattern() const;

  /// Query ownership of @p pattern().
  /// @return True if this class owns the @p pattern() memory.
  bool hasPatternOwnership() const;

  /// Query the combination of @c RayFlag bits to be used when applying this pattern. Defaults to @c kDefaultRayFlags.
  /// @return The configured @c RayFlag bit set.
  unsigned rayFlags() const;

  /// Set the combination of @c RayFlag bits to be used when applying this pattern. Note that patterns which deviate
  /// from the @c kDefaultRayFlags can result in significantly different semantics which invalidate the name
  /// "ClearingPattern" as it is used in this context. For example, clearing the flag @c kRfEndPointAsFree can have
  /// the "ClearingPattern" generate occupied voxels. As such, this function should be used with care.
  /// @param ray_flags Modified @c RayFlag bit set to use when applying this pattern.
  void setRayFlags(unsigned ray_flags);

  /// Apply the clearing @c pattern() to @p map. This supports both APIs for both @c OccupancyMap and the @p GpuMap
  /// extension.
  ///
  /// This @c pattern() rays are transformed by the given @p position and @p rotation before being integrated
  /// into the @p map. Each ray reduced the probabilty of the first occupied voxel it encounters but no further.
  ///
  /// @param map The map to integrate the clearing pattern into.
  /// @param position Used to the origin of the rays in this clearing pattern.
  /// @param rotation Rotates the ray end points in this clearing pattern.
  /// @param probability_scaling Scaling factor applied to probability values.
  template <typename MAP>
  void apply(MAP *map, const glm::dvec3 &position, const glm::dquat &rotation, float probability_scaling = 1.0);

  /// @overload
  template <typename MAP>
  void apply(MAP *map, const glm::dmat4 &pattern_transform, float probability_scaling = 1.0);

  /// Query the last ray set used in @c apply . This is the pattern transformed to match the position and rotation
  /// last supplied to @c apply() . Contains start/end pairs.
  /// @param[out] element_count Set to the number of elements in the returned value. The number of rays is half this.
  /// @return A pointer to the ray set.
  const glm::dvec3 *lastRaySet(size_t *element_count) const;

private:
  const glm::dvec3 *buildRaySet(size_t *element_count, const glm::dvec3 &position, const glm::dquat &rotation);

  const glm::dvec3 *buildRaySet(size_t *element_count, const glm::dmat4 &pattern_transform);

  std::unique_ptr<ClearingPatternDetail> imp_;
};

template <typename MAP>
void ClearingPattern::apply(MAP *map, const glm::dvec3 &position, const glm::dquat &rotation, float probability_scaling)
{
  size_t ray_element_count = 0u;
  const glm::dvec3 *ray_set = buildRaySet(&ray_element_count, position, rotation);
  const float initial_miss_value = map->missValue();
  map->setMissValue(initial_miss_value * probability_scaling);
  map->integrateRays(ray_set, unsigned(ray_element_count), rayFlags());
  map->setMissValue(initial_miss_value);
}

template <typename MAP>
void ClearingPattern::apply(MAP *map, const glm::dmat4 &pattern_transform, float probability_scaling)
{
  // Reserve memory for the ray set.
  size_t ray_element_count = 0u;
  const glm::dvec3 *ray_set = buildRaySet(&ray_element_count, pattern_transform);
  const float initial_miss_value = map->missValue();
  map->setMissValue(initial_miss_value * probability_scaling);
  map->integrateRays(ray_set, unsigned(ray_element_count), rayFlags());
  map->setMissValue(initial_miss_value);
}
}  // namespace ohm

#endif  // CLEARINGPATTERN_H
