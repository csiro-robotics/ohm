// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CLEARINGPATTERN_H
#define CLEARINGPATTERN_H

#include "OhmConfig.h"

#include "ohm/RayFlag.h"
#include "ohm/RayPattern.h"
#include "ohm/RayMapper.h"

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
  /// This means the rays are applied with the following flags: <tt>kRfEndPointAsFree | kRfStopOnFirstOccupied |
  /// kRfClearOnly</tt>.
  ///
  /// This has the effect of having rays only degrade the first occupied voxel struck, then halt traversal. Intervening
  /// voxels are left unchanged.
  class ClearingPattern
  {
  public:
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
    map->integrateRays(ray_set, unsigned(ray_element_count), kRfEndPointAsFree | kRfStopOnFirstOccupied | kRfClearOnly);
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
    map->integrateRays(ray_set, unsigned(ray_element_count), kRfEndPointAsFree | kRfStopOnFirstOccupied | kRfClearOnly);
    map->setMissValue(initial_miss_value);
  }
}  // namespace ohm

#endif  // CLEARINGPATTERN_H
