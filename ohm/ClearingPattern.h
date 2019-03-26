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

#include <glm/glm.hpp>

#include <vector>

namespace ohm
{
  struct ClearingPatternDetail;
  class OccupancyMap;

  /// A helper class for applying a @c RayPattern as a clearing patter to an @c OccupancyMap. The @p apply() method is
  /// templated so that it may also be used to apply a clearing pattern to a @c GpuMap.
  ///
  /// The class is constructed with a @c RayPattern, optionally taking ownership of the pointer. The @c apply() method
  /// is called to generate transformed rays from the @p RayPattern and integrate them into the map as clearing rays.
  /// This means the rays are applied with the following flags: <tt>kRfEndPointAsFree | kRfStopOnFirstOccupied |
  /// kRfClearOnly</tt>.
  ///
  /// This has the effect of having rays only degrade the first occupied voxel struck, then halt traversal. Interveining
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
    /// @param The @c RayPattern.
    const RayPattern *pattern() const;

    /// Query ownership of @p pattern().
    /// @return True if this class owns the @p pattern() memory.
    bool hasPatternOwnership() const;

    /// Apply the clearing patter to the @p map. This clears
    void apply(OccupancyMap *map, const glm::dvec3 &position, const glm::dquat &rotation, double scaling = 1.0);

    /// Apply the clearing patter to the @p map. This clears
    template <typename MAP>
    void apply(MAP *map, const glm::dvec3 &position, const glm::dquat &rotation, double scaling = 1.0);

  private:
    const std::vector<glm::dvec3> &buildRaySet(const glm::dvec3 &position, const glm::dquat &rotation, double scaling);

    std::unique_ptr<ClearingPatternDetail> imp_;
  };

  template <typename MAP>
  void ClearingPattern::apply(MAP *map, const glm::dvec3 &position, const glm::dquat &rotation, double scaling)
  {
    // Reserve memory for the ray set.
    const std::vector<glm::dvec3> &ray_set = buildRaySet(position, rotation, scaling);
    map->integrateRays(ray_set, ray_set.size(), kRfEndPointAsFree | kRfStopOnFirstOccupied | kRfClearOnly);
  }
}  // namespace ohm

#endif  // CLEARINGPATTERN_H
