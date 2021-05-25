// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_RAYSQUERY_H
#define OHM_RAYSQUERY_H

#include "OhmConfig.h"

#include "OccupancyType.h"
#include "Query.h"
#include "QueryFlag.h"

#include <glm/fwd.hpp>

#include <vector>

namespace ohm
{
struct RaysQueryDetail;

/// A query which takes a set of rays and searches for existing obstructions along that ray without modifying the input
/// map.
///
/// The query provides several pieces of information about each ray:
/// - @c ranges() contains the distance until reaching either an occupied voxel, or the end of the ray.
/// - @c unobservedVolumes() is an approximate volumetric accumulation of the voxels traversed by the ray.
/// - @c terminalOccupancyTypes() yields the type of voxel each ray query terminated on.
/// - @c intersectedVoxels() contains the voxel key for each terminating voxel. Contains null keys for filtered rays
///   (@c OccupancyMap::rayFilter()).
///
/// The number of results @c numberOfResults() should match the @c numberOfRays() .
///
/// The @c unobservedVolumes() traces along each ray and accrues the previously unobserved volume that would be observed
/// (assuming no new obstacles are encountered). This is calculated by accruing `(exit_range^3 - enter_range^3)` for
/// each previously unobserved voxel through which the ray passes prior to encountering an occupied voxel. This can be
/// utilised to perform integration in spherical coordinates, where the angular weights and normalisation are applied
/// externally for each ray. The calculation is optionally scaled by @c volumeCoefficient() , which defaults to one.
///
/// Where @c enter_range and @c exit_range are the ranges at which the ray enters and leaves a voxel respectively.
/// This value is accumulated for each unobserved or null voxel.
///
/// Note: on a hard reset, the set of rays is cleared, while a soft reset leaves the ray set unchanged.
///
/// @todo (KS) Get better documentation for the maths behind the @c unobservedVolumes() calculation.
class ohm_API RaysQuery : public Query
{
public:
  /// Default flags to execute this query with.
  static const unsigned kDefaultFlags = kQfNoCache;

protected:
  /// Constructor used for inherited objects. This supports deriving @p LineQueryDetail into
  /// more specialised forms.
  /// @param detail pimple style data structure. When null, a @c LineQueryDetail is allocated by
  /// this method.
  explicit RaysQuery(RaysQueryDetail *detail);

public:
  /// Constructor. The map and rays must be set before using.
  RaysQuery();

  /// Destructor
  ~RaysQuery() override;

  // --- Parameterisation ---

  /// Set the coefficient used in calculating the @c unobservedVolumes() .
  /// See class documentation for detail.
  /// @param coefficient The new coefficient.
  void setVolumeCoefficient(double coefficient);
  /// Get the coefficient used in calculating the @c unobservedVolumes() .
  /// See class documentation for detail.
  /// @return The set coefficient.
  double volumeCoefficient() const;

  /// Set the rays to intersect.
  /// @param rays Origin/end point pairs.
  /// @param element_count Number of elements in @p rays . Expected to be even to account for the origin/end pairing.
  void setRays(const glm::dvec3 *rays, size_t element_count);
  /// Set the rays to intersect.
  /// @param rays Origin/end point pairs. The size is expected to be even to account for the origin/end pairing.
  void setRays(const std::vector<glm::dvec3> &rays);

  /// Add rays to the existing set.
  /// @param rays Origin/end point pairs.
  /// @param element_count Number of elements in @p rays . Expected to be even to account for the origin/end pairing.
  void addRays(const glm::dvec3 *rays, size_t element_count);
  /// Add rays to the existing set.
  /// @param rays Origin/end point pairs. The size is expected to be even to account for the origin/end pairing.
  void addRays(const std::vector<glm::dvec3> &rays);
  /// Add a single ray to the existing set.
  /// @param origin The ray origin.
  /// @param end_point The ray end_point.
  void addRay(const glm::dvec3 &origin, const glm::dvec3 &end_point);

  /// Clear the existing ray set. Also cleared on a hard @c reset(true) .
  void clearRays();

  /// Query the array of query rays.
  const glm::dvec3 *rays(size_t *count = nullptr) const;
  /// Query the number of query rays.
  size_t numberOfRays() const;

  // --- Results ---

  /// An approximation of the volume of previously unobserved space each ray traverses. The number of elements matches
  /// @c numberOfResults() .
  /// @return The unobserved volume traversed by the rays.
  const double *unobservedVolumes() const;

  /// The occupancy type of the voxels at which each ray terminates. This will be @c OccupancyType::kOccupied if an
  /// occupied voxel is encountered along the ray, otherwise it will be the type of the last voxel along the ray. The
  /// number of elements matches @c numberOfResults() .
  /// @return The set of end voxel states for the rays.
  const OccupancyType *terminalOccupancyTypes() const;

protected:
  void onSetMap() override;
  bool onExecute() override;
  bool onExecuteAsync() override;
  void onReset(bool hard_reset) override;

  /// Access internal details.
  /// @return Internal details.
  RaysQueryDetail *imp();
  /// Access internal details.
  /// @return Internal details.
  const RaysQueryDetail *imp() const;
};

inline void RaysQuery::setRays(const std::vector<glm::dvec3> &rays)
{
  return setRays(rays.data(), rays.size());
}

inline void RaysQuery::addRays(const std::vector<glm::dvec3> &rays)
{
  addRays(rays.data(), rays.size());
}
}  // namespace ohm

#endif  // OHM_RAYSQUERY_H
