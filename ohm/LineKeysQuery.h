// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_LINEKEYSQUERY_H
#define OHM_LINEKEYSQUERY_H

#include "OhmConfig.h"

#include "Query.h"
#include "QueryFlag.h"

#include <glm/fwd.hpp>

namespace ohm
{
  struct LineKeysQueryDetail;

  /// This query calculates the voxels intersecting a batch of rays.
  ///
  /// The results are similar to those of @c OccupancyMap::calculateSegmentKeys() (identical when using CPU),
  /// but supports batched and GPU based calculation. The GPU calculation is generally only marginally faster
  /// than CPU, but GPU supports @c executeAsync(), which is not supported for CPU.
  ///
  /// In practice, this query isn't very useful as the GPU performance gains are minimal.
  ///
  /// General usage is:
  /// - Initialise the query object setting the map and the GPU flag if required.
  /// - Call @c setRays() to define the ray start/end point pairs.
  /// - Call @c execute() or @c executeAsync() followed by @c wait() (GPU only).
  /// - Process results (see below).
  ///
  /// The @c numberOfResults() will match the number of rays (@c pointCount given to @c setRays()) and for
  /// each of these results, there is an entry in @c resultIndices() and @c resultCounts(). The indices indicate the
  /// offsets into @c intersectedVoxels() for each ray in the same order they appear in @c setRays(). The
  /// counts identify how many voxels are present for the current line counting from the associated index.
  /// There should always be at least one voxel per ray for the start/end voxel. More generally, the first voxel
  /// is the ray start voxel and the last voxel is the ray end voxel.
  class ohm_API LineKeysQuery : public Query
  {
  protected:
    /// Constructor used for inherited objects. This supports deriving @p LineKeysQueryDetail into
    /// more specialised forms.
    /// @param detail pimple style data structure. When null, a @c LineKeysQueryDetail is allocated by
    /// this method.
    LineKeysQuery(LineKeysQueryDetail *detail);

  public:
    /// Construct a new query using the given parameters.
    /// @param map The map to operate on. Only the voxel resolution and region sizes are used.
    /// @param query_flags Flags controlling the query behaviour. See @c QueryFlag and @c LineKeysQuery::Flag.
    LineKeysQuery(ohm::OccupancyMap &map, unsigned query_flags = 0u);

    /// Construct a new query using the given parameters.
    /// @param query_flags Flags controlling the query behaviour. See @c QueryFlag and @c LineKeysQuery::Flag.
    LineKeysQuery(unsigned query_flags = 0);

    /// Destructor.
    ~LineKeysQuery();

    /// Set the ray point pairs to operate on. The @p rays elements must be in start/end point pairs.
    /// @param rays Ray start/end point pairs.
    /// @param point_count Number of elements in @p rays. Expected to be an even number; i.e., twice the number of rays.
    void setRays(const glm::dvec3 *rays, size_t point_count);

    /// Get the array of ray points set in the last call to @c setRays().
    /// @return The last ray set assigned in @c setRays(). The number of elements is @c rayPointCount().
    const glm::dvec3 *rays() const;

    /// Return the number of point elements in @p rays(). The number of rays is half this as ray points are in
    /// start/end point pairs.
    /// @return The number of elements in @p rays().
    size_t rayPointCount() const;

    /// Get the array of result index offsets into @c intersectecVoxels().
    ///
    /// This identifies the offsets for each ray into @c intersectedVoxels() where the results for that ray begin.
    /// The corresponding number of voxels for each ray are accessible via @c resultCounts(). The number of elements
    /// is set by the number of rays, accessible via @c numberOfResults().
    ///
    /// Only value once execution completes.
    ///
    /// @return Index offsets into @p intersectedVoxels() for each ray.
    const size_t *resultIndices() const;

    /// Get the array of result voxel counts in @c intersectecVoxels() for each ray.
    ///
    /// This identifies the number of voxels for each ray in @c intersectedVoxels(). The corresponding offset for
    /// each ray into @c intersectedVoxels() is available via @c resultIndices(). The number of elements
    /// is set by the number of rays, accessible via @c numberOfResults().
    ///
    /// Only value once execution completes.
    ///
    /// @return Number of voxels intersected for each ray.
    const size_t *resultCounts() const;

  protected:
    bool onExecute() override;
    bool onExecuteAsync() override;
    void onReset(bool hard_reset) override;

    LineKeysQueryDetail *imp();
    const LineKeysQueryDetail *imp() const;
  };
}  // namespace ohm


#endif  // OHM_LINEKEYSQUERY_H
