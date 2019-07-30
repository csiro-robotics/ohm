// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_LINEQUERY_H
#define OHM_LINEQUERY_H

#include "OhmConfig.h"

#include "Query.h"
#include "QueryFlag.h"

#include <glm/fwd.hpp>

namespace ohm
{
  class GpuMap;
  struct LineQueryDetail;

  /// A line segment intersection query for an @c OccupancyMap.
  ///
  /// A line segment query determines the range of voxels for which intersect a given line segment and for each such
  /// voxels, calculates the range to the nearest obstructed voxel. Obstructed voxles are defined as occupied voxels
  /// and optionally unknown voxels when the query flag @c QF_UnknownAsOccupied is set.
  ///
  /// The results are given in @c intersectedVoxels() and @c ranges(). The @c intersectedVoxels() identify the voxels
  /// touched by the line segment, in order, from start to end. The corresponding @c ranges() yield the nearest
  /// obstacle range for each voxel in @c intersectedVoxels(), or -1 if there is no voxel within the search range.
  /// Setting the flag @c QF_NearestResult modifies the results to only report one voxel which has the shortest
  /// range value (excluding unobstructed voxels). The result set may be empty in the case where @c QF_NearestResult is
  /// set and all voxels along the line have no obstructions within the search range.
  ///
  /// The CPU implementation is O(nmm) where n is the number of voxels in the line and m is defined by search radius
  /// divided by the map resolution (i.e., the number of voxels required to reach the search radius). The GPU
  /// implementation is closer to worst case O(m) although it incurs additional, initial overhead. The GPU
  /// implementation supports caching to offset this overhead.
  ///
  /// Using the GPU implementation, the @c LineQuery invokes the @c VoxelRanges query in order to cache the obstacle
  /// ranges. On a soft @c reset(), the obstacle ranges are only recalculated for regions which have not yet
  /// been calculated. A hard @c reset() should be invoked whenever the map changes to recalculate these ranges.
  ///
  /// Note that a @c VoxelRanges query pointer may be given to the constructor to use an external @c VoxelRanges
  /// query object. In this case this object does not take ownership of the external @c VoxelRanges query object.
  ///
  /// @todo Add a flag which will early out of completing the query when the line enters a region which contains only
  /// uncertain voxels (or does not exist). This is to help avoid creating those regions when using GPU queries
  /// based on the assumption that we are not interested in continuing the query when it is wholly obstructed.
  ///
  /// @bug Update the comment without reference to defunct VoxelRange, instead noting that ClearanceProcess is
  /// required and precalculated clearance values may be assumed in GPU mode. searchRadius() is ingored in this
  /// mode.
  class ohm_API LineQuery : public Query
  {
  public:
    /// Default flags to execute this query with.
    static const unsigned kDefaultFlags = kQfNoCache;

  protected:
    /// Constructor used for inherited objects. This supports deriving @p LineQueryDetail into
    /// more specialised forms.
    /// @param detail pimple style data structure. When null, a @c LineQueryDetail is allocated by
    /// this method.
    LineQuery(LineQueryDetail *detail);

  public:
    LineQuery();

    /// Construct a new query using the given parameters.
    /// @param map The map to perform the query on.
    /// @param start_point The global coordinate marking the start of the line segment.
    /// @param end_point The global coordinate marking the end of the line segment.
    /// @param search_radius Defines the "width" of the line. See @c searchRadius().
    /// @param query_flags Flags controlling the query behaviour. See @c QueryFlag and @c LineQuery::Flag.
    LineQuery(OccupancyMap &map,  // NOLINT(google-runtime-references)
              const glm::dvec3 &start_point, const glm::dvec3 &end_point, float search_radius,
              unsigned query_flags = kDefaultFlags);

    /// Destructor.
    ~LineQuery() override;

    /// Get the query's line segment start point.
    /// @return The start point in global coordinates.
    glm::dvec3 startPoint() const;
    /// Set the query's line segment start point.
    /// @param point The start point in global coordinates.
    void setStartPoint(const glm::dvec3 &point);

    /// Get the query's line segment end point.
    /// @return The end point in global coordinates.
    glm::dvec3 endPoint() const;
    /// Set the query's line segment end point.
    /// @param point The end point in global coordinates.
    void setEndPoint(const glm::dvec3 &point);

    /// Get the search radius around the line segment.
    /// @return The search radius around the line segment used by the query.
    float searchRadius() const;
    /// Set the search radius around the line segment. This effectively defines a width for the line segment.
    /// All reported results are within this distance of the line segment.
    /// @param radius The new search radius.
    void setSearchRadius(float radius);

    /// Get the range value used when there are no obstructions within range.
    /// @return The range value reported for voxels with no obstructions in range.
    /// @see setDefaultRangeValue()
    float defaultRangeValue() const;
    /// Set the range value used when there are no obstructions for a voxel within the @c searchRadius().
    /// The default is -1.
    ///
    /// Note that setting this value to be smaller than the @c searchRadius() is ambiguous.
    /// @param range The range to report for voxels with no obstructions within range.
    void setDefaultRangeValue(float range);

    /// Get the axis weightings applied when determining the nearest obstructing voxel.
    /// @return Current axis weighting.
    /// @see @c setAxisScaling()
    glm::vec3 axisScaling() const;

    /// Set the per axis scaling applied when determining the closest obstructing voxel.
    ///
    /// This vector effectively distorts the map using the following scaling matrix:
    /// @code{.unparsed}
    ///   axisScaling.x   0               0
    ///   0               axisScaling.y   0
    ///   0               0               axisScaling.z
    /// @endcode
    ///
    /// The default scaling of (1, 1, 1) has no overall impact, meaning that the closest obstructing voxel is
    /// selected. Scaling of (1, 1, 2) scales the Z axis by a factor of two, making the Z axis half as important as
    /// either the X or Y axes.
    ///
    /// Results are scaled by calculating the distance between two voxels as show in the following pseudo code:
    /// @code
    ///   separation = nearestObstacleCentre - voxelCentre;
    ///   separation.x *= 1.0f * axisScaling.x;
    ///   separation.y *= 1.0f * axisScaling.y;
    ///   separation.z *= 1.0f * axisScaling.z;
    ///   float range = length(separation)
    /// @endcode
    ///
    /// Note a zero value causes the scaling on that axis to be zero, making all voxels on that axis logically,
    /// equidistant. This may yield undefined resuts. Negative values are not recommended, but will effectively be
    /// equivalent to their absolute value.
    ///
    /// Modifying this value will require a cache reset (@c reset(true)) to recalculate ranges.
    ///
    /// @param scaling The new axis scaling to apply.
    void setAxisScaling(const glm::vec3 &scaling);

  protected:
    bool onExecute() override;
    bool onExecuteAsync() override;
    void onReset(bool hard_reset) override;

    LineQueryDetail *imp();
    const LineQueryDetail *imp() const;
  };
}  // namespace ohm

#endif  // OHM_LINEQUERY_H
