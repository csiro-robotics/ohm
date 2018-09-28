// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYNEARESTNEIGHBOURS_H_
#define OCCUPANCYNEARESTNEIGHBOURS_H_

#include "ohmconfig.h"

#include "occupancyquery.h"

#include <glm/fwd.hpp>

namespace ohm
{
  struct NearestNeighboursDetail;

  /// A nearest neighbours query for an @c OccupancyMap.
  ///
  /// This finds obstructed voxels within a fixed radius of a known point; i.e., obstructed voxels intersecting the
  /// query sphere. The reported @c intersectedVoxels() contains all obstructed voxels within the search sphere while
  /// the corresponding @c ranges() identify the distance from the query centre to the centre of each relevant voxel.
  /// The order of results is undefined and may change between calls.
  ///
  /// Setting the flag @c QF_NearestResult modifies the results such than only one voxel is reported and this is the
  /// voxel closest to the search centre. Again, which voxel this is may be change between calls when multiple voxels
  /// report exactly the same range.
  ///
  /// The query only tests for intersections between the query sphere and the centre of nearby voxels. This means that
  /// specifying a small search radius (~ the voxel resolution) may consistently yield zero results.
  ///
  /// A GPU implementation is supported for this query, however it is inferior to the CPU implementation in two ways:
  /// - The CPU implementation is usually faster.
  /// - The GPU implementation is too memory intensive and may result in a crash/SEGFAULT.
  class ohm_API NearestNeighbours : public Query
  {
  protected:
    /// Constructor used for inherited objects. This supports deriving @p NearestNeighboursDetail into
    /// more specialised forms.
    /// @param detail pimple style data structure. When null, a @c NearestNeighboursDetail is allocated by
    /// this method.
    NearestNeighbours(NearestNeighboursDetail *detail = nullptr);

  public:
    /// Construct a new query using the given parameters.
    /// @param map The map to perform the query on.
    /// @param nearPoint The global coordinate to search around.
    /// @param searchRadius Defines the search radius around @p nearPoint.
    /// @param queryFlags Flags controlling the query behaviour. See @c QueryFlag and @c LineQuery::Flag.
    NearestNeighbours(OccupancyMap &map, const glm::dvec3 &nearPoint,
                      float searchRadius, unsigned queryFlags);
    /// Destructor.
    ~NearestNeighbours();

    /// Get the global coordinate around which the search is centred.
    /// @return The centre of the search.
    glm::dvec3 nearPoint() const;
    /// Set the global coordinate around which to search.
    /// @param point The new search coordinate.
    void setNearPoint(const glm::dvec3 &point);

    /// Get the search radius around @p nearPoint().
    /// @return The search radius around the search centre.
    float searchRadius() const;
    /// Set the search radius around the search centre.
    /// @param radius The new search radius.
    void setSearchRadius(float range);

  protected:
    void onSetMap() override;
    bool onExecute() override;
    bool onExecuteAsync() override;
    void onReset(bool hardReset) override;

    NearestNeighboursDetail *imp();
    const NearestNeighboursDetail *imp() const;
  };
}

#endif // OCCUPANCYNEARESTNEIGHBOURS_H_
