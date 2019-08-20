// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_CLEARANCEPROCESS_H
#define OHMGPU_CLEARANCEPROCESS_H

#include "OhmGpuConfig.h"

#include <ohm/MappingProcess.h>
#include <ohm/QueryFlag.h>

#include <glm/fwd.hpp>

namespace ohm
{
  struct ClearanceProcessDetail;
  class OccupancyMap;

  /// This query calculates the @c Voxel::clearance() for all voxels within the query extents.
  ///
  /// For each voxel in the query expanse, the range to the nearest obstructed voxel (see @c Query) is calculated and
  /// written back into the voxel's @c clearance value. The range of the calculation is limited by the
  /// @c searchRange() and is not exhaustive.
  ///
  /// The query @c minExtents() and @c maxExtents() loosely limit the zone for which obstacle ranges are calculated.
  /// All voxels from all regions within the extents are calculated. That is, even a partial overlap with a range
  /// results in all voxels for that region being calculated.
  ///
  /// The CPU implementation performs a brute force O(nmm) where n is defined by the number of voxels within all regions
  /// in the search extents, an m is defined by the number of voxels required to reach the @c searchRadius(). The GPU
  /// implementation is closer to worst case O(m) although it incurs additional, initial overhead. The GPU
  /// implementation is recommended over the CPU implementation.
  ///
  /// Both CPU and GPU implementations keep track of which regions have been previously calculated. Results are not
  /// recalculated for a region unless a hard @c reset() is performed.
  ///
  /// Note that the CPU implementation is more accurate than the GPU algorithm. The former directly calculates ranges
  /// to the nearest obstacles, while the latter uses an approximated flood fill. As a result, the GPU algorithm is
  /// optimistic and may report longer ranges than it should.
  ///
  /// The results written to @c Voxel::clearance() should be interpreted as follows:
  /// - 0.0 => The voxel in question is itself an obstruction.
  /// - > 0 => There is an obstructed voxel within the @c searchRadius().
  /// - < 0 => There are no obstructions within the @c searchRadius().
  ///
  /// This query respects the @c QF_UnknownAsOccupied flag, allowing unknown obstacles to be considered as obstacles.
  ///
  /// Note that for this @c Query the following methods are invalid, have different semantics or
  /// have no meaning:
  /// - @c Query::numberOfResults() not used.
  /// - @c Query::intersectedVoxels() contains no data.
  /// - @c Query::ranges() contains no data.
  ///
  /// Results are instead available via @c VoxelConst::clearance() or  @c Voxel::clearance().
  class ohmgpu_API ClearanceProcess : public MappingProcess
  {
  public:
    /// Extended query flags for @c ClearanceProcess.
    enum QueryFlag
    {
      /// Instantiate regions which are in unknown space.
      kQfInstantiateUnknown = (kQfSpecialised << 0),
    };

    /// Empty constructor.
    ClearanceProcess();

    /// Construct a new query using the given parameters.
    ///
    /// This constructor will instantiate a GPU cache in order to aid in the query.
    ///
    /// @param search_radius Defines the search radius around @p nearPoint.
    /// @param query_flags Flags controlling the query behaviour. See @c QueryFlag and @c ClearanceProcess::QueryFlag.
    ClearanceProcess(float search_radius, unsigned query_flags);
    /// Destructor.
    ~ClearanceProcess() override;

    /// Get the search radius to which we look for obstructing voxels.
    /// @return The radius to look for obstacles within.
    float searchRadius() const;
    /// Set the search radius to which we look for obstructing voxels.
    ///
    /// Modifying this value may require a cache reset.
    /// @param range The new search radius.
    void setSearchRadius(float range);

    unsigned queryFlags() const;
    void setQueryFlags(unsigned flags);

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

    void reset() override;

    /// Update the processing queue to process part of the dirty list.
    /// @param map The map to process.
    /// @param time_slice The amount of time available for processing (seconds). Stop if exceeded.
    /// @return See @c MappingProcessResult.
    int update(OccupancyMap &map, double time_slice) override;

    /// Calculate clearance values for all regions within the given extents.
    ///
    /// This is call ignores the processing list, and blocks until the calculations are complete.
    ///
    /// The @p force flag (default @c true) ensures the recalculation is forced regardless of whether
    /// there are existing values or not. The region is still marked as having up to date clearance values.
    ///
    /// @param map The map to process.
    /// @param min_extents The minimum extents corner of the region to calculate.
    /// @param max_extents The maximum extents corner of the region to calculate.
    /// @param force Force recalculation of the clearance values even if they seem up to date.
    ///   This is required if any of the clearance calculation parameters change.
    void calculateForExtents(OccupancyMap &map,  // NOLINT(google-runtime-references)
                             const glm::dvec3 &min_extents, const glm::dvec3 &max_extents, bool force = true);

  protected:
    /// Update clearance for the given region.
    /// @param map The operating map.
    /// @param region_key The key of the region to update.
    /// @param force Force update => update even if not dirty.
    /// @return True if work was done. False if nothing need be done.
    bool updateRegion(OccupancyMap &map,  // NOLINT(google-runtime-references)
                      const glm::i16vec3 &region_key, bool force);

    ClearanceProcessDetail *imp();
    const ClearanceProcessDetail *imp() const;

  private:
    ClearanceProcessDetail *imp_;
  };
}  // namespace ohm

#endif  // OHMGPU_CLEARANCEPROCESS_H
