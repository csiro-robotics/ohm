// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELMEAN_H
#define VOXELMEAN_H

#include "OhmConfig.h"

#include "MapCoord.h"
#include "Voxel.h"
#include "VoxelOccupancy.h"

#include <glm/vec3.hpp>

/// @defgroup voxelmean Voxel Mean Position
/// These functions are used to manipulate the voxel mean positioning voxel fields. @c VoxelMean allows a voxel to store
/// a position field offset from the centre of the voxel, thereby refining the precision of the voxel position.
///
/// To support voxel mean positioning, an @c OccupancyMap must first enable this feature by the constructor flag, or
/// by calling @c OccupancyMap::setVoxelMeanEnabled(true). This ensures a voxel layer is added to the map containing
/// to hold the @c VoxelMean data. This structure contains a quantised voxel mean coordinate pattern and a point count.
/// The voxel mean coordinate pattern is defined as follows:
/// - bits [0, 9]: voxel mean x coord
/// - bits [10, 19]: voxel mean y coord
/// - bits [20, 29]: voxel mean z coord
/// - bit 30: unused
/// - bit 31: marks voxel mean data as present
///
/// XYZ coordinates are assigned 10 bits each to store a quantised voxel mean coordinate. This coordinate is quantised
/// accross the range of 10-bits allowing for 1024 possible positions along each axis. If no voxel mean pattern is
/// present, then the voxel centre is given as the voxel position.
///
/// When samples are integrated into a voxel, the voxel mean position is calculated via a progressive average
/// combining the existing coordinate and the new sample value.
/// This is given as:
/// @code{.unparsed}
///   new_mean_coord = current_mean_coord + (sample - current_mean_coord) / (point_count + 1)
/// @endcode

namespace ohm
{
#include "VoxelMeanCompute.h"

  /// @internal
  template <typename V>
  inline glm::dvec3 positionUnsafeT(const Voxel<V> &voxel)
  {
    const VoxelMean &mean_info = voxel.data();
    glm::dvec3 mean = voxel.map()->voxelCentreGlobal(voxel.key());
    mean += subVoxelToLocalCoord<glm::dvec3>(mean_info.coord, voxel.map()->resolution());
    return mean;
  }

  /// @ingroup voxelmean
  /// Calculate the mean position within the given @p voxel .
  ///
  /// @c Voxel<VoxelMean>::isValid() must be true before calling.
  /// @param voxel The voxel to query the mean coordinate for.
  inline glm::dvec3 positionUnsafe(const Voxel<VoxelMean> &voxel) { return positionUnsafeT(voxel); }

  /// @ingroup voxelmean
  /// Calculate the mean position within the given @p voxel .
  ///
  /// `Voxel<const VoxelMean>::isValid()` must be true before calling.
  /// @param voxel The voxel to query the mean coordinate for.
  inline glm::dvec3 positionUnsafe(const Voxel<const VoxelMean> &voxel) { return positionUnsafeT(voxel); }

  /// @internal
  template <typename V>
  inline glm::dvec3 positionSafeT(const Voxel<V> &voxel)
  {
    if (voxel.isValid())
    {
      return positionUnsafeT(voxel);
    }

    if (!voxel.key().isNull() && voxel.map())
    {
      return voxel.map()->voxelCentreGlobal(voxel.key());
    }
    return glm::dvec3{ 0 };
  }

  /// Query the position of @c voxel if the @p voxel might be invalid or the voxel layer might be invalid.
  ///
  /// In order of availability, the retun value will be:
  /// - The @c VoxelMean position - @p voxel must be fully valid.
  /// - The voxel centre of the @p voxel.key() - @p voxel map and key must be valid.
  /// - `(0, 0, 0)`
  /// @param voxel The voxel to query.
  /// @return The best available coordinate for the voxel (see remarks).
  inline glm::dvec3 positionSafe(const Voxel<VoxelMean> &voxel) { return positionSafeT(voxel); }
  /// @overload
  inline glm::dvec3 positionSafe(const Voxel<const VoxelMean> &voxel) { return positionSafeT(voxel); }

  /// @ingroup voxelmean
  /// Explicitly set the mean position for @p voxel .
  ///
  /// `Voxel<const VoxelMean>::isValid()` must be true before calling.
  ///
  /// @param voxel The voxel to modify.
  /// @param pos The new mean position for @p voxel . Must be within the bounds of @p voxel .
  /// @param count Optional argument to explicitly set the number of samples gather to attain the position.
  ///   This affects subsequent mean updates, with larger values reducing the influence of new positions.
  ///   Use 0 to leave the current count value as is.
  inline void setPositionUnsafe(Voxel<VoxelMean> &voxel, const glm::dvec3 &pos, unsigned count = 0)
  {
    VoxelMean &mean_info = voxel.data();
    mean_info.coord = subVoxelCoord(pos - voxel.map()->voxelCentreGlobal(voxel.key()), voxel.map()->resolution());
    mean_info.count = (count) ? count : mean_info.count;
  }

  /// @ingroup voxelmean
  /// A validated version of @c setPositionUnsafe() ensuring @c voxel is valid before attepting to write.
  /// @param voxel The voxel to modify. May be in invalid.
  /// @param pos The new mean position for @p voxel . Must be within the bounds of @p voxel .
  /// @param count Optional argument to explicitly set the number of samples gathered to attain the position.
  ///   This affects subsequent mean updates, with larger values reducing the influence of new positions.
  ///   Use 0 to leave the current count value as is.
  inline void setPositionSafe(Voxel<VoxelMean> &voxel, const glm::dvec3 &pos, unsigned count = 0)
  {
    if (voxel.isValid())
    {
      setPositionUnsafe(voxel, pos, count);
    }
  }

  /// @ingroup voxelmean
  /// Update the mean position for a voxel, adjusting the mean with the new coordinate @p pos .
  ///
  /// `Voxel<const VoxelMean>::isValid()` must be true before calling.
  ///
  /// @param voxel The voxel to modify.
  /// @param pos The new coordinate to incorporate into the mean.
  inline void updatePositionUnsafe(Voxel<VoxelMean> &voxel, const glm::dvec3 &pos)
  {
    VoxelMean &mean_info = voxel.data();
    mean_info.coord = subVoxelUpdate(mean_info.coord, mean_info.count,
                                     pos - voxel.map()->voxelCentreGlobal(voxel.key()), voxel.map()->resolution());
    mean_info.count = std::min(mean_info.count, std::numeric_limits<unsigned>::max() - 1u) + 1;
  }

  /// @ingroup voxelmean
  /// A validated version of @c updatePositionUnsafe() ensuring @c voxel is valid before attepting to write.
  /// @param voxel The voxel to modify. May be invalid.
  /// @param pos The new coordinate to incorporate into the mean.
  inline void updatePositionSafe(Voxel<VoxelMean> &voxel, const glm::dvec3 &pos)
  {
    if (voxel.isValid())
    {
      updatePositionUnsafe(voxel, pos);
    }
  }

  /// A convenience function for integrating a single hit into @p map with a voxel mean update.
  /// Note: this is a sub-optimal way of updating the @p map . Use a @c RayMapper or the @c Voxel API for batch updates.
  /// @param map The map to update. Must have an occupancy layer, but the mean layer is optional.
  /// @param sample The sample where the hit occurs.
  inline void integrateHit(ohm::OccupancyMap &map, const glm::dvec3 &sample)
  {
    const Key key(map.voxelKey(sample));
    Voxel<float> occupancy(&map, map.layout().occupancyLayer(), key);
    integrateHit(occupancy);
    Voxel<VoxelMean> mean(&map, map.layout().meanLayer(), key);
    updatePositionSafe(mean, sample);
  }
}  // namespace ohm

#endif  // VOXELMEAN_H
