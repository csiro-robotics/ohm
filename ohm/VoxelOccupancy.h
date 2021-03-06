// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELOCCUPANCY_H
#define VOXELOCCUPANCY_H

#include "OhmConfig.h"

#include "OccupancyMap.h"
#include "OccupancyType.h"
#include "Voxel.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif  // NOMINMAX
#include <cmath>


/// @defgroup voxeloccupancy Voxel Occupancy Functions
/// These functions are used to update the voxel occupancy value in a consistent way. This deals with various update
/// conditions including:
/// - uninitialised voxel
/// - min/max value caps
/// - saturation
/// - null update (noop)

namespace ohm
{
/// Define fmin for use in `VoxelOccpuancyCompute.h` functions. @c fmin() is overloaded in GPU for float and double,
/// but the global C symbol is double only. We alias @c std::fmin() , which is overloaded, so we can call just @c fmin()
/// in both CPU and GPU code.
using std::fmax;
using std::fmin;
/// Define fmax for use in `VoxelOccpuancyCompute.h` as per @c fmin()
#include "VoxelOccupancyCompute.h"

/// @ingroup voxeloccupancy
/// Value used to identify invalid or uninitialised voxels.
/// @return A numeric value considered invalid for a voxel value.
constexpr float unobservedOccupancyValue()
{
  return std::numeric_limits<float>::infinity();
}


/// @ingroup voxeloccupancy
/// Integrate a hit into the referenced @p voxel .
/// This adjust the occupancy data - @c Voxel<float>::data() - increasing it by the @c OccupancyMap::hitValue() .
///
/// Note @c voxel is self contained so long as it is valid.
///
/// @c Voxel<float>::isValid() must be true before calling.
///
/// @param voxel The voxel to update.
inline void integrateHit(Voxel<float> &voxel)
{
  float occupancy;
  voxel.read(&occupancy);
  const float initial_value = occupancy;
  const OccupancyMap &map = *voxel.map();
  occupancyAdjustHit(&occupancy, initial_value, map.hitValue(), unobservedOccupancyValue(), map.maxVoxelValue(),
                     map.saturateAtMinValue() ? map.minVoxelValue() : std::numeric_limits<float>::lowest(),
                     map.saturateAtMaxValue() ? map.maxVoxelValue() : std::numeric_limits<float>::max(), false);
  voxel.write(occupancy);
}

/// A convenience function for integrating a single hit into @p map . Note this is a sub-optimal way of updating
/// the @p map . Use a @c RayMapper or the @c Voxel API for batch updates.
/// @param map The map to update.
/// @param key The key for the voxel to create/update.
inline void integrateHit(ohm::OccupancyMap &map, const ohm::Key &key)
{
  Voxel<float> voxel(&map, map.layout().occupancyLayer(), key);
  integrateHit(voxel);
}

/// @ingroup voxeloccupancy
/// Integrate a miss into the referenced @p voxel .
/// This adjusts the occupancy data - @c Voxel<float>::data() - decreasing it by adding @c OccupancyMap::missValue() .
///
/// Note @c voxel is self contained so long as it is valid.
///
/// @c Voxel<float>::isValid() must be true before calling.
///
/// @param voxel The voxel to update.
inline void integrateMiss(Voxel<float> &voxel)
{
  float occupancy;
  voxel.read(&occupancy);
  const float initial_value = occupancy;
  const OccupancyMap &map = *voxel.map();
  occupancyAdjustMiss(&occupancy, initial_value, map.missValue(), unobservedOccupancyValue(), map.minVoxelValue(),
                      map.saturateAtMinValue() ? map.minVoxelValue() : std::numeric_limits<float>::lowest(),
                      map.saturateAtMaxValue() ? map.maxVoxelValue() : std::numeric_limits<float>::max(), false);
  voxel.write(occupancy);
}

/// A convenience function for integrating a single miss into @p map . Note this is a sub-optimal way of updating
/// the @p map . Use a @c RayMapper or the @c Voxel API for batch updates.
/// @param map The map to update.
/// @param key The key for the voxel to create/update.
inline void integrateMiss(ohm::OccupancyMap &map, const Key &key)
{
  Voxel<float> voxel(&map, map.layout().occupancyLayer(), key);
  integrateMiss(voxel);
}

/// @ingroup voxeloccupancy
/// Determine the @c OccupancyType for an occupancy @p value
///
/// @param value The value to categorise.
/// @param map The map within which the value sits.
/// @return The @c OccupancyType for the @p value .
inline OccupancyType occupancyType(float value, const OccupancyMap &map)
{
  if (value < unobservedOccupancyValue())
  {
    if (value < map.occupancyThresholdValue())
    {
      return kFree;
    }

    return kOccupied;
  }

  return kUnobserved;
}

template <typename T>
inline OccupancyType occupancyTypeT(const Voxel<T> &voxel)
{
  if (voxel.isValid())
  {
    float occupancy;
    voxel.read(&occupancy);
    return occupancyType(occupancy, *voxel.map());
  }
  return kNull;
}

/// Query the @c OccupancyType for @p voxel , which may be null/invalid.
/// @param voxel The voxel of interest by occupancy layer.
/// @return The occupancy type for the voxel given it's value. May be @c kNull when @p voxel is null.
inline OccupancyType occupancyType(const Voxel<float> &voxel)
{
  return occupancyTypeT(voxel);
}
/// @overload
inline OccupancyType occupancyType(const Voxel<const float> &voxel)
{
  return occupancyTypeT(voxel);
}

/// @ingroup voxeloccupancy
/// Return @c true if @p value represents an occupied voxel within @p map .
/// @param value The occupancy value to test.
/// @param map The map context within which to operate.
/// @return True if occupied.
inline bool isOccupied(float value, const OccupancyMap &map)
{
  return value != unobservedOccupancyValue() && value >= map.occupancyThresholdValue();
}

template <typename T>
inline bool isOccupiedT(const Voxel<T> &voxel)
{
  float occupancy;
  voxel.read(&occupancy);
  return isOccupied(occupancy, *voxel.map());
}

/// @ingroup voxeloccupancy
/// Return @c true if @p voxel represents an occupied voxel.
/// @param voxel The occupancy voxel test. Must not be null.
/// @return True if occupied.
inline bool isOccupied(const Voxel<float> &voxel)
{
  return voxel.isValid() && isOccupiedT(voxel);
}
/// @overload
inline bool isOccupied(const Voxel<const float> &voxel)
{
  return voxel.isValid() && isOccupiedT(voxel);
}


/// @ingroup voxeloccupancy
/// Return @c true if @p value represents a free voxel within @p map .
/// @param value The occupancy value to test. Must not be null.
/// @param map The map context within which to operate.
/// @return True if free.
inline bool isFree(float value, const OccupancyMap &map)
{
  return value != unobservedOccupancyValue() && value < map.occupancyThresholdValue();
}

template <typename T>
inline bool isFreeT(const Voxel<T> &voxel)
{
  float occupancy;
  voxel.read(&occupancy);
  return isFree(occupancy, *voxel.map());
}

/// @ingroup voxeloccupancy
/// Return @c true if @p voxel represents a free voxel.
/// @param voxel The occupancy voxel to test. Must not be null.
/// @return True if free.
inline bool isFree(const Voxel<float> &voxel)
{
  return voxel.isValid() && isFreeT(voxel);
}
/// @overload
inline bool isFree(const Voxel<const float> &voxel)
{
  return voxel.isValid() && isFreeT(voxel);
}


/// @ingroup voxeloccupancy
/// Return @c true if @p value represents an unobserved voxel value.
/// @param value The occupancy value to test.
/// @param map The map context within which to operate. Not used.
/// @return True if unobserved.
inline bool isUnobserved(float value, const OccupancyMap &map)
{
  (void)map;
  return value == unobservedOccupancyValue();
}
/// @overload
inline bool isUnobserved(float value)
{
  return value == unobservedOccupancyValue();
}

template <typename T>
inline bool isUnobservedT(const Voxel<T> &voxel)
{
  float occupancy;
  voxel.read(&occupancy);
  return isUnobserved(occupancy, *voxel.map());
}

/// @ingroup voxeloccupancy
/// Return @c true if @p voxel represents an unobserved, but not null voxel.
/// @param voxel The occupancy voxel to test. Must not be null.
/// @return True if unobserved.
inline bool isUnobserved(const Voxel<float> &voxel)
{
  return isUnobservedT(voxel);
}

/// @overload
inline bool isUnobserved(const Voxel<const float> &voxel)
{
  return isUnobservedT(voxel);
}

template <typename T>
inline bool isUnobservedOrNullT(const Voxel<T> &voxel)
{
  // Note: the call to voxel.isValid() is effectively redundant as it's the negated check of voxel.isNull()
  // but it does silence a clang-tidy warning. Perhaps this will help with branch prediction as well because the explict
  // check is clearer?
  return voxel.isNull() || (voxel.isValid() && isUnobservedT(voxel));
}

/// @ingroup voxeloccupancy
/// Return @c true if @p voxel represents an unobserved, but not null voxel.
/// @param voxel The occupancy voxel to test. May be null.
/// @return True if unobserved or null.
inline bool isUnobservedOrNull(const Voxel<float> &voxel)
{
  return isUnobservedOrNullT(voxel);
}
/// @overload
inline bool isUnobservedOrNull(const Voxel<const float> &voxel)
{
  return isUnobservedOrNullT(voxel);
}
}  // namespace ohm

#endif  // VOXELOCCUPANCY_H
