// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELOCCUPANCY_H
#define VOXELOCCUPANCY_H

// Note: this header is included in GPU code.
// Because of this "OhmConfig.h" and <cmath> cannot be included here and you may need to include those first.

/// @defgroup voxeloccupancy Voxel Occupancy Functions
/// These functions are used to update the voxel occupancy value in a consistent way. This deals with various update
/// conditions including:
/// - uninitialised voxel
/// - min/max value caps
/// - saturation
/// - null update (noop)

#if !GPUTIL_DEVICE
#ifndef __device__
#define __device__
#endif  // __device__
#ifndef __host__
#define __host__
#endif  // __host__

#ifndef NOMINMAX
#define NOMINMAX
#endif  // NOMINMAX
#include <cmath>

namespace ohm
{
#else  // GPUTIL_DEVICE

#endif  // GPUTIL_DEVICE

  /// @ingroup voxeloccupancy
  /// Adjust the @p occupancy value up by adding @c hit_adjustment , assuming that @c hit_adjustment is positive.
  ///
  /// The resulting occupancy is:
  /// - unchanged if @p null_update is true
  /// - unchanged if @p initial_value is at @c saturation_min
  /// - unchanged if @p initial_value is at @c saturation_max
  /// - min of @p hit_adjustment and @p max_value if @p initial_value is equal to @p uninitialised_value
  /// - min of `initial_value + hit_adjustment` and @p max_value otherwise
  ///
  /// @param occupancy The occupancy value to adjust.
  /// @param initial_value The initial value for @p occupancy (i.e., `*occupancy`)
  /// @param hit_adjustment The value to add to @p initial_value when initialised. Set this value when not initialised.
  /// @param uninitialised_value The special value used to indicate an uninitialised occupancy value. Typically +inf
  /// @param max_value The maximum value allowed for occupancy. Assumed to be positive.
  /// @param saturation_min The minimum saturation value. Occupancy at this (low) value should not be modified.
  ///   Use -inf to disable (or @c std::numeric_limits<float>::lowest() ).
  /// @param saturation_max The maximum saturation value. Occupancy at this (high) value should not be modified.
  ///   Use +inf to disable (or @c std::numeric_limits<float>::max() ).
  /// @param null_update Flag indicating nothing should be modified by this function. Used for loops which may have
  ///   such noop conditions.
  inline void occupancyAdjustHit(float *occupancy, float initial_value, float hit_adjustment, float uninitialised_value,
                                 float max_value, float saturation_min, float saturation_max, bool null_update)
  {
    const bool uninitialised = initial_value == uninitialised_value;
    const float base_value = (null_update || !uninitialised) ? initial_value : 0.0f;
    hit_adjustment =
      (!null_update && (uninitialised || saturation_min < initial_value && initial_value < saturation_max)) ?
        hit_adjustment :
        0.0f;
    *occupancy = (base_value != uninitialised_value) ? fmin(base_value + hit_adjustment, max_value) : base_value;
  }

  /// @ingroup voxeloccupancy
  /// Adjust the @p occupancy value by setting @p adjusted_value which is assumed to be an increase from the current
  /// value (or from an uninitialised value). Note that @c adjusted_value may be negative so long as it effects an
  /// increase in value.
  ///
  /// The resulting occupancy is:
  /// - unchanged if @p null_update is true
  /// - unchanged if @p initial_value is at @c saturation_min
  /// - unchanged if @p initial_value is at @c saturation_max
  /// - min of @p adjusted_value and @p max_value otherwise
  ///
  /// @param occupancy The occupancy value to adjust.
  /// @param initial_value The initial value for @p occupancy (i.e., `*occupancy`)
  /// @param adjusted_value The new value for @p occupancy assumed to be an increase from the @p initial_value .
  /// @param uninitialised_value The special value used to indicate an uninitialised occupancy value. Typically +inf
  /// @param max_value The maximum value allowed for occupancy. Assumed to be positive.
  /// @param saturation_min The minimum saturation value. Occupancy at this (low) value should not be modified.
  ///   Use -inf to disable (or @c std::numeric_limits<float>::lowest() ).
  /// @param saturation_max The maximum saturation value. Occupancy at this (high) value should not be modified.
  ///   Use +inf to disable (or @c std::numeric_limits<float>::max() ).
  /// @param null_update Flag indicating nothing should be modified by this function. Used for loops which may have
  ///   such noop conditions.
  inline void occupancyAdjustUp(float *occupancy, float initial_value, float adjusted_value, float uninitialised_value,
                                float max_value, float saturation_min, float saturation_max, bool null_update)
  {
    const bool uninitialised = initial_value == uninitialised_value;
    adjusted_value =
      (!null_update && (uninitialised || saturation_min < initial_value && initial_value < saturation_max)) ?
        adjusted_value :
        initial_value;
    *occupancy = (adjusted_value != uninitialised_value) ? fmin(max_value, adjusted_value) : adjusted_value;
  }

  /// @ingroup voxeloccupancy
  /// Adjust the @p occupancy value down by adding @c miss_adjustment , assuming that @c miss_adjustment is negative.
  ///
  /// The resulting occupancy is:
  /// - unchanged if @p null_update is true
  /// - unchanged if @p initial_value is at @c saturation_min
  /// - unchanged if @p initial_value is at @c saturation_max
  /// - max of @p miss_adjustment and @p min_value if @p initial_value is equal to @p uninitialised_value
  /// - max of `initial_value + miss_adjustment` and @p min_value otherwise
  ///
  /// @param occupancy The occupancy value to adjust.
  /// @param initial_value The initial value for @p occupancy (i.e., `*occupancy`)
  /// @param miss_adjustment The value to add to @p initial_value when initialised. Set this value when not initialised.
  /// @param uninitialised_value The special value used to indicate an uninitialised occupancy value. Typically +inf
  /// @param min_value The maximum value allowed for occupancy. Assumed to be negative.
  /// @param saturation_min The minimum saturation value. Occupancy at this (low) value should not be modified.
  ///   Use -inf to disable (or @c std::numeric_limits<float>::lowest() ).
  /// @param saturation_max The maximum saturation value. Occupancy at this (high) value should not be modified.
  ///   Use +inf to disable (or @c std::numeric_limits<float>::max() ).
  /// @param null_update Flag indicating nothing should be modified by this function. Used for loops which may have
  ///   such noop conditions.
  inline void occupancyAdjustMiss(float *occupancy, float initial_value, float miss_adjustment,
                                  float uninitialised_value, float min_value, float saturation_min,
                                  float saturation_max, bool null_update)
  {
    const bool uninitialised = initial_value == uninitialised_value;
    const float base_value = (null_update || !uninitialised) ? initial_value : 0.0f;
    miss_adjustment =
      (!null_update && (uninitialised || saturation_min < initial_value && initial_value < saturation_max)) ?
        miss_adjustment :
        0.0f;
    *occupancy = (base_value != uninitialised_value) ? fmax(min_value, base_value + miss_adjustment) : base_value;
  }

  /// @ingroup voxeloccupancy
  /// Adjust the @p occupancy value by setting @p adjusted_value which is assumed to be a decrease from the current
  /// value (or from an uninitialised value). Note that @c adjusted_value may be positive so long as it effects a
  /// decrease in value.
  ///
  /// The resulting occupancy is:
  /// - unchanged if @p null_update is true
  /// - unchanged if @p initial_value is at @c saturation_min
  /// - unchanged if @p initial_value is at @c saturation_max
  /// - max of @p adjusted_value and @p min_value otherwise
  ///
  /// @param occupancy The occupancy value to adjust.
  /// @param initial_value The initial value for @p occupancy (i.e., `*occupancy`)
  /// @param adjusted_value The new value for @p occupancy assumed to be an increase from the @p initial_value .
  /// @param uninitialised_value The special value used to indicate an uninitialised occupancy value. Typically +inf
  /// @param min_value The maximum value allowed for occupancy. Assumed to be positive.
  /// @param saturation_min The minimum saturation value. Occupancy at this (low) value should not be modified.
  ///   Use -inf to disable (or @c std::numeric_limits<float>::lowest() ).
  /// @param saturation_max The maximum saturation value. Occupancy at this (high) value should not be modified.
  ///   Use +inf to disable (or @c std::numeric_limits<float>::max() ).
  /// @param null_update Flag indicating nothing should be modified by this function. Used for loops which may have
  ///   such noop conditions.
  inline void occupancyAdjustDown(float *occupancy, float initial_value, float adjusted_value,
                                  float uninitialised_value, float min_value, float saturation_min,
                                  float saturation_max, bool null_update)
  {
    const bool uninitialised = initial_value == uninitialised_value;
    adjusted_value =
      (!null_update && (uninitialised || saturation_min < initial_value && initial_value < saturation_max)) ?
        adjusted_value :
        initial_value;
    *occupancy = (adjusted_value != uninitialised_value) ? fmax(min_value, adjusted_value) : adjusted_value;
  }

#if !GPUTIL_DEVICE
}  // namespace ohm
#endif  // !GPUTIL_DEVICE

#endif  // VOXELMEAN_H
