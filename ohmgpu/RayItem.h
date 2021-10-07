// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_RAYITEM_H
#define OHMGPU_RAYITEM_H

#include "OhmGpuConfig.h"

#include <ohm/Key.h>
#include <ohm/RayFilter.h>

#include <glm/vec3.hpp>

#include <cinttypes>

namespace ohm
{
/// This is an internal structure used in @c GpuMap for sorting algorithm to group rays before upload.
/// It is made available for use in unit tests to ensure the same order used in GPU can be used in CPU after rays have
/// been sorted.
struct RayItem
{
  /// Origin of the ray. Unless clipped, this is the sensor location.
  glm::dvec3 origin;
  /// End point of the sample ray. Unless clipped, this is the location of the sample detection.
  glm::dvec3 sample;
  /// Map @c Key corresponding to @p origin .
  Key origin_key;
  /// Map @c Key corresponding to @p sample .
  Key sample_key;
  /// Intensity value associated with the @p sample .
  float intensity;
  /// Quantised, relative timestamp.
  unsigned timestamp;
  /// @c RayFilterFlag values corresponding to any modification which have been made to @p origin and @p sample .
  unsigned filter_flags;

  /// Key comparison `a < b` to help with less than operator.
  static bool isLessKey(const Key &a, const Key &b);

  /// Sorting operator.
  bool operator<(const RayItem &other) const;
};

inline bool isLessKey(const Key &a, const Key &b)
{
  // Multiplier/shift value to move a region key axis such that each axis is in its own bit set.
  const int64_t region_stride = 0x10000u;
  // Shift and mask together the region key axes
  const int64_t region_index_a = static_cast<int64_t>(a.regionKey().x) +
                                 static_cast<int64_t>(region_stride * a.regionKey().y) +
                                 region_stride * region_stride * static_cast<int64_t>(a.regionKey().z);
  const int64_t region_index_b = static_cast<int64_t>(b.regionKey().x) +
                                 region_stride * static_cast<int64_t>(b.regionKey().y) +
                                 region_stride * region_stride * static_cast<int64_t>(b.regionKey().z);
  // Multiplier/shift value to move a local key axis such that each axis is in its own bit set.
  const uint32_t local_stride = 0x100u;
  // Shift and mask together the local key axes
  const uint32_t local_index_a = uint32_t(a.localKey().x) + local_stride * uint32_t(a.localKey().y) +
                                 local_stride * local_stride * uint32_t(a.localKey().z);
  const uint32_t local_index_b = uint32_t(b.localKey().x) + local_stride * uint32_t(b.localKey().y) +
                                 local_stride * local_stride * uint32_t(b.localKey().z);
  // Compare the results.
  // - By region index next
  // - Finally by local index.
  return (region_index_a < region_index_b) || (region_index_a == region_index_b && local_index_a < local_index_b);
}

inline bool RayItem::operator<(const RayItem &other) const
{
  // For the comparison, we merge the region key values into a single 64-bit value
  // and the same for the local index. Then we compare the resulting values.
  // The final ordering is irrelevant in terms of which is "less". The goal is to group
  // items with the same sample key.

  const RayItem &a = *this;
  const RayItem &b = other;
  const int clipped_a = !!(a.filter_flags & kRffClippedEnd);
  const int clipped_b = !!(b.filter_flags & kRffClippedEnd);

  // Compare the results. We sort such that:
  // - Items with unclipped end points come first.
  // - By sample key
  // - By origin key
  return clipped_a < clipped_b || (clipped_a == clipped_b && a.sample_key < b.sample_key) ||
         (clipped_a == clipped_b && a.sample_key == b.sample_key && a.origin_key < b.origin_key);
}

}  // namespace ohm

#endif  // OHMGPU_RAYITEM_H
