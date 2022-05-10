// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef RAYFLAG_H
#define RAYFLAG_H

// Used as a GPU header.

#if !GPUTIL_DEVICE
namespace ohm
{
#endif  // !GPUTIL_DEVICE
/// Flags affecting the behaviour of how rays are integrated into the map.
enum RayFlag
#if !GPUTIL_DEVICE
  // Unsigned type specification not valid for NVidia OpenCL code.
  : unsigned
#endif  // !GPUTIL_DEVICE
{
  /// Default behaviour.
  kRfDefault = 0,
  /// Change behaviour such that the end point is considered another free voxel, rather than occupied.
  kRfEndPointAsFree = (1u << 0u),
  /// Change behaviour such that traversal stops as soon as an occupied voxel is reached. Ray traversal terminates
  /// after adjusting the occupied voxel.
  kRfStopOnFirstOccupied = (1u << 1u),
  /// Skip the first, non-sample voxel in the ray. Useful for dealing with secondary returns where multiple samples
  /// lie along the same ray. This way, secondary samples can start at the previous/primary sample point without
  /// affecting the occupancy of this voxel. However, note that in this use case, the sensor voxel will also always be
  /// skipped.
  kRfExcludeOrigin = (1u << 2u),
  /// Do not process the sample voxel.
  kRfExcludeSample = (1u << 3u),
  /// Exclude the ray part, integrating only the sample. This flag is only recommended in debugging or validation.
  /// @c RayMapperBase code is not optimised for this flag.
  kRfExcludeRay = (1u << 4u),
  /// Do not adjust the occupancy value of currently unobserved voxels.
  kRfExcludeUnobserved = (1u << 5u),
  /// Do not adjust the occupancy value of currently free voxels.
  kRfExcludeFree = (1u << 6u),
  /// Do not adjust the occupancy value of currently occupied voxels.
  kRfExcludeOccupied = (1u << 7u),

  /// Trace each ray backwards, from sample to origin? The default is to trace rays from origin to sample. Reverse
  /// tracing can reduce GPU voxel contention near sensor origins and improve performance. However, forward tracing
  /// must be used for ray clearing patterns.
  ///
  /// Note: even when reversed, the sample voxel is updated to help avoid contention where many samples fall in the
  /// same voxel.
  ///
  // This may be ignored by some algorithms, such as ray queries.
  kRfReverseWalk = (1u << 8u),

  /// Internal use flag values start here (not to be set by user).
  kRfInternal = (1u << 16u),
  /// Marks that timestamps are available for GPU. This is an internal flag.
  kRfInternalTimestamps = (kRfInternal << 0u)
};
#if !GPUTIL_DEVICE
}  // namespace ohm
#endif  // !GPUTIL_DEVICE

#endif  // RAYFLAG_H
