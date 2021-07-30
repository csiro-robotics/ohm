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

  /// Internal use flag values start here (not to be set by user).
  kRfInternal = (1u << 16u),
  /// Marks that timestamps are available for GPU. This is an internal flag.
  kRfInternalTimestamps = (kRfInternal << 0u)
};
#if !GPUTIL_DEVICE
}  // namespace ohm
#endif  // !GPUTIL_DEVICE

#endif  // RAYFLAG_H
