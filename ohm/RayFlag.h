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
  {
    /// Default behaviour.
    kRfDefault = 0,
    /// Change behaviour such that the end point is considered another free voxel, rather than occupied.
    kRfEndPointAsFree = (1 << 0),
    /// Change behaviour such that traversal stops as soon as an occupied voxel is reached. Ray traversal terminates
    /// after adjusting the occupied voxel.
    kRfStopOnFirstOccupied = (1 << 1),
    /// Change behaviour such that only voxels which are occupied have their probability adjusted. Free and unknown
    /// voxels are left unchanged.
    kRfClearOnly = (1 << 2),
    /// Do not process the sample voxel.
    kRfExcludeSample = (1 << 3)
  };
#if !GPUTIL_DEVICE
} // namespace ohm
#endif  // !GPUTIL_DEVICE

#endif // RAYFLAG_H
