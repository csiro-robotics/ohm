// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_VOXEL_TOUCH_TIME_COMPUTE_H
#define OHM_VOXEL_TOUCH_TIME_COMPUTE_H

/// Encode as milliseconds
#define OHM_VOXEL_TOUCH_TIME_SCALE 0.001

// Do not include config header. This can be used from GPU code.
inline unsigned encodeVoxelTouchTime(double timebase, double timestamp)
{
  return (unsigned)((timestamp - timebase) / OHM_VOXEL_TOUCH_TIME_SCALE);
}

// Do not include config header. This can be used from GPU code.
inline double decodeVoxelTouchTime(double timebase, unsigned touch_time)
{
  return touch_time * OHM_VOXEL_TOUCH_TIME_SCALE + timebase;
}

#endif  // OHM_VOXEL_TOUCH_TIME_COMPUTE_H
