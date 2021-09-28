// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_VOXEL_TOUCH_TIME_COMPUTE_H
#define OHM_VOXEL_TOUCH_TIME_COMPUTE_H

// Do not include config header. This can be used from GPU code.

#if !GPUTIL_DEVICE
#define OHM_DEVICE_HOST
#else  // GPUTIL_DEVICE
#define OHM_DEVICE_HOST __device__ __host__
#endif  // GPUTIL_DEVICE

/// Encode as milliseconds
#define OHM_VOXEL_TOUCH_TIME_SCALE 0.001

/// Encode a voxel timestamp. Encoded as milliseconds since @p timebase .
/// @param timebase The base time to encode relative to.
/// @param timestamp The timestamp to encode.
/// @return Milliseconds elapsed between @p timebase and @p timestamp .
inline unsigned OHM_DEVICE_HOST encodeVoxelTouchTime(double timebase, double timestamp)
{
  return (unsigned)((timestamp - timebase) / OHM_VOXEL_TOUCH_TIME_SCALE);
}

/// Decode a voxel timestamp. Decoded as milliseconds since @p timebase .
/// @param timebase The base time to decode relative to.
/// @param touch_time Encoded time - milliseconds elapsed since @p timebase .
/// @return The decoded tiemstamp.
inline double OHM_DEVICE_HOST decodeVoxelTouchTime(double timebase, unsigned touch_time)
{
  return touch_time * OHM_VOXEL_TOUCH_TIME_SCALE + timebase;
}

#undef OHM_DEVICE_HOST

#endif  // OHM_VOXEL_TOUCH_TIME_COMPUTE_H
