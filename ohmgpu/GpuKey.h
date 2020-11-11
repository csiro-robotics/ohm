// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUKEY_H
#define GPUKEY_H

#include "MapCoord.h"

/// printf formating string macro for @c GpuKey.
#define KEY_F "[%d %d %d : %d %d %d]"
/// printf argument expansion macro for @c GpuKey.
///
/// @code
/// GpuKey key = { 1, 2, 3, 4, 5, 6, 7 };
/// printf("Key is: " KEY_F "\n", KEY_A(key));
/// @endcode
#define KEY_A(key) \
  (key).region[0], (key).region[1], (key).region[2], (int)(key).voxel[0], (int)(key).voxel[1], (int)(key).voxel[2]

#if !GPUTIL_DEVICE

#ifndef __device__
#define __device__
#endif  // __device__
#ifndef __host__
#define __host__
#endif  // __host__

namespace ohm
{
#endif  // !GPUTIL_DEVICE
/// @c ohm::Key representation in GPU.
///
/// This structure must exactly match the memory alignment of ohm::Key.
typedef struct GpuKey_t  // NOLINT(readability-identifier-naming, modernize-use-using)
{
  /// Region key.
  short region[3];  // NOLINT(modernize-avoid-c-arrays, google-runtime-int)
  /// Voxel key.
  /// Element 3 is used to identify clipped ray keys. That is, voxel[3] = 1 for a "sample voxel" indicates we have
  /// actually clipped the sample ray and, while the voxel is the last relevant voxel in the ray, it is not the
  /// sample voxel.
  unsigned char voxel[4];  // NOLINT(modernize-avoid-c-arrays)
} GpuKey;
#if !GPUTIL_DEVICE
}  // namespace ohm
#endif  // !GPUTIL_DEVICE

#ifdef GPUTIL_DEVICE

/// Test for equality between two @c GpuKey objects.
/// @param a The first key.
/// @param b The second key.
/// @return True if @p a and @p b are exactly equal.
inline __device__ __host__ bool equalKeys(const GpuKey *a, const GpuKey *b);

__device__ __host__ void stepKeyAlongAxis(GpuKey *key, int axis, int step, const int3 *regionDim);
__device__ __host__ void moveKeyAlongAxis(GpuKey *key, int axis, int step, const int3 *regionDim);

#if __OPENCL_C_VERSION__ >= 200
__device__ __host__ void copyKey(GpuKey *out, const GpuKey *in);

__device__ __host__ void copyKey(GpuKey *out, const GpuKey *in)
{
  out->region[0] = in->region[0];
  out->region[1] = in->region[1];
  out->region[2] = in->region[2];
  out->voxel[0] = in->voxel[0];
  out->voxel[1] = in->voxel[1];
  out->voxel[2] = in->voxel[2];
  out->voxel[3] = in->voxel[3];
}
#else  // __OPENCL_C_VERSION__ >= 200
// copyKey implemented with a macro as before OpenCL 2.0 we need to cater for __global memory qualifier.
#define copyKey(out, in) *out = *in
#endif  // __OPENCL_C_VERSION__ >= 200

inline __device__ __host__ bool equalKeys(const GpuKey *a, const GpuKey *b)
{
  return a->region[0] == b->region[0] && a->region[1] == b->region[1] && a->region[2] == b->region[2] &&
         a->voxel[0] == b->voxel[0] && a->voxel[1] == b->voxel[1] && a->voxel[2] == b->voxel[2];
}

inline __device__ __host__ void stepKeyAlongAxis(GpuKey *key, int axis, int step, const int3 *regionDim)
{
  const int axisDim = (axis == 0) ? regionDim->x : ((axis == 1) ? regionDim->y : regionDim->z);
  int vind = key->voxel[axis] + step;
  // Manage region stepping.
  key->region[axis] += (vind < 0) ? -1 : 0;        // Step down a region
  key->region[axis] += (vind >= axisDim) ? 1 : 0;  // Step up a region
  vind = (vind >= 0) ? vind : axisDim - 1;         // Underflow voxel index.
  vind = (vind < axisDim) ? vind : 0;              // Overflow voxel index.
  key->voxel[axis] = (uchar)vind;
}


inline __device__ __host__ void moveKeyAlongAxis(GpuKey *key, int axis, int step, const int3 *regionDim)
{
  const int axisDim = (axis == 0) ? regionDim->x : ((axis == 1) ? regionDim->y : regionDim->z);

  // We first step within the chunk region. If we can't then we step the region and reset
  // stepped local axis value. We need to expand the byte precision of the voxel key to support region changes.
  int localKey[3] = { key->voxel[0], key->voxel[1], key->voxel[2] };
  localKey[axis] += step;
  // glm::i16vec3 regionKey = key.regionKey();
  if (step >= 0)
  {
    // Positive step or zero step.
    key->region[axis] += localKey[axis] / axisDim;
    localKey[axis] %= axisDim;
  }
  else
  {
    // Negative step.
    // Create a region step which simulates a floating point floor.
    key->region[axis] += ((localKey[axis] - (axisDim - 1)) / axisDim);
    if (localKey[axis] < 0)
    {
      // This is nuts. In C/C++, the % operator is not actually a modulus operator.
      // It's a "remainder" operator. A modulus operator should only give positive results,
      // but in C a negative input will generate a negative output. Through the magic of
      // StackOverflow, here's a good explanation:
      //  https://stackoverflow.com/questions/11720656/modulo-operation-with-negative-numbers
      // This means that here, given localKey[axis] is negative, the modulus:
      //    localKey[axis] % axisDim
      // will give results in the range (-axisDim, 0]. So, lets set the limit
      // to 4, then we get the following: like follows:
      //
      // i  i%4   4 - i%4
      //  0  0    4
      // -1 -1    3
      // -2 -2    2
      // -3 -3    1
      // -4  0    4
      // -5 -1    3
      // -6 -2    2
      // -7 -3    1
      // -8  0    4
      //
      // The last column above shows the results of the following line of code.
      // This generates the wrong results in that the '4' results in the last
      // column should be 0. We could apply another "% axisDim" to
      // the result or just add the if statement below.
      localKey[axis] = axisDim + localKey[axis] % axisDim;
      localKey[axis] = (localKey[axis] != axisDim) ? localKey[axis] : 0;
    }
    // else
    // {
    //   localKey[axis] %= axisDim;
    // }
  }

  key->voxel[0] = localKey[0];
  key->voxel[1] = localKey[1];
  key->voxel[2] = localKey[2];
}

// GpuKey dir calculator for a single axis. Results are:
// . 0 => keys are equal on axis.
// . 1 => end key indexes a higher voxel along axis.
// . -1 => end key indexes a lower voxel along axis.
//
// Calculated as follows:
//
// . -1 end_region < start_region
// . 1 end_region > start_region
// . -1 end_region == start_region && end_voxel < start_voxel
// . 1 end_region == start_region && end_voxel > start_voxel
// . 0 end_region == start_region && end_voxel == start_voxel
#define _OHM_GPUKEY_AXIS_DIR(start, end, axis)                      \
  ((end)->region[axis] != (start)->region[axis] ?                   \
     ((end)->region[axis] > (start)->region[axis] ? 1.0f : -1.0f) : \
     ((end)->voxel[axis] != (end)->voxel[1] ? ((end)->voxel[axis] > (start)->voxel[axis] ? 1.0f : -1.0f) : 0.0f))

inline __device__ __host__ float3 keyDirection(const GpuKey *start, const GpuKey *end)
{
  return make_float3(_OHM_GPUKEY_AXIS_DIR(start, end, 0), _OHM_GPUKEY_AXIS_DIR(start, end, 1),
                     _OHM_GPUKEY_AXIS_DIR(start, end, 2));
}

// Calculate the number of voxels from start to end.
//
// @todo Consolidate this code with OccupancyMap::rangeBetween()
inline __device__ __host__ int3 keyDiff(const GpuKey *start, const GpuKey *end, const int3 *regionDim)
{
  // First diff the regions.
  const int3 region_diff =
    make_int3(end->region[0] - start->region[0], end->region[1] - start->region[1], end->region[2] - start->region[2]);
  int3 voxel_diff;

  // Voxel difference is the sum of the local difference plus the region step difference.
  voxel_diff.x = end->voxel[0] - start->voxel[0] + region_diff.x * regionDim->x;
  voxel_diff.y = end->voxel[1] - start->voxel[1] + region_diff.y * regionDim->y;
  voxel_diff.z = end->voxel[2] - start->voxel[2] + region_diff.z * regionDim->z;

  return voxel_diff;
}

#endif  // GPUTIL_DEVICE

#endif  // GPUKEY_H
