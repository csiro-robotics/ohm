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

#ifndef __OPENCL_C_VERSION__
namespace ohm
{
#endif  // !__OPENCL_C_VERSION__
  /// @c ohm::Key representation in GPU.
  ///
  /// This structure must exactly match the memory alignment of ohm::Key.
  struct GpuKey
  {
    /// Region key.
    short region[3];
    /// Voxel key.
    // Element 3 is provided for padding, but may be used as a context based value.
    unsigned char voxel[4];
  };
#ifndef __OPENCL_C_VERSION__
}  // namespace ohm
#endif  // !__OPENCL_C_VERSION__

#ifdef __OPENCL_C_VERSION__

void stepKeyAlongAxis(struct GpuKey *key, int axis, int step, const int3 *regionDim);
void moveKeyAlongAxis(struct GpuKey *key, int axis, int step, const int3 *regionDim);

#if __OPENCL_C_VERSION__ >= 200
void copyKey(struct GpuKey *out, const struct GpuKey *in);

void copyKey(struct GpuKey *out, const struct GpuKey *in)
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

void stepKeyAlongAxis(struct GpuKey *key, int axis, int step, const int3 *regionDim)
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


void moveKeyAlongAxis(struct GpuKey *key, int axis, int step, const int3 *regionDim)
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

#endif  // __OPENCL_C_VERSION__

#endif  // GPUKEY_H
