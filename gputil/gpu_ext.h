// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

// This header file can be included into an OpenCL to be compiled for either OpenCL or CUDA.
// It is used to help provide a mapping from functions available on one platform to the other.
// For example, we define make_typeN() macros on OpenCL to match CUDA style initialisation of
// vector types.
//
// The header also helps resolve some missing defines on some OpenCL platforms.
//
// This is a work in progress and will be extended as needed.
//
// Other usage notes:
// - Do not use __local as a prefix to local memory arguments in OpenCL. Use __localarg instead.
//   This is then removed in CUDA compilation as it is not needed. __local already has meaning
//   in CUDA so we couldn't just use that.

#ifndef GPU_EXT_H_
#define GPU_EXT_H_

#ifndef NULL
#define NULL 0
#endif  // !NULL

// For CUDA import:
#ifdef __CUDACC__

//-----------------------------------------------------------------------------
// CUDA defines
//-----------------------------------------------------------------------------
#define LOCAL_ARG(TYPE, VAR)
#define LOCAL_VAR(TYPE, VAR, SIZE) TYPE VAR = (TYPE)&shared_mem_[(shared_mem_offset_ =+ SIZE)];
#define LOCAL_ENABLE() \
  size_t shared_mem_offset_ = 0; \
  extern __shared__ char shared_mem_[]
#define LM_PER_THREAD(per_thread_size) ((per_thread_size) * blockDim.x * blockDim.y * blockDim.z)

#else  // __CUDACC__

#define LOCAL_ARG(TYPE, VAR) \
  , __local TYPE VAR
#define LOCAL_VAR(TYPE, VAR, SIZE)
#define LOCAL_ENABLE()
#define LM_PER_THREAD(PER_THREAD_SIZE)

//-----------------------------------------------------------------------------
// OpenCL defines
//-----------------------------------------------------------------------------
#define __localarg __local
#define __device__

#ifndef make_char2
#define make_char2 (char2)
#define make_char3 (char3)
#define make_char4 (char4)
#define make_uchar2 (uchar2)
#define make_uchar3 (uchar3)
#define make_uchar4 (uchar4)
#define make_short2 (short2)
#define make_short3 (short3)
#define make_short4 (short4)
#define make_short16 (short16)
#define make_ushort2 (ushort2)
#define make_ushort3 (ushort3)
#define make_ushort4 (ushort4)
#define make_int2 (int2)
#define make_int3 (int3)
#define make_int4 (int4)
#define make_uint2 (uint2)
#define make_uint3 (uint3)
#define make_uint4 (uint4)
#define make_long2 (long2)
#define make_long3 (long3)
#define make_long4 (long4)
#define make_ulong2 (ulong2)
#define make_ulong3 (ulong3)
#define make_ulong4 (ulong4)
#define make_float2 (float2)
#define make_float3 (float3)
#define make_float4 (float4)
#define make_double2 (double2)
#endif  // make_char2

#ifndef xyz
#define xyz(V) (V).xyz
#endif  // xyz

// Missing defines on some OpenCL devices.
// TODO: add more.
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288f
#endif  // M_PI

#define SQR(X) ((X) * (X))

#endif  // __CUDACC__

inline __device__ bool isGlobalThread(size_t x, size_t y, size_t z)
{
  return x == get_global_id(0) && y == get_global_id(1) && z == get_global_id(2);
}

inline __device__ bool isLocalThread(size_t x, size_t y, size_t z)
{
  return x == get_local_id(0) && y == get_local_id(1) && z == get_local_id(2);
}

inline __device__ bool isInGroup(size_t x, size_t y, size_t z)
{
  return x == get_group_id(0) && y == get_group_id(1) && z == get_group_id(2);
}

#endif  // GPU_EXT_H_
