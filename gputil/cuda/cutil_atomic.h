// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CUTIL_ATOMIC_H
#define CUTIL_ATOMIC_H

#include <cstddef>

//----------------------------------------------------------------------------------------------------------------------
// Alias CUDA atomics to match C/OpenCL 2.0 atomics
//----------------------------------------------------------------------------------------------------------------------
#ifdef __CUDACC__
using atomic_int = int;
using atomic_uint = uint;
using atomic_long = long;
using atomic_ulong = long;
using atomic_float = float;
using atomic_double = double;
using atomic_intptr_t = size_t;
using atomic_uintptr_t = size_t;
using atomic_size_t = size_t;
using atomic_ptrdiff_t = size_t;

inline __device__ void gputilAtomicInit(atomic_int *obj, int val) { *obj = val; }
inline __device__ void gputilAtomicStore(atomic_int *obj, int val) { *obj = val; }
inline __device__ int gputilAtomicLoad(atomic_int *obj) { return *obj; }
inline __device__ int gputilAtomicExchange(atomic_int *obj, int desired) { return atomicExch(obj, desired); }
inline __device__ bool gputilAtomicCas(atomic_int *obj, int expected, int desired) { return atomicCAS(obj, expected, desired) == expected; }

inline __device__ void gputilAtomicInit(atomic_uint *obj, uint val) { *obj = val; }
inline __device__ void gputilAtomicStore(atomic_uint *obj, uint val) { *obj = val; }
inline __device__ uint gputilAtomicLoad(atomic_uint *obj) { return *obj; }
inline __device__ uint gputilAtomicExchange(atomic_uint *obj, uint desired) { return atomicExch(obj, desired); }
inline __device__ bool gputilAtomicCas(atomic_uint *obj, uint expected, uint desired) { return atomicCAS(obj, expected, desired) == expected; }

inline __device__ void gputilAtomicInitF32(atomic_float *obj, float val) { *obj = val; }
inline __device__ void gputilAtomicStoreF32(atomic_float *obj, float val) { *obj = val; }
inline __device__ float gputilAtomicLoadF32(atomic_float *obj) { return *obj; }
inline __device__ float gputilAtomicExchangeF32(atomic_float *obj, float desired) { return atomicExch(obj, desired); }
inline __device__ bool gputilAtomicCasF32(atomic_float *obj, float expected, float desired)
{
  return __int_as_float(atomicCAS((atomic_int *)obj, __float_as_int(expected), __float_as_int(desired))) == expected;
}

// Note: CUDA semantics for atomicInc/Dec differ from OpenCL atomic_inc/dec. We use the OpenCL semantics, where there
// is no validation value and always increments/decrements.
inline __device__ int gputilAtomicAdd(atomic_int *p, int val) { return atomicAdd(p, val); }
inline __device__ int gputilAtomicSub(atomic_int *p, int val) { return atomicSub(p, val); }
inline __device__ int gputilAtomicInc(atomic_int *p) { return atomicAdd(p, 1); }
inline __device__ int gputilAtomicDec(atomic_int *p) { return atomicSub(p, 1); }
inline __device__ int gputilAtomicMin(atomic_int *p, int val) { return atomicMin(p, val); }
inline __device__ int gputilAtomicMax(atomic_int *p, int val) { return atomicMax(p, val); }
inline __device__ int gputilAtomicAnd(atomic_int *p, int val) { return atomicAnd(p, val); }
inline __device__ int gputilAtomicOr(atomic_int *p, int val) { return atomicOr(p, val); }
inline __device__ int gputilAtomicXor(atomic_int *p, int val) { return atomicXor(p, val); }

inline __device__ uint gputilAtomicAdd(atomic_uint *p, uint val) { return atomicAdd(p, val); }
inline __device__ uint gputilAtomicSub(atomic_uint *p, uint val) { return atomicSub(p, val); }
inline __device__ uint gputilAtomicInc(atomic_uint *p) { return atomicAdd(p, 1); }
inline __device__ uint gputilAtomicDec(atomic_uint *p) { return atomicSub(p, 1); }
inline __device__ uint gputilAtomicMin(atomic_uint *p, uint val) { return atomicMin(p, val); }
inline __device__ uint gputilAtomicMax(atomic_uint *p, uint val) { return atomicMax(p, val); }
inline __device__ uint gputilAtomicAnd(atomic_uint *p, uint val) { return atomicAnd(p, val); }
inline __device__ uint gputilAtomicOr(atomic_uint *p, uint val) { return atomicOr(p, val); }
inline __device__ uint gputilAtomicXor(atomic_uint *p, uint val) { return atomicXor(p, val); }

#endif // __CUDACC__

#endif // CUTIL_ATOMIC_H
