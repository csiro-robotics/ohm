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
// - Use __local as a prefix to local memory arguments in OpenCL. Removed in CUDA
// - Use local for local memory delcarations. Converted to __shared__ in CUDA.
//   This is then removed in CUDA compilation as it is not needed. __local already has meaning
//   in CUDA so we couldn't just use that.

#ifndef GPU_EXT_H_
#define GPU_EXT_H_

#define GPUTIL_NULL 0
#define GPUTIL_OPENCL 1
#define GPUTIL_CUDA 2

// Setup GPUTIL_DEVICE. This has a non-zero value only when building device code.
#if defined(__OPENCL_C_VERSION__)
#define GPUTIL_DEVICE GPUTIL_OPENCL
#elif defined(__CUDACC__)
#define GPUTIL_DEVICE GPUTIL_CUDA
#else
#define GPUTIL_DEVICE GPUTIL_NULL
#endif

#if GPUTIL_DEVICE == GPUTIL_OPENCL
#endif // GPUTIL_DEVICE == GPUTIL_OPENCL

#if GPUTIL_DEVICE == GPUTIL_OPENCL
#ifndef NULL
#define NULL 0
#endif  // !NULL

#define LOCAL_ARG(TYPE, VAR) \
  , __local TYPE VAR
#define LOCAL_VAR(TYPE, VAR, SIZE)
#define LOCAL_MEM_ENABLE()
#define LM_PER_THREAD(PER_THREAD_SIZE)

//-----------------------------------------------------------------------------
// OpenCL defines
//-----------------------------------------------------------------------------
#define __device__
#define __host__

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

//----------------------------------------------------------------------------------------------------------------------
// Atomic definitions. Make consistent atomic function interface across all GPU code versions.
//
// Only 32-bit types are supported.
//----------------------------------------------------------------------------------------------------------------------
#if __OPENCL_C_VERSION__ >= 210

inline void gputilAtomicInitI32(__global atomic_int *obj, int val) { atomic_init(obj, val); }
inline void gputilAtomicInitI32(__local atomic_int *obj, int val) { atomic_init(obj, val); }
inline void gputilAtomicStoreI32(__global atomic_int *obj, int val) { atomic_store(obj, val); }
inline void gputilAtomicStoreI32(__local atomic_int *obj, int val) { atomic_store(obj, val); }
inline int gputilAtomicLoadI32(__global atomic_int *obj) { return atomic_load(obj); }
inline int gputilAtomicLoadI32(__local atomic_int *obj) { return atomic_load(obj); }
inline int gputilAtomicExchangeI32(__global atomic_int *obj, int desired) { return atomic_exchange(obj, desired); }
inline int gputilAtomicExchangeI32(__local atomic_int *obj, int desired) { return atomic_exchange(obj, desired); }
inline bool gputilAtomicCasI32(__global atomic_int *obj, int expected, int desired) { return atomic_compare_exchange_weak(obj, &expected, desired); }
inline bool gputilAtomicCasI32(__local atomic_int *obj, int expected, int desired) { return atomic_compare_exchange_weak(obj, &expected, desired); }

inline void gputilAtomicInitU32(__global atomic_uint *obj, uint val) { atomic_init(obj, val); }
inline void gputilAtomicInitU32(__local atomic_uint *obj, uint val) { atomic_init(obj, val); }
inline void gputilAtomicStoreU32(__global atomic_uint *obj, uint val) { atomic_store(obj, val); }
inline void gputilAtomicStoreU32(__local atomic_uint *obj, uint val) { atomic_store(obj, val); }
inline uint gputilAtomicLoadU32(__global atomic_uint *obj) { return atomic_load(obj); }
inline uint gputilAtomicLoadU32(__local atomic_uint *obj) { return atomic_load(obj); }
inline uint gputilAtomicExchangeU32(__global atomic_uint *obj, uint desired) { return atomic_exchange(obj, desired); }
inline uint gputilAtomicExchangeU32(__local atomic_uint *obj, uint desired) { return atomic_exchange(obj, desired); }
inline bool gputilAtomicCasU32(__global atomic_uint *obj, uint expected, uint desired) { return atomic_compare_exchange_weak(obj, &expected, desired); }
inline bool gputilAtomicCasU32(__local atomic_uint *obj, uint expected, uint desired) { return atomic_compare_exchange_weak(obj, &expected, desired); }

inline void gputilAtomicInitF32(__global atomic_float *obj, float val) { atomic_init(obj, val); }
inline void gputilAtomicInitF32(__local atomic_float *obj, float val) { atomic_init(obj, val); }
inline void gputilAtomicStoreF32(__global atomic_float *obj, float val) { atomic_store(obj, val); }
inline void gputilAtomicStoreF32(__local atomic_float *obj, float val) { atomic_store(obj, val); }
inline float gputilAtomicLoadF32(__global atomic_float *obj) { return atomic_load(obj); }
inline float gputilAtomicLoadF32(__local atomic_float *obj) { return atomic_load(obj); }
inline float gputilAtomicExchangeF32(__global atomic_float *obj, float desired) { return atomic_exchange(obj, desired); }
inline float gputilAtomicExchangeF32(__local atomic_float *obj, float desired) { return atomic_exchange(obj, desired); }
inline bool gputilAtomicCasF32(__global atomic_float *obj, float expected, float desired) { return atomic_compare_exchange_weak(obj, &expected, desired); }
inline bool gputilAtomicCasF32(__local atomic_float *obj, float expected, float desired) { return atomic_compare_exchange_weak(obj, &expected, desired); }

#else  // __OPENCL_C_VERSION__ < 210
typedef volatile int atomic_int;
typedef volatile uint atomic_uint;
typedef volatile float atomic_float;

#define gputilAtomicInitI32(obj, val) *(obj) = val
#define gputilAtomicStoreI32(obj, val) *(obj) = val
#define gputilAtomicLoadI32(obj) *(obj)
#define gputilAtomicExchangeI32(obj, desired) atomic_xchg(obj, desired)
#define gputilAtomicCas(obj, expected, desired) (atomic_cmpxchg(obj, expected, desired) == (expected))

#define gputilAtomicInitF32(obj, val) *(obj) = val
#define gputilAtomicStoreF32(obj, val) *(obj) = val
#define gputilAtomicLoadF32(obj) *(obj)
#define gputilAtomicExchangeF32L(obj, desired) atomic_xchg((__local atomic_int *)obj, *(int *)&desired)
#define gputilAtomicExchangeF32(obj, desired) atomic_xchg((__global atomic_int *)obj, *(int *)&desired)

#define gputilAtomicCasF32L(obj, expected,  desired) (atomic_cmpxchg((__local atomic_int *)obj, *(int *)&expected, *(int *)&desired) == *(int *)&expected)
#define gputilAtomicCasF32(obj, expected,  desired) (atomic_cmpxchg((__global atomic_int *)obj, *(int *)&expected, *(int *)&desired) == *(int *)&expected)

#endif // __OPENCL_C_VERSION__

#define gputilAtomicAdd(p, val) atomic_add(p, val)
#define gputilAtomicSub(p, val) atomic_sub(p, val)
#define gputilAtomicInc(p) atomic_inc(p)
#define gputilAtomicDec(p) atomic_dec(p)
#define gputilAtomicMin(p, val) atomic_min(p, val)
#define gputilAtomicMax(p, val) atomic_max(p, val)
#define gputilAtomicAnd(p, val) atomic_and(p, val)
#define gputilAtomicOr(p, val) atomic_or(p, val)
#define gputilAtomicXor(p, val) atomic_xor(p, val)

#endif  // GPUTIL_DEVICE == GPUTIL_OPENCL

//----------------------------------------------------------------------------------------------------------------------
// Utility/helper functions.
//----------------------------------------------------------------------------------------------------------------------
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
