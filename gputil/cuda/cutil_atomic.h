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

enum memory_order : int
{
  memory_order_relaxed,
  memory_order_acquire,
  memory_order_release,
  memory_order_acq_rel,
  memory_order_seq_cst,
};

template <typename T>
struct AtomicArg
{
  using Type = T;
  using ArgType = T;
  static inline __device__ T *arg(T *val) { return val; }
  static inline __device__ T arg(const T &val) { return val; }
  static inline __device__ T val(T *val) { return *val; }
  static inline __device__ T val(const T &val) { return val; }
};

template <typename T>
struct AtomicArg<volatile T>
{
  using Type = T;
  using ArgType = T;
  static inline __device__ T *arg(volatile T *val) { return (T *)val; }
  static inline __device__ T arg(const volatile T &val) { return val; }
  static inline __device__ T val(volatile T *val) { return *val; }
  static inline __device__ T val(const volatile T &val) { return val; }
};

template <>
struct AtomicArg<float>
{
  using Type = float;
  using ArgType = int;
  static inline __device__ int *arg(float *val) { return (int *)val; }
  static inline __device__ int arg(const float &val) { return *(const int *)&val; }
  static inline __device__ float val(float *val) { return *val; }
  static inline __device__ float val(const float &val) { return val; }
};

template <>
struct AtomicArg<volatile float>
{
  using Type = float;
  using ArgType = int;
  static inline __device__ int *arg(volatile float *val) { return (int *)val; }
  static inline __device__ int arg(const volatile float &val) { return *(const int *)&val; }
  static inline __device__ float val(volatile float *val) { return *val; }
  static inline __device__ float val(const volatile float &val) { return val; }
};

template <>
struct AtomicArg<double>
{
  using Type = double;
  using ArgType = unsigned long long int;
  static inline __device__ unsigned long long int *arg(double *val) { return (unsigned long long int *)val; }
  static inline __device__ unsigned long long int arg(const double &val) { return *(const unsigned long long int *)&val; }
  static inline __device__ double val(double *val) { return *val; }
  static inline __device__ double val(const double &val) { return val; }
};

template <>
struct AtomicArg<volatile double>
{
  using Type = double;
  using ArgType = unsigned long long int;
  static inline __device__ unsigned long long int *arg(volatile double *val) { return (unsigned long long int *)val; }
  static inline __device__ unsigned long long int arg(const volatile double &val) { return *(const unsigned long long int *)&val; }
  static inline __device__ double val(volatile double *val) { return *val; }
  static inline __device__ double val(const volatile double &val) { return val; }
};

template <typename T, typename V>
inline __device__ void atomic_init(T *obj, V value)
{
  *obj = value;
}

template <typename T, typename V>
inline __device__ void atomic_store(T *obj, V desired, memory_order)
{
  *obj = desired;
}

template <typename T>
inline __device__ typename AtomicArg<T>::Type atomic_load(T *obj)
{
  return *obj;
}

template <typename T>
inline __device__ typename AtomicArg<T>::Type atomic_load_explicit(T *obj, memory_order /* ignored */)
{
  return *obj;
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::Type atomic_add(T *obj, V val)
{
  return atomicAdd(AtomicArg<T>::arg(obj), AtomicArg<V>::val(val));
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::Type atomic_sub(T *obj, V val)
{
  return atomicSub(AtomicArg<T>::arg(obj), AtomicArg<V>::val(val));
}

template <typename T>
inline __device__ typename AtomicArg<T>::Type atomic_inc(T *obj)
{
  // OpenCL/CUDA semantics differ slightly. Use atomicAdd().
  return atomicAdd(AtomicArg<T>::arg(obj), 1);
}

template <typename T>
inline __device__ typename AtomicArg<T>::Type atomic_dec(T *obj)
{
  // OpenCL/CUDA semantics differ slightly. Use atomicSub().
  return atomicSub(AtomicArg<T>::arg(obj), 1);
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::Type atomic_exchange(T *obj, V desired)
{
  return atomicExch(AtomicArg<T>::arg(obj), AtomicArg<V>::val(desired));
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::Type atomic_min(T *obj, V val)
{
  return atomicMin(AtomicArg<T>::arg(obj), AtomicArg<V>::val(val));
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::T atomic_max(T *obj, V val)
{
  return atomicMax(AtomicArg<T>::arg(obj), AtomicArg<V>::val(val));
}

template <typename T, typename V>
inline __device__ bool atomic_compare_exchange_weak(T *obj, V *expected, V desired)
{
  return atomicCAS(AtomicArg<T>::arg(obj), AtomicArg<V>::val(expected), AtomicArg<V>::val(desired)) == AtomicArg<V>::val(expected);
}

template <typename T, typename V>
inline __device__ bool atomic_compare_exchange_weak_explicit(T *obj, V *expected, V desired, memory_order /*success*/, memory_order /*failure*/)
{
  return atomicCAS(AtomicArg<T>::arg(obj), AtomicArg<V>::val(expected), AtomicArg<V>::val(desired)) == AtomicArg<V>::val(expected);
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::Type atomic_and(T *obj, V val)
{
  return atomicAnd(AtomicArg<T>::arg(obj), AtomicArg<V>::val(val));
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::Type atomic_or(T *obj, V val)
{
  return atomicOr(AtomicArg<T>::arg(obj), AtomicArg<V>::val(val));
}

template <typename T, typename V>
inline __device__ typename AtomicArg<T>::Type atomic_xor(T *obj, V val)
{
  return atomicXor(AtomicArg<T>::arg(obj), AtomicArg<V>::val(val));
}

#endif // __CUDACC__

#endif // CUTIL_ATOMIC_H
