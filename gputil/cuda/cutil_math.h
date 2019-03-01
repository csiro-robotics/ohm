// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CUTIL_MATH_H
#define CUTIL_MATH_H

#include <cuda_runtime.h>

#include <cmath>

using uchar = unsigned char;
using ushort = unsigned short;
using uint = unsigned int;
using ulong = unsigned long long int;

//----------------------------------------------------------------------------------------------------------------------
// Basic vector operations.
//----------------------------------------------------------------------------------------------------------------------
#define _VECTOR_UOPS(type)                                                                                           \
  inline __host__ __device__ bool operator==(const type##2 & a, const type##2 & b)                                   \
  {                                                                                                                  \
    return a.x == b.x && a.y == b.y;                                                                                 \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ bool operator!=(const type##2 & a, const type##2 & b) { return !operator==(a, b); }     \
                                                                                                                     \
  inline __host__ __device__ type##2 operator+(const type##2 & a, const type##2 & b)                                 \
  {                                                                                                                  \
    return make_##type##2(a.x + b.x, a.y + b.y);                                                                     \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 operator+(const type##2 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##2(a.x + scalar, a.y + scalar);                                                               \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##2 operator-(const type##2 & a, const type##2 & b)                                 \
  {                                                                                                                  \
    return make_##type##2(a.x - b.x, a.y - b.y);                                                                     \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 operator-(const type##2 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##2(a.x - scalar, a.y - scalar);                                                               \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##2 &operator+=(type##2 & a, const type##2 & b)                                     \
  {                                                                                                                  \
    a.x += b.x;                                                                                                      \
    a.y += b.y;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 &operator+=(type##2 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x += scalar;                                                                                                   \
    a.y += scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##2 &operator-=(type##2 & a, const type##2 & b)                                     \
  {                                                                                                                  \
    a.x -= b.x;                                                                                                      \
    a.y -= b.y;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 &operator-=(type##2 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x -= scalar;                                                                                                   \
    a.y -= scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##2 operator*(const type##2 & a, const type##2 & b)                                 \
  {                                                                                                                  \
    return make_##type##2(a.x * b.x, a.y * b.y);                                                                     \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##2 &operator*=(type##2 & a, const type##2 & b)                                     \
  {                                                                                                                  \
    a.x *= b.x;                                                                                                      \
    a.y *= b.y;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 operator*(const type##2 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##2(a.x * scalar, a.y * scalar);                                                               \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 operator*(const S &scalar, const type##2 & a)                                   \
  {                                                                                                                  \
    return make_##type##2(a.x * scalar, a.y * scalar);                                                               \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 &operator*=(type##2 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x *= scalar;                                                                                                   \
    a.y *= scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##2 operator/(const type##2 & a, const type##2 & b)                                 \
  {                                                                                                                  \
    return make_##type##2(a.x / b.x, a.y / b.y);                                                                     \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##2 &operator/=(type##2 & a, const type##2 & b)                                     \
  {                                                                                                                  \
    a.x /= b.x;                                                                                                      \
    a.y /= b.y;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 operator/(const type##2 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##2(a.x / scalar, a.y / scalar);                                                               \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##2 &operator/=(type##2 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x /= scalar;                                                                                                   \
    a.y /= scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ bool operator==(const type##3 & a, const type##3 & b)                                   \
  {                                                                                                                  \
    return a.x == b.x && a.y == b.y && a.z == b.z;                                                                   \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ bool operator!=(const type##3 & a, const type##3 & b) { return !operator==(a, b); }     \
                                                                                                                     \
  inline __host__ __device__ type##3 operator+(const type##3 & a, const type##3 & b)                                 \
  {                                                                                                                  \
    return make_##type##3(a.x + b.x, a.y + b.y, a.z + b.z);                                                          \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##3 operator-(const type##3 & a, const type##3 & b)                                 \
  {                                                                                                                  \
    return make_##type##3(a.x - b.x, a.y - b.y, a.z - b.z);                                                          \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 operator+(const type##3 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##3(a.x + scalar, a.y + scalar, a.z + scalar);                                                 \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 operator-(const type##3 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##3(a.x - scalar, a.y - scalar, a.z - scalar);                                                 \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##3 &operator+=(type##3 & a, const type##3 & b)                                     \
  {                                                                                                                  \
    a.x += b.x;                                                                                                      \
    a.y += b.y;                                                                                                      \
    a.z += b.z;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##3 &operator-=(type##3 & a, const type##3 & b)                                     \
  {                                                                                                                  \
    a.x -= b.x;                                                                                                      \
    a.y -= b.y;                                                                                                      \
    a.z -= b.z;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 &operator+=(type##3 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x += scalar;                                                                                                   \
    a.y += scalar;                                                                                                   \
    a.z += scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 &operator-=(type##3 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x -= scalar;                                                                                                   \
    a.y -= scalar;                                                                                                   \
    a.z -= scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##3 operator*(const type##3 & a, const type##3 & b)                                 \
  {                                                                                                                  \
    return make_##type##3(a.x * b.x, a.y * b.y, a.z * b.z);                                                          \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##3 &operator*=(type##3 & a, const type##3 & b)                                     \
  {                                                                                                                  \
    a.x *= b.x;                                                                                                      \
    a.y *= b.y;                                                                                                      \
    a.z *= b.z;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 operator*(const type##3 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##3(a.x * scalar, a.y * scalar, a.z * scalar);                                                 \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 operator*(const S &scalar, const type##3 & a)                                   \
  {                                                                                                                  \
    return make_##type##3(a.x * scalar, a.y * scalar, a.z * scalar);                                                 \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 &operator*=(type##3 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x *= scalar;                                                                                                   \
    a.y *= scalar;                                                                                                   \
    a.z *= scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##3 operator/(const type##3 & a, const type##3 & b)                                 \
  {                                                                                                                  \
    return make_##type##3(a.x / b.x, a.y / b.y, a.z / b.z);                                                          \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##3 &operator/=(type##3 & a, const type##3 & b)                                     \
  {                                                                                                                  \
    a.x /= b.x;                                                                                                      \
    a.y /= b.y;                                                                                                      \
    a.z /= b.z;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 operator/(const type##3 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##3(a.x / scalar, a.y / scalar, a.z / scalar);                                                 \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##3 &operator/=(type##3 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x /= scalar;                                                                                                   \
    a.y /= scalar;                                                                                                   \
    a.z /= scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ bool operator==(const type##4 & a, const type##4 & b)                                   \
  {                                                                                                                  \
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;                                                     \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ bool operator!=(const type##4 & a, const type##4 & b) { return !operator==(a, b); }     \
                                                                                                                     \
  inline __host__ __device__ type##4 operator+(const type##4 & a, const type##4 & b)                                 \
  {                                                                                                                  \
    return make_##type##4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);                                               \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##4 operator-(const type##4 & a, const type##4 & b)                                 \
  {                                                                                                                  \
    return make_##type##4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);                                               \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##4 operator+(const type##4 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##4(a.x + scalar, a.y + scalar, a.z + scalar, a.w + scalar);                                   \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##4 operator-(const type##4 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##4(a.x - scalar, a.y - scalar, a.z - scalar, a.w - scalar);                                   \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##4 &operator+=(type##4 & a, const type##4 & b)                                     \
  {                                                                                                                  \
    a.x += b.x;                                                                                                      \
    a.y += b.y;                                                                                                      \
    a.z += b.z;                                                                                                      \
    a.w += b.w;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##4 &operator-=(type##4 & a, const type##4 & b)                                     \
  {                                                                                                                  \
    a.x -= b.x;                                                                                                      \
    a.y -= b.y;                                                                                                      \
    a.z -= b.z;                                                                                                      \
    a.w -= b.w;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##4 operator*(const type##4 & a, const type##4 & b)                                 \
  {                                                                                                                  \
    return make_##type##4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);                                               \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##4 &operator*=(type##4 & a, const type##4 & b)                                     \
  {                                                                                                                  \
    a.x *= b.x;                                                                                                      \
    a.y *= b.y;                                                                                                      \
    a.z *= b.z;                                                                                                      \
    a.w *= b.w;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##4 operator*(const type##4 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##4(a.x * scalar, a.y * scalar, a.z * scalar, a.w * scalar);                                   \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##4 operator*(const S &scalar, const type##4 & a)                                   \
  {                                                                                                                  \
    return make_##type##4(a.x * scalar, a.y * scalar, a.z * scalar, a.w * scalar);                                   \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##4 &operator*=(type##4 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x *= scalar;                                                                                                   \
    a.y *= scalar;                                                                                                   \
    a.z *= scalar;                                                                                                   \
    a.w *= scalar;                                                                                                   \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##4 operator/(const type##4 & a, const type##4 & b)                                 \
  {                                                                                                                  \
    return make_##type##4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);                                               \
  }                                                                                                                  \
                                                                                                                     \
  inline __host__ __device__ type##4 &operator/=(type##4 & a, const type##4 & b)                                     \
  {                                                                                                                  \
    a.x /= b.x;                                                                                                      \
    a.y /= b.y;                                                                                                      \
    a.z /= b.z;                                                                                                      \
    a.w /= b.w;                                                                                                      \
    return a;                                                                                                        \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##4 operator/(const type##4 & a, const S &scalar)                                   \
  {                                                                                                                  \
    return make_##type##4(a.x / scalar, a.y / scalar, a.z / scalar, a.w / scalar);                                   \
  }                                                                                                                  \
                                                                                                                     \
  template <typename S>                                                                                              \
  inline __host__ __device__ type##4 &operator/=(type##4 & a, const S &scalar)                                       \
  {                                                                                                                  \
    a.x /= scalar;                                                                                                   \
    a.y /= scalar;                                                                                                   \
    a.z /= scalar;                                                                                                   \
    a.w /= scalar;                                                                                                   \
    return a;                                                                                                        \
  }

#define _VECTOR_OPS(type) \
  _VECTOR_UOPS(type) \
  inline __host__ __device__ type##2 operator-(const type##2 & a) { return make_##type##2(-a.x, -a.y); }             \
  inline __host__ __device__ type##3 operator-(const type##3 & a) { return make_##type##3(-a.x, -a.y, -a.z); }       \
  inline __host__ __device__ type##4 operator-(const type##4 & a) { return make_##type##4(-a.x, -a.y, -a.z, -a.w); }


_VECTOR_OPS(char);
_VECTOR_UOPS(uchar);
_VECTOR_OPS(short);
_VECTOR_UOPS(ushort);
_VECTOR_OPS(int);
_VECTOR_UOPS(uint);
_VECTOR_OPS(long);
_VECTOR_UOPS(ulong);
_VECTOR_OPS(float);
_VECTOR_OPS(double);

//----------------------------------------------------------------------------------------------------------------------
// OpenCL equivalent extensions
//----------------------------------------------------------------------------------------------------------------------
#define _VECTOR_LOGIC(type, itype)                                                                           \
  inline __device__ __host__ itype##2 isequal(const type##2 & a, const type##2 & b)                          \
  {                                                                                                          \
    return make_##itype##2((a.x == b.x) ? -1 : 0, (a.y == b.y) ? -1 : 0);                                    \
  }                                                                                                          \
                                                                                                             \
  inline __device__ __host__ itype##3 isequal(const type##3 & a, const type##3 & b)                          \
  {                                                                                                          \
    return make_##itype##3((a.x == b.x) ? -1 : 0, (a.y == b.y) ? -1 : 0, (a.z == b.z) ? -1 : 0);             \
  }                                                                                                          \
                                                                                                             \
  inline __device__ __host__ itype##4 isequal(const type##4 & a, const type##4 & b)                          \
  {                                                                                                          \
    return make_##itype##4((a.x == b.x) ? -1 : 0, (a.y == b.y) ? -1 : 0, (a.z == b.z) ? -1 : 0,              \
                           (a.w == b.w) ? -1 : 0);                                                           \
  }                                                                                                          \
                                                                                                             \
  inline __device__ __host__ int any(const type##2 & x) { return (x.x != 0 || x.y != 0) ? 1 : 0; }           \
                                                                                                             \
  inline __device__ __host__ int any(const type##3 x) { return (x.x != 0 || x.y != 0 || x.z != 0) ? 1 : 0; } \
                                                                                                             \
  inline __device__ __host__ int any(const type##4 & x)                                                      \
  {                                                                                                          \
    return (x.x != 0 || x.y != 0 || x.z != 0 || x.w != 0) ? 1 : 0;                                           \
  }                                                                                                          \
                                                                                                             \
  inline __device__ __host__ int all(type##2 x) { return (x.x != 0 && x.y != 0) ? 1 : 0; }                   \
                                                                                                             \
  inline __device__ __host__ int all(type##3 x) { return (x.x != 0 && x.y != 0 && x.z != 0) ? 1 : 0; }       \
                                                                                                             \
  inline __device__ __host__ int all(type##4 x) { return (x.x != 0 && x.y != 0 && x.z != 0 && x.w != 0) ? 1 : 0; }

_VECTOR_LOGIC(char, int);
_VECTOR_LOGIC(uchar, int);
_VECTOR_LOGIC(short, int);
_VECTOR_LOGIC(ushort, int);
_VECTOR_LOGIC(int, int);
_VECTOR_LOGIC(uint, int);
_VECTOR_LOGIC(long, long);
_VECTOR_LOGIC(ulong, long);
_VECTOR_LOGIC(float, int);
_VECTOR_LOGIC(double, long);

//----------------------------------------------------------------------------------------------------------------------
// Vector product
//----------------------------------------------------------------------------------------------------------------------

#define _VECTOR_GEOM(type)                                                                                      \
  inline __host__ __device__ type dot(const type##2 & a, const type##2 & b) { return a.x * b.x + a.y * b.y; }   \
                                                                                                                \
  inline __host__ __device__ type dot(const type##3 & a, const type##3 & b)                                     \
  {                                                                                                             \
    return a.x * b.x + a.y * b.y + a.z * b.z;                                                                   \
  }                                                                                                             \
                                                                                                                \
  inline __host__ __device__ type dot(const type##4 & a, const type##4 & b)                                     \
  {                                                                                                             \
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;                                                       \
  }                                                                                                             \
                                                                                                                \
  inline __host__ __device__ type length(const type##2 & a) { return std::sqrt(dot(a, a)); }                    \
                                                                                                                \
  inline __host__ __device__ type length(const type##3 & a) { return std::sqrt(dot(a, a)); }                    \
                                                                                                                \
  inline __host__ __device__ type length(const type##4 & a) { return std::sqrt(dot(a, a)); }                    \
                                                                                                                \
  inline __host__ __device__ type##2 normalize(const type##2 & a) { return a * (1.0f / std::sqrt(dot(a, a))); } \
                                                                                                                \
  inline __host__ __device__ type##3 normalize(const type##3 & a) { return a * (1.0f / std::sqrt(dot(a, a))); } \
                                                                                                                \
  inline __host__ __device__ type##4 normalize(const type##4 & a) { return a * (1.0f / std::sqrt(dot(a, a))); } \
                                                                                                                \
  inline __host__ __device__ type##3 cross(type##3 a, type##3 b)                                                \
  {                                                                                                             \
    return make_##type##3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);                 \
  }

_VECTOR_GEOM(float)
_VECTOR_GEOM(double)

//----------------------------------------------------------------------------------------------------------------------
// Conversion
//----------------------------------------------------------------------------------------------------------------------
#define _VECTOR_CONVERT(to, from)                                                                                \
  inline __host__ __device__ to##2 convert_##to##2(const from##2 & a) { return make_##to##2(to(a.x), to(a.y)); } \
                                                                                                                 \
  inline __host__ __device__ to##3 convert_##to##3(const from##3 & a)                                            \
  {                                                                                                              \
    return make_##to##3(to(a.x), to(a.y), to(a.z));                                                              \
  }                                                                                                              \
                                                                                                                 \
  inline __host__ __device__ to##4 convert_##to##4(const from##4 & a)                                            \
  {                                                                                                              \
    return make_##to##4(to(a.x), to(a.y), to(a.z), to(a.w));                                                     \
  }

#define _VECTOR_CONVERT_SET(to) \
  _VECTOR_CONVERT(to, char)     \
  _VECTOR_CONVERT(to, uchar)    \
  _VECTOR_CONVERT(to, short)    \
  _VECTOR_CONVERT(to, ushort)   \
  _VECTOR_CONVERT(to, int)      \
  _VECTOR_CONVERT(to, uint)     \
  _VECTOR_CONVERT(to, float)    \
  _VECTOR_CONVERT(to, double)

_VECTOR_CONVERT_SET(char)
_VECTOR_CONVERT_SET(uchar)
_VECTOR_CONVERT_SET(short)
_VECTOR_CONVERT_SET(ushort)
_VECTOR_CONVERT_SET(int)
_VECTOR_CONVERT_SET(uint)
_VECTOR_CONVERT_SET(float)
_VECTOR_CONVERT_SET(double)

//----------------------------------------------------------------------------------------------------------------------
// Utility
//----------------------------------------------------------------------------------------------------------------------
template <typename T>
inline __host__ __device__ T clamp(const T &val, const T &min_val, const T &max_val)
{
  return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}


#endif  // CUTIL_MATH_H
