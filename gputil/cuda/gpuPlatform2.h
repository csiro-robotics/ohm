// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPLATFORM2_H
#define GPUPLATFORM2_H

#include <cuda_runtime.h>

namespace gputil
{
  using char1 = ::char1;          // NOLINT
  using char2 = ::char2;          // NOLINT
  using char3 = ::char3;          // NOLINT
  using char4 = ::char4;          // NOLINT
  using uchar = unsigned char;    // NOLINT
  using uchar1 = ::uchar1;        // NOLINT
  using uchar2 = ::uchar2;        // NOLINT
  using uchar3 = ::uchar3;        // NOLINT
  using uchar4 = ::uchar4;        // NOLINT
  using short1 = ::short1;        // NOLINT
  using short2 = ::short2;        // NOLINT
  using short3 = ::short3;        // NOLINT
  using short4 = ::short4;        // NOLINT
  using ushort = unsigned short;  // NOLINT
  using ushort1 = ::ushort1;      // NOLINT
  using ushort2 = ::ushort2;      // NOLINT
  using ushort3 = ::ushort3;      // NOLINT
  using ushort4 = ::ushort4;      // NOLINT
  using int1 = ::int1;            // NOLINT
  using int2 = ::int2;            // NOLINT
  using int3 = ::int3;            // NOLINT
  using int4 = ::int4;            // NOLINT
  using uint = unsigned int;      // NOLINT
  using uint1 = ::uint1;          // NOLINT
  using uint2 = ::uint2;          // NOLINT
  using uint3 = ::uint3;          // NOLINT
  using uint4 = ::uint4;          // NOLINT
  using long1 = ::longlong1;      // NOLINT
  using long2 = ::longlong2;      // NOLINT
  using long3 = ::longlong3;      // NOLINT
  using long4 = ::longlong4;      // NOLINT
  using ulong1 = ::ulonglong1;    // NOLINT
  using ulong2 = ::ulonglong2;    // NOLINT
  using ulong3 = ::ulonglong3;    // NOLINT
  using ulong4 = ::ulonglong4;    // NOLINT
  using float1 = ::float1;        // NOLINT
  using float2 = ::float2;        // NOLINT
  using float3 = ::float3;        // NOLINT
  using float4 = ::float4;        // NOLINT
  using double1 = ::double1;      // NOLINT
  using double2 = ::double2;      // NOLINT
}  // namespace gputil

#endif  // GPUPLATFORM2_H
