// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPLATFORM2_H
#define GPUPLATFORM2_H

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif  // !__APPLE__

namespace gputil
{
  typedef cl_char char1;       // NOLINT
  typedef cl_char2 char2;      // NOLINT
  typedef cl_char3 char3;      // NOLINT
  typedef cl_char4 char4;      // NOLINT
  typedef cl_uchar uchar;      // NOLINT
  typedef cl_uchar uchar1;     // NOLINT
  typedef cl_uchar2 uchar2;    // NOLINT
  typedef cl_uchar3 uchar3;    // NOLINT
  typedef cl_uchar4 uchar4;    // NOLINT
  typedef short short1;        // NOLINT
  typedef cl_short2 short2;    // NOLINT
  typedef cl_short3 short3;    // NOLINT
  typedef cl_short4 short4;    // NOLINT
  typedef cl_ushort ushort;    // NOLINT
  typedef cl_ushort ushort1;   // NOLINT
  typedef cl_ushort2 ushort2;  // NOLINT
  typedef cl_ushort3 ushort3;  // NOLINT
  typedef cl_ushort4 ushort4;  // NOLINT
  typedef cl_int int1;         // NOLINT
  typedef cl_int2 int2;        // NOLINT
  typedef cl_int3 int3;        // NOLINT
  typedef cl_int4 int4;        // NOLINT
  typedef cl_uint uint;        // NOLINT
  typedef cl_uint uint1;       // NOLINT
  typedef cl_uint2 uint2;      // NOLINT
  typedef cl_uint3 uint3;      // NOLINT
  typedef cl_uint4 uint4;      // NOLINT
  typedef cl_long long1;       // NOLINT
  typedef cl_long2 long2;      // NOLINT
  typedef cl_long3 long3;      // NOLINT
  typedef cl_long4 long4;      // NOLINT
  typedef cl_ulong ulong;      // NOLINT
  typedef cl_ulong ulong1;     // NOLINT
  typedef cl_ulong2 ulong2;    // NOLINT
  typedef cl_ulong3 ulong3;    // NOLINT
  typedef cl_ulong4 ulong4;    // NOLINT
  typedef cl_float float1;     // NOLINT
  typedef cl_float2 float2;    // NOLINT
  typedef cl_float3 float3;    // NOLINT
  typedef cl_float4 float4;    // NOLINT
  typedef cl_double double1;   // NOLINT
  typedef cl_double2 double2;  // NOLINT
}  // namespace gputil

#endif  // GPUPLATFORM2_H
