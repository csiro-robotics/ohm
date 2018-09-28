// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPLATFORM2_H_
#define GPUPLATFORM2_H_

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif // !__APPLE__

namespace gputil
{
  typedef cl_char char1;
  typedef cl_char2 char2;
  typedef cl_char3 char3;
  typedef cl_char4 char4;
  typedef cl_uchar uchar1;
  typedef cl_uchar2 uchar2;
  typedef cl_uchar3 uchar3;
  typedef cl_uchar4 uchar4;
  typedef cl_short2 short2;
  typedef cl_short3 short3;
  typedef cl_short4 short4;
  typedef cl_ushort ushort1;
  typedef cl_ushort2 ushort2;
  typedef cl_ushort3 ushort3;
  typedef cl_ushort4 ushort4;
  typedef cl_int int1;
  typedef cl_int2 int2;
  typedef cl_int3 int3;
  typedef cl_int4 int4;
  typedef cl_uint uint1;
  typedef cl_uint2 uint2;
  typedef cl_uint3 uint3;
  typedef cl_uint4 uint4;
  typedef cl_long long1;
  typedef cl_long2 long2;
  typedef cl_long3 long3;
  typedef cl_long4 long4;
  typedef cl_ulong ulong1;
  typedef cl_ulong2 ulong2;
  typedef cl_ulong3 ulong3;
  typedef cl_ulong4 ulong4;
  typedef cl_float float1;
  typedef cl_float2 float2;
  typedef cl_float3 float3;
  typedef cl_float4 float4;
  typedef cl_double double1;
  typedef cl_double2 double2;
}

#endif // GPUPLATFORM2_H_
