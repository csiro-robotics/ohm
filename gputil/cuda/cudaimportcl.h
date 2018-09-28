// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CUDA_IMPORT_CL_H_
#define CUDA_IMPORT_CL_H_

// Kernel
#define __kernel __global__

// Memory
#define __constant __constant__
#define __global
#define __local __shared__

// Useful information:
// https://www.sharcnet.ca/help/index.php/Porting_CUDA_to_OpenCL

/*

Hardware Terminology
CUDA                          OpenCL
SM (Stream Multiprocessor)    CU (Compute Unit)
Thread                        Work-item
Block                         Work-group
Global memory                 Global memory
Constant memory               Constant memory
Shared memory                 Local memory
Local memory                  Private memory


Qualifiers for Kernel Functions
CUDA                                OpenCL
__constant__ variable declaration   __constant variable declaration
__device__ function                 No annotation necessary
__device__ variable declaration     __global variable declaration
__global__ function                 __kernel function
__shared__ variable declaration     __local variable declaration


Kernels Indexing
CUDA                              OpenCL
gridDim                           get_num_groups()
blockDim                          get_local_size()
blockIdx                          get_group_id()
threadIdx                         get_local_id()
blockIdx * blockDim + threadIdx   get_global_id()
gridDim * blockDim                get_global_size()
CUDA is using threadIdx.x to get the id for the first dimension while OpenCL is using get_local_id(0).


Kernels Synchronization
CUDA                    OpenCL
__syncthreads()         barrier(CLK_LOCAL_MEM_FENCE)
__threadfence()         No direct equivalent
__threadfence_block()   mem_fence()
No direct equivalent    read_mem_fence()
No direct equivalent    write_mem_fence()

*/

// Synchronisation.
#define barrier(...) __syncthreads()
#define mem_fence __threadfence_block

typedef unsigned char uchar;
typedef unsigned int uint;

inline __device__ unsigned get_num_groups(int i)
{
  switch (i)
  {
  default:
  case 0:
    return gridDim.x;
  case 1:
    return gridDim.y;
  case 2:
    return gridDim.z;
  }
}

inline __device__ unsigned get_local_size(int i)
{
  switch (i)
  {
  default:
  case 0:
    return blockDim.x;
  case 1:
    return blockDim.y;
  case 2:
    return blockDim.z;
  }
}

inline __device__ unsigned get_group_id(int i)
{
  switch (i)
  {
  default:
  case 0:
    return blockIdx.x;
  case 1:
    return blockIdx.y;
  case 2:
    return blockIdx.z;
  }
}

inline __device__ unsigned get_local_id(int i)
{
  switch (i)
  {
  default:
  case 0:
    return threadIdx.x;
  case 1:
    return threadIdx.y;
  case 2:
    return threadIdx.z;
  }
}

inline __device__ unsigned get_global_size(int i)
{
  switch (i)
  {
  default:
  case 0:
    return gridDim.x * blockDim.x;
  case 1:
    return gridDim.y * blockDim.y;
  case 2:
    return gridDim.z * blockDim.z;
  }
}

inline __device__ unsigned get_global_id(int i)
{
  switch (i)
  {
  default:
  case 0:
    return blockIdx.x * blockDim.x + threadIdx.x;
  case 1:
    return blockIdx.y * blockDim.y + threadIdx.y;
  case 2:
    return blockIdx.z * blockDim.z + threadIdx.z;
  }
}

inline __device__ float3 xyz(float4 v)
{
  return make_float3(v.x, v.y, v.z);
}

// Atomic operation mapping : OpenCL to CUDA
#define atomic_add atomicAdd
#define atomic_sub atomicSub
#define atomic_xchg atomicExch
#define atomic_min atomicMin
#define atomic_max atomicMax
#define atomic_inc(addr) atomicAdd(addr, 1)
#define atomic_dec(addr) atomicSub(addr, 1)
#define atomic_cmpxchg atomicCAS
#define atomic_and atomicAnd
#define atomic_or atomicOr
#define atomic_xor atomicXor

#include <math_constants.h>

#define M_PI CUDART_PI

#include "cutil_math.h"

#endif // CUDA_IMPORT_CL_H_
