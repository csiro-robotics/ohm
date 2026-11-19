// Maps the CUDA runtime API used by gputil onto HIP. This directory is on the include path of HIP builds only
// (OHM_FEATURE_HIP), so the CUDA sources compile unchanged for AMD GPUs.
#ifndef GPUTIL_HIP_COMPAT_CUDA_RUNTIME_H
#define GPUTIL_HIP_COMPAT_CUDA_RUNTIME_H

#include <hip/hip_runtime.h>

#define cudaDeviceProp hipDeviceProp_t
#define cudaError_t hipError_t
#define cudaErrorLaunchFailure hipErrorLaunchFailure
#define cudaErrorMemoryAllocation hipErrorMemoryAllocation
#define cudaErrorNotReady hipErrorNotReady
#define cudaEventBlockingSync hipEventBlockingSync
#define cudaEventCreateWithFlags hipEventCreateWithFlags
#define cudaEventDestroy hipEventDestroy
#define cudaEventQuery hipEventQuery
#define cudaEventRecord hipEventRecord
#define cudaEventSynchronize hipEventSynchronize
#define cudaEvent_t hipEvent_t
#define cudaFree hipFree
#define cudaFreeHost hipHostFree
#define cudaGetDevice hipGetDevice
#define cudaGetDeviceCount hipGetDeviceCount
#define cudaGetDeviceProperties hipGetDeviceProperties
#define cudaGetErrorString hipGetErrorString
#define cudaHostAlloc hipHostMalloc
#define cudaLaunchKernel hipLaunchKernel
#define cudaMalloc hipMalloc
#define cudaMemcpy hipMemcpy
#define cudaMemcpyAsync hipMemcpyAsync
#define cudaMemcpyDeviceToDevice hipMemcpyDeviceToDevice
#define cudaMemcpyDeviceToHost hipMemcpyDeviceToHost
#define cudaMemcpyHostToDevice hipMemcpyHostToDevice
#define cudaMemcpyHostToHost hipMemcpyHostToHost
#define cudaMemcpyKind hipMemcpyKind
#define cudaMemset hipMemset
#define cudaMemsetAsync hipMemsetAsync
#define cudaOccupancyMaxPotentialBlockSizeVariableSMem hipOccupancyMaxPotentialBlockSizeVariableSMem
#define cudaSetDevice hipSetDevice
#define cudaStreamAddCallback hipStreamAddCallback
#define cudaStreamCreate hipStreamCreate
#define cudaStreamDestroy hipStreamDestroy
#define cudaStreamSynchronize hipStreamSynchronize
#define cudaStreamWaitEvent hipStreamWaitEvent
#define cudaStream_t hipStream_t
#define cudaSuccess hipSuccess

#endif  // GPUTIL_HIP_COMPAT_CUDA_RUNTIME_H
