//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#ifndef CLUBUFFER_H
#define CLUBUFFER_H

#include "clu.h"

namespace clu
{
  /// Calculate the best allocation size for @p requestedAllocationSize.
  ///
  /// Allocations should be cache line aligned for optimal efficiency.
  /// See https://software.intel.com/en-us/node/558501.
  /// The allocation is padded to either @p byteSizeAlignment or to the cache line size when
  /// @p byteSizeAlignment is zero.
  ///
  /// @param context The device context to allocate for.
  /// @param requested_allocation_size The requested allocation in bytes.
  /// @param byte_size_alignment The requested data alignment. Zero to use the cache line size.
  /// @return The padded/aligned allocation size.
  inline size_t bestAllocationSize(const cl::Context &context, size_t requested_allocation_size, size_t byte_size_alignment = 0)
  {
    size_t allocation_size = requested_allocation_size;

    if (byte_size_alignment == 0)
    {
      // Alignment padding not specified. Use the cache line size for optimal efficiency.
      // See https://software.intel.com/en-us/node/558501
      cl_device_id device_id = getFirstDevice(context);
      if (device_id)
      {
        clGetDeviceInfo(device_id, CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,
                        sizeof(byte_size_alignment), &byte_size_alignment, nullptr);
      }
    }

    // Pad allocations as requested.
    if (byte_size_alignment)
    {
      const size_t size_mod = allocation_size % byte_size_alignment;
      if (size_mod)
      {
        allocation_size += (byte_size_alignment - allocation_size % byte_size_alignment);
      }
    }

    return allocation_size;
  }

  /// Ensure the given @p buffer is of a size sufficient to hold @p elementCount elements of @p T.
  ///
  /// This ensures a minimum allocation of <tt>sizeof(T) * elementCount</tt> rounded up to the
  /// given @p byteSizeAlignment.
  ///
  /// @param buffer The buffer to operate on.
  /// @param flags OpenCL memory flags for the buffer.
  /// @param context The OpenCL context to allocate in. For existing allocated @p buffer objects
  ///   this must match the context the buffer has already been allocated within.
  /// @param element_count The number of elements required.
  /// @param error Error code return value.
  /// @param byte_size_alignment The byte padding alignment for the allocation size.
  ///       Zero to ensure padding to the cache line size in the default device of
  ///       @p context.
  /// @return True if the @p buffer has been allocated for the first time or reallocated.
  /// False if the buffer has been left as is. Errors are reported via @p error.
  template <typename T>
  bool ensureBufferSize(cl::Buffer &buffer, cl_mem_flags flags, const cl::Context &context,
                        size_t element_count, cl_int *error = nullptr, cl_uint byte_size_alignment = 0)
  {
    if (buffer())
    {
      size_t buffer_size = 0;
      clGetMemObjectInfo(buffer(), CL_MEM_SIZE, sizeof(buffer_size), &buffer_size, nullptr);
      if (buffer_size >= element_count * sizeof(T))
      {
        if (error)
        {
          *error = CL_SUCCESS;
        }
        return false;
      }

//      if (bufferSize)
//      {
//        std::cout << "Free " << bufferSize << std::endl;
//      }

      // Current buffer too small. Release existing buffer first (if any).
      buffer = cl::Buffer();
    }

    // Allocate the new buffer.
    cl_int clerr;
    const size_t allocation_size = bestAllocationSize(context, sizeof(T) * element_count, byte_size_alignment);
    buffer = cl::Buffer(context, flags, allocation_size, nullptr, &clerr);
//    std::cout << "Allocate " << allocationSize << std::endl;
    if (error)
    {
      *error = clerr;
    }
    return clerr == CL_SUCCESS;
  }


  /// Upload data to the GPU, creating the GPU buffer if required.
  ///
  /// The GPU buffer is created, or resized if it does not support @p elementCount
  /// elements. However, there may be a size mismatch, with the @p gpuBuffer elements
  /// being larger than the @p cpuBuffer elements. This most notably comes about when
  /// using 12 byte float3 data type and uploading to cl_float3 which is generally 16
  /// bytes. As such, the @p GPUT must be at least as large as @p CPUT. The upload is
  /// fastest when the sizes match exactly.
  ///
  /// @param queue The command queue used to upload with.
  /// @param context OpenCL context.
  /// @param gpu_buffer GPU buffer object. Size may be increased as required, but not decreased.
  /// @param cpu_buffer GPU buffer to upload from.
  /// @param element_count The number of elements in @p cpuBuffer to copy into @p gpuBuffer.
  /// @param allow_write When true, @p gpuBuffer is created with @c CL_MEM_READ_WRITE, otherwise
  ///     it is created with @c CL_MEM_READ_ONLY. Ignored if @p gpuBuffer is already initialised.
  /// @param reference Reference message passed to @p clu::checkError() on error.
  /// @param log The stream to log errors to.
  /// @tparam GPUT The GPU buffer type to upload to.
  /// @tparam CPUT The CPU buffer type to upload to.
  template <typename GPUT, typename CPUT>
  bool upload(cl::CommandQueue &queue, cl::Context &context,
              cl::Buffer &gpu_buffer, const CPUT *cpu_buffer, size_t element_count,
              bool allow_write, const char *reference, std::ostream &log = std::cerr)
  {
    static_assert(sizeof(GPUT) >= sizeof(GPUT), "GPU type smaller than CPU type.");
    const CPUT *cpu = cpu_buffer;

    cl_int clerr = 0;
    const cl_mem_flags mem_flags = (allow_write) ? CL_MEM_READ_ONLY : CL_MEM_READ_WRITE;
    clu::ensureBufferSize<GPUT>(gpu_buffer, mem_flags, context, element_count, &clerr);
    if (!clu::checkError(log, clerr, reference))
    {
      return false;
    }

    // Map target buffer.
    GPUT *gpu_mem = static_cast<GPUT *>(clEnqueueMapBuffer(queue(), gpu_buffer(),
                                                           CL_TRUE, CL_MAP_WRITE,
                                                           0, sizeof(GPUT) * element_count,
                                                           0, nullptr, nullptr, &clerr));

    if (!clu::checkError(log, clerr, reference))
    {
      return false;
    }

    if (sizeof(CPUT) == sizeof(GPUT))
    {
      memcpy(gpu_mem, cpu, sizeof(CPUT) * element_count);
    }
    else
    {
      GPUT *gpu = gpu_mem;
      for (size_t i = 0; i < element_count; ++i, ++gpu, ++cpu)
      {
        memcpy(gpu, cpu, sizeof(CPUT));
      }
    }

    // Unmap buffer.
    clerr = clEnqueueUnmapMemObject(queue(), gpu_buffer(),
                                    gpu_mem, 0, nullptr, nullptr);
    return clu::checkError(log, clerr, reference);
  }


  /// Download data from an existing GPU buffer to a CPU buffer.
  ///
  /// The size restrictions on @p GPUT and @p CPUT are the same as for @c upload().
  /// Both buffers must support at least @p elementCount elements.
  ///
  /// @param queue The command queue used to upload with.
  /// @param cpu_buffer GPU buffer to upload from.
  /// @param gpu_buffer GPU buffer object. Size may be increased as required, but not decreased.
  /// @param element_count The number of elements in @p cpuBuffer to copy into @p gpuBuffer.
  /// @param reference Reference message passed to @p clu::checkError() on error.
  /// @param log The stream to log errors to.
  /// @tparam GPUT The GPU buffer type to upload to.
  /// @tparam CPUT The CPU buffer type to upload to.
  template <typename GPUT, typename CPUT>
  bool download(cl::CommandQueue &queue, CPUT *cpu_buffer, cl::Buffer &gpu_buffer,
                size_t element_count, const char *reference, std::ostream &log = std::cerr)
  {
    static_assert(sizeof(GPUT) >= sizeof(GPUT), "GPU type smaller than CPU type.");
    CPUT *cpu = cpu_buffer;

    cl_int clerr = 0;
    // Map target buffer.
    GPUT *gpu_mem = static_cast<GPUT *>(clEnqueueMapBuffer(queue(), gpu_buffer(),
                                                           CL_TRUE, CL_MAP_READ,
                                                           0, sizeof(GPUT) * element_count,
                                                           0, nullptr, nullptr, &clerr));

    if (!clu::checkError(log, clerr, reference))
    {
      return false;
    }

    if (sizeof(GPUT) == sizeof(CPUT))
    {
      memcpy(cpu, gpu_mem, sizeof(CPUT) * element_count);
    }
    else
    {
      GPUT *gpu = gpu_mem;
      for (size_t i = 0; i < element_count; ++i, ++gpu, ++cpu)
      {
        memcpy(cpu, gpu, sizeof(*cpu));
      }
    }

    // Unmap buffer.
    clerr = clEnqueueUnmapMemObject(queue(), gpu_buffer(),
                                    gpu_mem, 0, nullptr, nullptr);
    return clu::checkError(log, clerr, reference);
  }
}

#endif // CLUBUFFER_H
