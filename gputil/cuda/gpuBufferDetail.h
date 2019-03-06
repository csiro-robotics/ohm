// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUBUFFERDETAIL_H
#define GPUBUFFERDETAIL_H

#include "gputil/gpuDevice.h"

#include "gputil/cuda/gpuMemRegion.h"

#include <cuda_runtime.h>

namespace gputil
{
  enum PinStatus : unsigned
  {
    kPinnedForRead = (1 << 0),
    kPinnedForWrite = (1 << 1)
  };

  struct BufferDetail
  {
    void *device_mem = nullptr;
    /// Host mapped memory for pinning. Maintained by gpuPinnedBuffer code.
    void *mapped_mem = nullptr;
    size_t alloc_size = 0;
    unsigned flags = 0;
    unsigned pinned_status = 0;
    Device device;
    /// List of dirty regions which need to be copied from mapped_mep to device_mem.
    std::vector<MemRegion> dirty_write;
  };

    bool bufferCopy(void *dst, const void *src, size_t byte_count, cudaMemcpyKind kind, Queue *queue, Event *block_on,
                  Event *completion);
    uint8_t *pin(BufferDetail &buf, PinMode mode);
    void unpin(BufferDetail &imp, void *pinned_ptr, PinMode mode, Queue *queue, Event *block_on, Event *completion);
}  // namespace gputil

#endif  // GPUBUFFERDETAIL_H
