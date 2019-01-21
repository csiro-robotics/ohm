// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUBUFFERDETAIL_H
#define GPUBUFFERDETAIL_H

#include "gpuConfig.h"

#include <clu/clu.h>

#include "gputil/gpuDevice.h"

#include <string>

namespace gputil
{
  class Event;
  class Queue;

  struct BufferDetail
  {
    Device device;
    cl::Buffer buffer;
    size_t requested_size = 0;
    unsigned flags = 0;
    unsigned request_flags = 0;
  };

  uint8_t *pin(BufferDetail &imp, PinMode mode);
  /// Unpin memory. Supports asynchronous unpinning by providing a queue, in which case
  /// a @p completion event is also recommended.
  void unpin(BufferDetail &imp, void *pinned_ptr, Queue *queue = nullptr, Event *block_on = nullptr,
             Event *completion = nullptr);
}  // namespace gputil

#endif  // GPUBUFFERDETAIL_H
