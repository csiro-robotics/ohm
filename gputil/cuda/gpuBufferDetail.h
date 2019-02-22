// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUBUFFERDETAIL_H
#define GPUBUFFERDETAIL_H

#include "gputil/gpuDevice.h"

#include <string>

namespace gputil
{
  struct BufferDetail
  {
    void *mem = nullptr;
    size_t alloc_size = 0;
    unsigned flags = 0;
    Device device;
  };

  uint8_t *pin(BufferDetail &imp, PinMode mode);
  /// Unpin memory. Supports asynchronous unpinning by providing a queue, in which case
  /// a @p completion event is also recommended.
  void unpin(BufferDetail &imp, void *pinnedPtr, PinMode mode,
             Queue *queue = nullptr, Event *block_on = nullptr, Event *completion = nullptr);
}

#endif // GPUBUFFERDETAIL_H
