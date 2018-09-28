// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUBUFFERDETAIL_H_
#define GPUBUFFERDETAIL_H_

#include "gpudevice.h"

#include <string>

namespace gputil
{
  struct BufferDetail
  {
    void *mem;
    size_t allocSize;
    unsigned flags;
    Device device;

    inline BufferDetail()
      : mem(nullptr)
      , allocSize(0)
      , flags(0)
    {}
  };

  uint8_t *pin(BufferDetail &imp, PinMode mode);
  /// Unpin memory. Supports asynchronous unpinning by providing a queue, in which case
  /// a @p completion event is also recommended.
  void unpin(BufferDetail &imp, void *pinnedPtr, PinMode mode,
             Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);
}

#endif // GPUBUFFERDETAIL_H_
