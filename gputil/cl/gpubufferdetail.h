// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUBUFFERDETAIL_H_
#define GPUBUFFERDETAIL_H_

#include "gpuconfig.h"

#include <clu/cl.hpp>

#include "gputil/gpudevice.h"

#include <string>

namespace gputil
{
  class Event;
  class Queue;

  struct BufferDetail
  {
    Device device;
    cl::Buffer buffer;
    size_t requestedSize;
    unsigned flags;

    inline BufferDetail()
      : requestedSize(0)
      , flags(0)
    {}
  };

  uint8_t *pin(BufferDetail &imp, PinMode mode);
  /// Unpin memory. Supports asynchronous unpinning by providing a queue, in which case
  /// a @p completion event is also recommended.
  void unpin(BufferDetail &imp, void *pinnedPtr,
             Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);
}

#endif // GPUBUFFERDETAIL_H_
