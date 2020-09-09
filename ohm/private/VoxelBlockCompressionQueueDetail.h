// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELMAPCOMPRESSIONQUEUEDETAIL_H
#define VOXELMAPCOMPRESSIONQUEUEDETAIL_H

#include "OhmConfig.h"

#include <tbb/concurrent_queue.h>
#include <tbb/mutex.h>

#include <atomic>
#include <deque>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace ohm
{
  class VoxelBlock;

  struct VoxelBlockCompressionQueueDetail
  {
    using Mutex = tbb::mutex;
    Mutex ref_lock;
    tbb::concurrent_queue<VoxelBlock *> compression_queue;
    std::atomic_int reference_count{ 0 };
    std::atomic_bool quit_flag{ false };
    std::thread processing_thread;
    bool running{ false };
  };
}  // namespace ohm

#endif  // VOXELMAPCOMPRESSIONQUEUEDETAIL_H
