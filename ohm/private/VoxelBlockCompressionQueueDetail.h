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

  /// Working data for the active voxel map compression thread.
  struct VoxelBlockCompressionThreadInfo
  {
    std::vector<uint8_t> compression_buffer;
    volatile bool quit_flag = false;
  };

  struct VoxelBlockCompressionQueueDetail
  {
    tbb::mutex queue_lock;
    std::condition_variable_any queue_notify;
    std::shared_ptr<std::thread> processing_thread;
    tbb::concurrent_queue<VoxelBlock *> compression_queue;
    volatile unsigned reference_count = 0;
    volatile VoxelBlock *current = nullptr;
    /// Active thread working data.
    VoxelBlockCompressionThreadInfo *thread_data = nullptr;
  };
}

#endif // VOXELMAPCOMPRESSIONQUEUEDETAIL_H
