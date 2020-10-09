// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELMAPCOMPRESSIONQUEUEDETAIL_H
#define VOXELMAPCOMPRESSIONQUEUEDETAIL_H

#include "OhmConfig.h"

#include "Mutex.h"

#ifdef OHM_THREADS
#include <tbb/concurrent_queue.h>
#else  // OHM_THREADS
#include <queue>
#endif  // OHM_THREADS

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
    using Mutex = ohm::Mutex;
    Mutex ref_lock;
#ifdef OHM_THREADS
    tbb::concurrent_queue<VoxelBlock *> compression_queue;
#else   // OHM_THREADS
    ohm::SpinMutex queue_lock;
    std::queue<VoxelBlock *> compression_queue;
#endif  // OHM_THREADS
    std::atomic_int reference_count{ 0 };
    std::atomic_bool quit_flag{ false };
    std::thread processing_thread;
    bool running{ false };
  };

  inline void push(VoxelBlockCompressionQueueDetail &detail, VoxelBlock *block)
  {
#ifdef OHM_THREADS
    detail.compression_queue.push(block);
#else   // OHM_THREADS
    std::unique_lock<ohm::SpinMutex> guard(detail.queue_lock);
    detail.compression_queue.emplace(block);
#endif  // OHM_THREADS
  }

  inline bool tryPop(VoxelBlockCompressionQueueDetail &detail, VoxelBlock **block)
  {
#ifdef OHM_THREADS
    return detail.compression_queue.try_pop(*block);
#else   // OHM_THREADS
    std::unique_lock<ohm::SpinMutex> guard(detail.queue_lock);
    if (!detail.compression_queue.empty())
    {
      *block = detail.compression_queue.back();
      detail.compression_queue.pop();
      return true;
    }

    return false;
#endif  // OHM_THREADS
  }
}  // namespace ohm

#endif  // VOXELMAPCOMPRESSIONQUEUEDETAIL_H
