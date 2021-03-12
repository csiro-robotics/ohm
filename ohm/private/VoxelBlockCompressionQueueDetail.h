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
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace ohm
{
class VoxelBlock;

/// Data structure used to track the blocks available for compression.
struct CompressionEntry
{
  VoxelBlock *voxels;      ///< [Compression enabled] voxel block.
  size_t allocation_size;  ///< Last calculated allocation size.
};

/// @c VoxelBlockCompressionQueue internals.
struct VoxelBlockCompressionQueueDetail
{
  using Mutex = ohm::Mutex;
  /// Reference count mutex.
  Mutex ref_lock;
#ifdef OHM_THREADS
  /// Queue used to push @c VoxelBlock candidates for compression.
  tbb::concurrent_queue<VoxelBlock *> compression_queue;
#else   // OHM_THREADS
  /// Spin lock mutex for @c compression_queue
  ohm::SpinMutex queue_lock;
  /// Queue used to push @c VoxelBlock candidates for compression.
  std::queue<VoxelBlock *> compression_queue;
#endif  // OHM_THREADS
  /// Full set of registered @c VoxelBlock items.
  std::vector<CompressionEntry> blocks;
  /// High tide to initiate compression at.
  std::atomic_uint64_t high_tide{ 12ull * 1024ull * 1024ull * 1024ull };
  /// Low tide to compression down to.
  std::atomic_uint64_t low_tide{ 6ull * 1024ull * 1024ull * 1024ull };
  /// Current allocation estimation.
  std::atomic_uint64_t estimated_allocated_size{ 0 };
  /// Thread reference count.
  std::atomic_int reference_count{ 0 };
  /// Thread quit flag.
  std::atomic_bool quit_flag{ false };
  /// Processing thread.
  std::thread processing_thread;
  /// True if @c processing_thread is running.
  bool running{ false };
  /// Set when instantiated for testing.
  bool test_mode{ false };
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
