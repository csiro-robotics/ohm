// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELBLOCKCOMPRESSIONQUEUE_H
#define VOXELBLOCKCOMPRESSIONQUEUE_H

#include "OhmConfig.h"

#include <tbb/mutex.h>

#include <memory>
#include <mutex>

namespace ohm
{
  class VoxelBlock;
  struct VoxelBlockCompressionThreadInfo;
  struct VoxelBlockCompressionQueueDetail;

  /// Background compression thread used to manage compression of @c VoxelBlock data.
  ///
  /// The queue supports reference counting in order to allow a single queue to be used accross multiple maps.
  /// Each map should call @c retain() on construction and @c release() when done. The queue supports releasing the
  /// last reference then attaining a new reference to start a new background thread.
  class ohm_API VoxelBlockCompressionQueue
  {
  public:
    /// Singleton access.
    static VoxelBlockCompressionQueue &instance();

    /// Constructor.
    VoxelBlockCompressionQueue();
    /// Destructor. Ensures thread is joined.
    ~VoxelBlockCompressionQueue();

    /// Retain the compression queue. The first reference starts the background thread.
    void retain();

    /// Release the compression queue. Releasing the last reference joins the background thread.
    void release();

    /// Push a @c VoxelBlock on the queue for compression.
    /// @param block The block to compress.
    void push(VoxelBlock *block);

  private:
    void joinCurrentThread(std::unique_lock<tbb::mutex> &guard);

    void run(VoxelBlockCompressionThreadInfo *thread_data);

    std::unique_ptr<VoxelBlockCompressionQueueDetail> imp_;
  };
}

#endif // VOXELBLOCKCOMPRESSIONQUEUE_H
