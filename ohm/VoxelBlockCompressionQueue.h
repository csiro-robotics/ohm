// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELBLOCKCOMPRESSIONQUEUE_H
#define VOXELBLOCKCOMPRESSIONQUEUE_H

#include "OhmConfig.h"

#include <vector>

namespace ohm
{
class VoxelBlock;
struct VoxelBlockCompressionQueueDetail;

/// Background compression thread used to manage compression of @c VoxelBlock data.
///
/// The queue supports reference counting in order to allow a single queue to be used accross multiple maps.
/// Each map should call @c retain() on construction and @c release() when done. The queue supports releasing the
/// last reference then attaining a new reference to start a new background thread.
///
/// An @c OccupancyMap will call @c retain() and @c release() on construction and destruction respectively.
class ohm_API VoxelBlockCompressionQueue
{
public:
  /// Singleton access.
  static VoxelBlockCompressionQueue &instance();

  /// Constructor.
  ///
  /// The @p test_mode must only be used for (unit) testing. This puts the queue into a synchronous mode and requires
  /// explicit calls to @c __tick().
  /// @param test_mode When true, creates a queue for testing.
  explicit VoxelBlockCompressionQueue(bool test_mode = false);
  /// Destructor. Ensures thread is joined.
  ~VoxelBlockCompressionQueue();

  /// Retain the compression queue. The first reference starts the background thread.
  void retain();

  /// Release the compression queue. Releasing the last reference joins the background thread.
  /// The next @c retain() call after the last @c release() reference will start a new thread.
  void release();

  uint64_t highWaterMark() const;
  void setHighWaterMark(uint64_t mark);
  uint64_t lowWaterMark() const;
  void setLowWaterMark(uint64_t mark);
  uint64_t estimatedAllocationSize() const;

  /// Push a @c VoxelBlock on the queue for compression.
  /// @param block The block to compress.
  void push(VoxelBlock *block);

  /// True if this object has been created in test mode.
  bool testMode() const;

  /// Test mode use only: called to perform a compression cycle. Must only be called when in @c testMode().
  /// @param compression_buffer Buffer used to compress into.
  void __tick(std::vector<uint8_t> &compression_buffer);

private:
  void joinCurrentThread();

  /// Main compression loop. This is the thread entry point.
  void run();

  std::unique_ptr<VoxelBlockCompressionQueueDetail> imp_;
};
}  // namespace ohm

#endif  // VOXELBLOCKCOMPRESSIONQUEUE_H
