// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUQUEUE_H
#define GPUQUEUE_H

#include "gpuConfig.h"

#include "gpuEvent.h"

#include <functional>

namespace gputil
{
  struct QueueDetail;

  /// Represents a command queue or stream on the GPU.
  ///
  /// Using multiple queues allow commands to be queue and managed in parallel. A queue may be
  /// synchronised using @c finish() or @c insertBarrier(). The former blocks until all queued operations
  /// have completed, while the latter creates a synchronisation point within the queue.
  ///
  /// Some functions, particularly those relating to @c Buffer, support an optional @c Queue argument.
  /// providing an explicit @c Queue argument changes the behaviour of such methods to be non-blocking
  /// operations. That is, the operation is inserted into the queue, without waiting for execution to
  /// complete. The @c Queue synchronisation methods, noted above, may be used to explicitly synchronise.
  ///
  /// In OpenCL terms this is an OpenCL queue. In CUDA terms it is a stream. A @c Queue may be used
  /// to invoke asynchronous GPU operations and control waiting for the results.
  ///
  /// Note that the synchronisation functionality provided here, namely @c finish() and @c insertBarrier(),
  /// are modelled more closely on CUDA streams than on the OpenCL event model. This is for simplicity.
  /// An equivalent to OpenCL style events is not really supported in CUDA.
  ///
  /// @todo Consider adding CUDA event/OpenCL marker supporting wait for methods.
  class gputilAPI Queue
  {
  public:
    /// Queue construction flags.
    enum QueueFlag
    {
      kProfile = (1 << 0)  ///< Enable profiling of the queue object.
    };

    /// Empty constructor.
    Queue();
    /// Copy constructor - references the same queue as @p other .
    /// @param other The queue to copy.
    Queue(const Queue &other);
    /// Move constructor.
    /// @param other The object to move.
    Queue(Queue &&other) noexcept;
    /// Construct from the underlying SDK queue implemntation.
    Queue(void *platform_queue);

    /// Destructor - release a reference to the unerlying queue object.
    ~Queue();

    /// Is this a valid @c Queue object?
    /// @return True if valid.
    bool isValid() const;

    /// Insert a barrier which ensures all operations before the barrier complete before executing
    /// operations queued after the barrier.
    void insertBarrier();

    /// Insert an event into the queue which marks the end of currently queued operations.
    /// This event may be used to block on completion of all currently queued operations.
    /// @return An event marking the current queue location.
    Event mark();

    /// Ensure that all queued commands have been submitted to the devices. A return does
    /// not mean that they have completed. For that, use @c finish().
    void flush();

    /// Wait for all outstanding operations in the queue to complete.
    void finish();

    /// Add a callback to the queue. The callback is invoked on CPU when queue execution reaches the current location
    /// in the queue. Note this callback may be called from a thread other than the main thread.
    /// @param callback The function to invoke.
    void queueCallback(const std::function<void(void)> &callback);

    /// Internal data access for private code.
    /// @return Internal queue details.
    QueueDetail *internal() const;

    /// Copy assignment.
    /// @param other The object to copy.
    Queue &operator=(const Queue &other);
    /// Move assignment.
    /// @param other The object to move.
    Queue &operator=(Queue &&other) noexcept;

  private:
    QueueDetail *queue_;
  };
}  // namespace gputil

#endif  // GPUQUEUE_H
