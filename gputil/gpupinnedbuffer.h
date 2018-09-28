// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPINNEDBUFFER_H_
#define GPUPINNEDBUFFER_H_

#include "gpuconfig.h"

#include "gpupinmode.h"

#include <cstddef>

namespace gputil
{
  class Buffer;
  class Event;
  class Queue;

  /// A utility class for direct memory access to pinned, host accessible memory.
  ///
  /// Pinned memory allows for far faster transfers.
  ///
  /// Falls back to unpinned memory transfers when required.
  ///
  /// To be used as transient objects only.
  class gputilAPI PinnedBuffer
  {
  public:
    /// Try pin @p buffer in the given @p mode.
    /// @param buffer Buffer to pin.
    /// @param mode The mode to pin in.
    PinnedBuffer(Buffer &buffer, PinMode mode);

    /// RValue constructor.
    /// @param other Temporary object to copy from.
    PinnedBuffer(PinnedBuffer &&other);

    PinnedBuffer(const PinnedBuffer &other) = delete;

    /// Destructor. Ensures unpinning in blocking mode. Call @c unpin() explicitly to support non-blocking writes.
    ~PinnedBuffer();

    /// Is the memory actually pinned?
    /// @return True if actually pinned, false when using fallbacks.
    bool isPinned() const;

    /// Repin the buffer if it has been unpinned.
    void pin();

    /// Unpin or release the pinned memory, invalidating this object.
    ///
    /// The unpin call blocks until the memory transfer completes when @p blocking is @c true. Otherwise this method
    /// may return before the memory transfer is complete. When calling asynchronously, it may be necessary to
    /// use Queue::insertBarrier() before performing any operations which read the target memory.
    ///
    /// Note that the destructor calls this method in blocking mode when this method has not been explicitly called.
    ///
    /// See @c Buffer class notes on asynchronous memory transfer for details on how the parameters modify the transfer,
    /// noting that asynchronous unpinning is only relevant when writting to the buffer.
    ///
    /// @param blocking True if this method waits for the memory transfer to complete before returning.
    /// @param queue Optional queue to use for the memory transfer. Recommended only for non-blocking use.
    /// @param blockOn Option event to wait on before unpinning.
    /// @param completion Event to track completion of the unpinning DMA.
    void unpin(Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

    /// Access the pinned buffer.
    /// @return The pinned buffer.
    inline Buffer *buffer() const { return _buffer; }

    /// Query the pinning mode.
    inline PinMode mode() const { return _mode; }

    /// Read from the buffer into @p dst.
    /// @param dst The memory to write to.
    /// @param byteCount Number of bytes to read.
    /// @param srcOffset Byte offset into the buffer to read from.
    /// @return The number of bytes read.
    size_t read(void *dst, size_t byteCount, size_t srcOffset = 0) const;

    /// Write data from @p src to the buffer.
    /// @param src Data to write into the buffer.
    /// @param byteCount Number of bytes to write.
    /// @param dstOffset Byte offset into the buffer to write at.
    /// @return The number of bytes written.
    size_t write(const void *src, size_t byteCount, size_t dstOffset = 0);

    /// RValue assignment operator.
    /// @param other Temporary object to copy from.
    PinnedBuffer &operator=(PinnedBuffer &&other);
    PinnedBuffer &operator=(const PinnedBuffer &other) = delete;

  private:
    Buffer *_buffer;
    void *_pinned;
    PinMode _mode;
  };
}

#endif // GPUPINNEDBUFFER_H_
