// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPINNEDBUFFER_H
#define GPUPINNEDBUFFER_H

#include "gpuConfig.h"

#include "gpuPinMode.h"

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
    /// Construct an invalid pinned buffer. May not be used.
    PinnedBuffer();

    /// Try pin @p buffer in the given @p mode.
    /// @param buffer Buffer to pin.
    /// @param mode The mode to pin in.
    PinnedBuffer(Buffer &buffer, PinMode mode);

    /// RValue constructor.
    /// @param other Temporary object to copy from.
    PinnedBuffer(PinnedBuffer &&other) noexcept;

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
    /// @param queue Optional queue to use for the memory transfer. Recommended only for non-blocking use.
    /// @param block_on Option event to wait on before unpinning.
    /// @param completion Event to track completion of the unpinning DMA.
    void unpin(Queue *queue = nullptr, Event *block_on = nullptr, Event *completion = nullptr);

    /// Access the pinned buffer.
    /// @return The pinned buffer.
    inline Buffer *buffer() const { return buffer_; }

    /// Query the pinning mode.
    inline PinMode mode() const { return mode_; }

    /// Read from the buffer into @p dst.
    /// @param dst The memory to write to.
    /// @param byte_count Number of bytes to read.
    /// @param src_offset Byte offset into the buffer to read from.
    /// @return The number of bytes read.
    size_t read(void *dst, size_t byte_count, size_t src_offset = 0) const;

    /// Write data from @p src to the buffer.
    /// @param src Data to write into the buffer.
    /// @param byte_count Number of bytes to write.
    /// @param dst_offset Byte offset into the buffer to write at.
    /// @return The number of bytes written.
    size_t write(const void *src, size_t byte_count, size_t dst_offset = 0);

    /// Read an array of data elements from the buffer.
    ///
    /// This method supports a size discrepancy between elements in the buffer and elements in @c dst.
    /// This is primarily to cater for the size difference between a 3-element float vector and
    /// @c cl_float3, which is a 4-element vector, with a redundant @c w component.
    ///
    /// Where elements in @p dst and @c this buffer do not match, specify the @p elementSize as the
    /// element stride in @p dst and @p bufferElementSize as the stride in @c this buffer.
    ///
    /// The method will copy as many elements as are available in @c this buffer. The @p dst memory
    /// must be sized as follows <tt>element_count * elementSize</tt>.
    ///
    /// @param dst The buffer to write into. Must be sized to suit.
    /// @param element_size The element stride in @p dst. The same stride is used for @c this buffer when
    ///     @p bufferElementSize is zero.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the read start. That is skip this number of elements before
    ///     reading.
    /// @param buffer_element_size Optional element stride within @c this buffer when different from
    ///     @p elementSize.
    /// @return The number of elements read. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    size_t readElements(void *dst, size_t element_size, size_t element_count, size_t offset_elements, 
                        size_t buffer_element_size);

    /// An overload of @c readElements() using the template sizes to determine the sizes.
    /// @tparam T The data type to read into and assumed to exactly match that stored in the GPU buffer.
    /// @param dst The buffer to write into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the read start. That is skip this number of elements before
    ///     reading.
    /// @return The number of elements read. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename T>
    inline size_t readElements(T *dst, size_t element_count, size_t offset_elements = 0)
    {
      return readElements(dst, sizeof(T), element_count, offset_elements, 0u);
    }

    /// An overload of @c readElements() using the template sizes to determine the sizes.
    /// @tparam BUFFER_TYPE The data type stored in the GPU buffer. Only required when its size differs from that of
    ///   @p T
    /// @tparam T The data type to read into.
    /// @param dst The buffer to write into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the read start. That is skip this number of elements before
    ///     reading.
    /// @return The number of elements read. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename BUFFER_TYPE, typename T>
    inline size_t readElements(T *dst, size_t element_count, size_t offset_elements = 0)
    {
      return readElements(dst, sizeof(T), element_count, offset_elements, sizeof(BUFFER_TYPE));
    }

    /// Write an array of data elements to the buffer.
    ///
    /// This method supports the same size discrepancy as @c readElements().
    ///
    /// The method will copy as many elements as are available in @c this buffer. The @p dst memory
    /// must be sized as follows <tt>element_count * element_size</tt>.
    ///
    /// @param src The buffer to read from.
    /// @param element_size The element stride in @p src. The same stride is used for @c this buffer when
    ///     @p bufferElementSize is zero.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements The offset to start writing at.
    /// @param buffer_element_size Optional element stride within @c this buffer when different from
    ///     @p element_size. Use zero when the size matches the @p element_size.
    /// @return The number of elements written. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    size_t writeElements(const void *src, size_t element_size, size_t element_count, size_t offset_elements,
                         size_t buffer_element_size);

    /// An overload of @c writeElements() using the template sizes to determine the sizes.
    /// @tparam T The data type to read into and assumed to exactly match that stored in the GPU buffer.
    /// @param src The buffer to read from into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the write start. That is skip this number of elements
    ///     before writing.
    /// @return The number of elements written. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename T>
    inline size_t writeElements(const T *src, size_t element_count, size_t offset_elements = 0)
    {
      return writeElements(src, sizeof(T), element_count, offset_elements, 0u);
    }

    /// An overload of @c writeElements() using the template sizes to determine the sizes.
    /// @tparam BUFFER_TYPE The data type stored in the GPU buffer. Only required when its size differs from that of
    ///   @p T
    /// @tparam T The data type to read into and assumed to exactly match that stored in the GPU buffer.
    /// @param src The buffer to read from into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the write start. That is skip this number of elements
    ///     before writing.
    /// @return The number of elements written. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename BUFFER_TYPE, typename T>
    inline size_t writeElements(const T *src, size_t element_count, size_t offset_elements = 0)
    {
      return writeElements(src, sizeof(T), element_count, offset_elements, sizeof(BUFFER_TYPE));
    }

    /// RValue assignment operator.
    /// @param other Temporary object to copy from.
    PinnedBuffer &operator=(PinnedBuffer &&other) noexcept;
    PinnedBuffer &operator=(const PinnedBuffer &other) = delete;

  private:
    Buffer *buffer_;
    void *pinned_;
    PinMode mode_;
  };
}

#endif // GPUPINNEDBUFFER_H
