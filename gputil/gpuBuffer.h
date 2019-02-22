// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUBUFFER_H
#define GPUBUFFER_H

#include "gpuConfig.h"

#include "gpuPinMode.h"

#include <cstddef>

namespace gputil
{
  struct BufferDetail;
  class Device;
  class Event;
  class Queue;

  /// Flags used to control @c Buffer creation.
  enum BufferFlag
  {
    /// Buffer memory can be read on the device.
    kBfRead = (1 << 0),
    /// Buffer memory can be written on the device.
    kBfWrite = (1 << 1),
    /// Buffer is in host accessible memory on the device.
    /// Required for buffer pinning.
    kBfHostAccess = (1 << 2),

    /// Alias for combining read/write flags.
    kBfReadWrite = kBfRead | kBfWrite,

    /// Alias for host accessible read.
    kBfReadHost = kBfRead | kBfHostAccess,

    /// Alias for host accessible write.
    kBfWriteHost = kBfWrite | kBfHostAccess,

    /// Alias for host accessible read/write.
    kBfReadWriteHost = kBfRead | kBfWrite | kBfHostAccess
  };

  // clang-format off
  /// @class Buffer
  /// Represents a GPU based buffer. Implementation depends on the
  /// GPU SDK.
  ///
  /// @par Synchronous vs Asynchronous tansfer
  /// Memory transfer function typically accept three optional parameters: a @c Queue and two @c Event pointers,
  /// typically labelled @c blockOn and @c competion. The behaviour of memory transfer functions may differ depending
  /// on whether a @c Queue is provide and whether @c blockOn and/or @c completion events are provide. Firstly,
  /// providing a non-null @c Queue invokes an asynchronous data transfer. As such the host memory buffer must remain
  /// valid until the transfer completes. The @p completion event supports the asynchronous transfer providing a means
  /// of monitoring completion of the memory transfer via either @c Event::waitFor() or @c Event::isComplete(). Finally,
  /// the @c blockOn argument specifies an existing event which must complete before @em starting the memory transfer.
  /// @c blockOn may be used with a null @p queue, in which case this call remains a blocking call. This behaviour is
  /// summarised below.
  ///
  /// @p queue  | @p blockOn  | @p completion | Behaviour
  /// --------- | ----------- | ------------- | ---------
  /// nullptr   | nullptr     | ignored       | Blocking call using host accessible memory if possible.
  /// nullptr   | non-null    | ignored       | Blocking call, waiting on @p blockOn first. Providing @p completion is redundant.
  /// non-null  | nullptr     | nullptr       | Asynchronous call, no event information available.
  /// non-null  | non-null    | -             | Asynchronous call. Memory transfer occurs after @p blockOn completes.
  /// non-null  | any         | non-null      | Asynchronous call. @p completion tracks the completion of the transfers.
  ///
  /// Note that using asychronous transfer may invoke a less optimal memory tranfer path (in OpenCL).
// clang-format off
  class gputilAPI Buffer
  {
  public:
    /// Create an invalid buffer object.
    Buffer();

    /// Create a buffer for the given @p device and @p byteSize.
    /// @param device The GPU device to create the buffer on/for.
    /// @param byte_size Number of bytes to allocate. Actual allocation may be larger.
    /// @param flags See @c BufferFlag.
    Buffer(const Device &device, size_t byte_size, unsigned flags = kBfReadWrite);

    /// R-Value constructor.
    Buffer(Buffer &&other) noexcept;

    /// Deleted.
    Buffer(const Buffer &&other) = delete;

    /// Destructor.
    ~Buffer();

    /// Deleted because we can't have two buffers referencing the same address.
    /// Use @c swap().
    Buffer &operator=(const Buffer &) = delete;

    /// R-value assignment.
    /// @return *this
    Buffer &operator=(Buffer &&other) noexcept;

    /// Create and initialise the @c Buffer object.
    ///
    /// The @c Buffer is created for the given @p device and initialised to the
    /// given @p byteSize. Read/write accessibility are set by @p flags.
    /// If the buffer has already been created, then the existing memory is release.
    /// This may change the @c Device associated with the @c Buffer.
    ///
    /// @param device The GPU device to create the buffer on/for.
    /// @param byte_size Number of bytes to allocate. Actual allocation may be larger.
    /// @param flags See @c BufferFlag.
    void create(const Device &device, size_t byte_size, unsigned flags = kBfRead);

    /// Release any existing memory and disassociate this @c Buffer from a @c Device.
    /// The @c Buffer object becomes invalid.
    void release();

    /// Swap this buffer with @p other. This swaps the underlying addresses and allocations.
    /// @param other The buffer to swap with.
    void swap(Buffer &other) noexcept;

    /// Checks if the buffer is valid for use.
    /// @return true if valid.
    bool isValid() const;

    /// Access the @c BufferFlag values used to create the buffer.
    unsigned flags() const;

    /// Returns the requested buffer allocation in bytes.
    /// @return Buffer size in bytes.
    size_t size() const;

    /// Returns the actual size of the buffer in bytes. May be larger than the requested size
    /// as the buffer is padded for better alignment.
    /// @return Actual buffer size in bytes.
    size_t actualSize() const;

    /// Check the number of @p T typed elements the buffer can hold.
    /// @tparam T The data type defining the element size.
    /// @return The number of elements the buffer has the capacity for.
    template <typename T>
    inline size_t elementCount() const
    {
      return size() / sizeof(T);
    }

    /// Check the number of data elements of size @p bufferElementSize the buffer can hold.
    /// @param buffer_element_size The size or stride of a single element in the buffer.
    /// @return The number of elements the buffer has the capacity for.
    inline size_t elementCount(size_t buffer_element_size) const { return size() / buffer_element_size; }

    /// Resize the buffer to the @p newSize if necessary.
    /// This will shrink the buffer, but the actual size may end up larger than @p newSize
    /// to improve performance.
    ///
    /// The buffer is only reallocated if @c actualSize() is insufficient to meet
    /// the needs of @p newSize.
    ///
    /// Note: existing data in the buffer may be lost.
    ///
    /// @param new_size The requested size in bytes.
    /// @return The actual allocation size (bytes).
    size_t resize(size_t new_size);

    /// Resizes the buffer even if it already has the capacity for @p newSize.
    /// This method behaves like @c resize(), except that it will re-allocate
    /// when the best actual size for @p newSize is smaller than the current actual size.
    /// @param new_size The requested size in bytes.
    /// @return The actual allocation size (bytes).
    size_t forceResize(size_t new_size);

    /// Resize the buffer to support @p element_count elements of type @c T.
    ///
    /// Note: existing data in the buffer may be lost.
    ///
    /// @param element_count The number of elements required.
    /// @tparam T The data type defining the element size.
    /// @return The actual buffer size in elements of @p T.
    template <typename T>
    inline size_t elementsResize(size_t element_count)
    {
      return resize(sizeof(T) * element_count) / sizeof(T);
    }

    /// Clear the buffer contents to @p value.
    /// Behaves a bit like @c memset(). Uses @c fill() underneath.
    /// @param value The value to clear to.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously. See @c fill().
    /// @param block_on Block the clear opteration on the completion of this event.
    /// @param completion When provided, makes changes to an asynchronous call and the event is set to mark completion.
    inline void clear(int value, Queue *queue = nullptr, Event *block_on = nullptr, Event *completion = nullptr)
    {
      fill(&value, sizeof(value), queue, block_on, completion);
    }

    /// @overload.
    inline void clear(unsigned value, Queue *queue = nullptr, Event *block_on = nullptr, Event *completion = nullptr)
    {
      fill(&value, sizeof(value), queue, block_on, completion);
    }

    /// Fill the buffer with the given @p pattern.
    ///
    /// The @c fill() operation may optionally be performed asynchronously by passing
    /// a pointer to an empty @c event object. Note that asynchronous operation may not be
    /// faster as synchronous operation can use pinned pointers.
    ///
    /// @param pattern The memory pattern to fill with.
    /// @param pattern_size The size of the data at @p pattern, in bytes.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the fill.
    /// @param completion Optional event to setup to mark completion of the fill operation.
    void fill(const void *pattern, size_t pattern_size, Queue *queue = nullptr, Event *block_on = nullptr,
              Event *completion = nullptr);

    /// Fill part of the buffer with @p pattern.
    ///
    /// This is much less efficient than filling the entire buffer using @c fill(), but may be
    /// more efficient than making multiple @c write() calls.
    ///
    /// The @c fill() operation may optionally be performed asynchronously by passing
    /// a pointer to an empty @c event object. Note that asynchronous operation may not be
    /// faster as synchronous operation can use pinned pointers.
    ///
    /// @param pattern The memory pattern to fill with.
    /// @param pattern_size The size of the data at @p pattern, in bytes.
    /// @param offset Offset into the buffer to write at (bytes).
    /// @param fill_bytes Number of total bytes to fill in the buffer. Best to be a multiple of @p patternSize, but need
    /// not be.
    /// @param queue The command queue in which to perform the operation.
    void fillPartial(
      const void *pattern, size_t pattern_size, size_t fill_bytes, size_t offset,
      Queue *queue = nullptr);  // too tricky for now: , Event *blockOn = nullptr, Event *completion = nullptr);

    /// Read data from the buffer.
    ///
    /// Address ranges are constrained by @p readByteCount, @p srcOffset and
    /// @c size() to ensure valid address ranges. This may limit the number of
    /// bytes actually read.
    ///
    /// @param dst The address to read data into.
    /// @param read_byte_count The number of bytes to read. @p dst must support this byte count.
    /// @param src_offset srcOffset The offset into this @c Buffer to start reading from (bytes).
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the read.
    /// @param completion Optional event to setup to mark completion of the read operation.
    /// @return The number of bytes read.
    size_t read(void *dst, size_t read_byte_count, size_t src_offset = 0, Queue *queue = nullptr,
                Event *block_on = nullptr, Event *completion = nullptr);

    /// Write data to the buffer.
    ///
    /// Address ranges are constrained by @p byteCount, @p dstOffset
    /// and @c size() to ensure valid address ranges. This may limit the number of
    /// bytes actually written.
    ///
    /// The behaviour of the write function is modified by the @p queue, @c blockOn and @c completion parameters
    /// just like the @c read() function. See that function for details.
    ///
    /// @param src The address holding the data to write into this @c Buffer.
    /// @param byte_count The number of bytes from @p src to write to this @c Buffer.
    /// @param dst_offset The offset into this @c Buffer to start writing at.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the write.
    /// @param completion Optional event to setup to mark completion of the write operation.
    /// @return The number of bytes written.
    size_t write(const void *src, size_t byte_count, size_t dst_offset = 0, Queue *queue = nullptr,
                 Event *block_on = nullptr, Event *completion = nullptr);

    /// Read an array of data elements from the buffer.
    ///
    /// This method supports a size discrepancy between elements in the buffer and elements in @c dst.
    /// This is primarily to cater for the size difference between a 3-element float vector and
    /// @c cl_float3, which is a 4-element vector, with a redundant @c w component.
    ///
    /// Where elements in @p dst and @c this buffer do not match, specify the @p element_size as the
    /// element stride in @p dst and @p bufferElementSize as the stride in @c this buffer.
    ///
    /// The method will copy as many elements as are available in @c this buffer. The @p dst memory
    /// must be sized as follows <tt>element_count * element_size</tt>.
    ///
    /// @param dst The buffer to write into. Must be sized to suit.
    /// @param element_size The element stride in @p dst. The same stride is used for @c this buffer when
    ///     @p bufferElementSize is zero.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the read start. That is skip this number of elements before
    ///     reading.
    /// @param buffer_element_size Optional element stride within @c this buffer when different from
    ///     @p element_size.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the read.
    /// @param completion Optional event to setup to mark completion of the read operation.
    /// @return The number of elements read. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    size_t readElements(void *dst, size_t element_size, size_t element_count, size_t offset_elements,
                        size_t buffer_element_size, Queue *queue = nullptr, Event *block_on = nullptr,
                        Event *completion = nullptr);

    /// An overload of @c readElements() using the template sizes to determine the sizes.
    /// @tparam T The data type to read into and assumed to exactly match that stored in the GPU buffer.
    /// @param dst The buffer to write into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the read start. That is skip this number of elements before
    ///     reading.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the read.
    /// @param completion Optional event to setup to mark completion of the read operation.
    /// @return The number of elements read. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename T>
    inline size_t readElements(T *dst, size_t element_count, size_t offset_elements = 0, Queue *queue = nullptr,
                               Event *block_on = nullptr, Event *completion = nullptr)
    {
      return readElements(dst, sizeof(T), element_count, offset_elements, 0u, queue, block_on, completion);
    }

    /// An overload of @c readElements() using the template sizes to determine the sizes.
    /// @tparam BUFFER_TYPE The data type stored in the GPU buffer. Only required when its size differs from that of
    ///   @p T
    /// @tparam T The data type to read into.
    /// @param dst The buffer to write into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the read start. That is skip this number of elements before
    ///     reading.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the read.
    /// @param completion Optional event to setup to mark completion of the read operation.
    /// @return The number of elements read. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename BUFFER_TYPE, typename T>
    inline size_t readElements(T *dst, size_t element_count, size_t offset_elements = 0, Queue *queue = nullptr,
                               Event *block_on = nullptr, Event *completion = nullptr)
    {
      return readElements(dst, sizeof(T), element_count, offset_elements, sizeof(BUFFER_TYPE), queue, block_on,
                          completion);
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
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the write.
    /// @param completion Optional event to setup to mark completion of the write operation.
    /// @return The number of elements written. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    size_t writeElements(const void *src, size_t element_size, size_t element_count, size_t offset_elements,
                         size_t buffer_element_size, Queue *queue = nullptr, Event *block_on = nullptr,
                         Event *completion = nullptr);

    /// An overload of @c writeElements() using the template sizes to determine the sizes.
    /// @tparam T The data type to read into and assumed to exactly match that stored in the GPU buffer.
    /// @param src The buffer to read from into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the write start. That is skip this number of elements
    ///     before writing.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the write.
    /// @param completion Optional event to setup to mark completion of the write operation.
    /// @return The number of elements written. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename T>
    inline size_t writeElements(const T *src, size_t element_count, size_t offset_elements = 0, Queue *queue = nullptr,
                                Event *block_on = nullptr, Event *completion = nullptr)
    {
      return writeElements(src, sizeof(T), element_count, offset_elements, 0u, queue, block_on, completion);
    }

    /// An overload of @c writeElements() using the template sizes to determine the sizes.
    /// @tparam BUFFER_TYPE The data type stored in the GPU buffer. Only required when its size differs from that of
    ///   @p T
    /// @tparam T The data type to read into and assumed to exactly match that stored in the GPU buffer.
    /// @param src The buffer to read from into. Must be sized to suit.
    /// @param element_count The number of elements to copy.
    /// @param offset_elements Number of elements to offset the write start. That is skip this number of elements
    ///     before writing.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param block_on Optional event to wait for be performing the write.
    /// @param completion Optional event to setup to mark completion of the write operation.
    /// @return The number of elements written. May be less than @c element_count if this buffer is not
    ///     large enough to hold @c element_count.
    template <typename BUFFER_TYPE, typename T>
    inline size_t writeElements(const T *src, size_t element_count, size_t offset_elements = 0, Queue *queue = nullptr,
                                Event *block_on = nullptr, Event *completion = nullptr)
    {
      return writeElements(src, sizeof(T), element_count, offset_elements, sizeof(BUFFER_TYPE), queue, block_on,
                           completion);
    }

    /// Internal pointer for argument passing to the device function/kernel.
    ///
    /// For CUDA this is the CUDA address which can be passed directly as an argument.
    ///
    /// For OpenCL the type is @c cl_mem, which may be set as kernel argument.
    void *argPtr() const;

    /// Return the internal pointer for argument passing as the given pointer type.
    ///
    /// CUDA looks as follows:
    /// @code
    ///   gputil::Buffer buffer;
    ///   //...
    ///   // Pass buffer as a CUDA float3 *
    ///   deviceFunction<<<...>>>(buffer.arg<float3 *>());
    /// @endcode
    ///
    /// OpenCL usage looks as follows (using OpenCL C++ wrapper):
    /// @code
    ///   gputil::Buffer buffer;
    ///   //...
    ///   cl::Kernel kernel;
    ///   //...
    ///   kernel.setArg(0, buffer.arg<cl_mem>());
    /// @endcode
    template <typename T>
    inline T arg()
    {
      return static_cast<T>(argPtr());
    }

    /// @c overload
    template <typename T>
    inline T arg() const
    {
      return static_cast<T>(argPtr());
    }

    // void *pin(PinMode mode);
    // void unpin(void *ptr, Queue *queue = nullptr, Event *block_on = nullptr, Event *completion = nullptr);

    /// @internal
    inline BufferDetail *detail() const { return imp_; }

  private:
    BufferDetail *imp_;
  };

  /// Copy data from @p src to @p dst. All data from @p src are copied and @p dst must be at least as large as @p src.
  ///
  /// See @c Buffer class comments for details on asynchronous transfer.
  ///
  /// @param dst The destination buffer.
  /// @param src The target buffer.
  /// @param queue Optional queue to perform the operation on. Providing a non null queue
  ///     changes the operation to occur asynchronously.
  /// @param block_on Optional event to wait for be performing the copy.
  /// @param completion Optional event to setup to mark completion of the copy operation.
  /// @return The number of bytes copied
  size_t copyBuffer(Buffer &dst, const Buffer &src, Queue *queue = nullptr, Event *block_on = nullptr,
                    Event *completion = nullptr);

  /// Copy data from @p src to @p dst copying only @p byteCount bytes.
  /// Both buffers must support @p byteCount bytes.
  ///
  /// See @c Buffer class comments for details on asynchronous transfer.
  ///
  /// @param dst The destination buffer.
  /// @param src The target buffer.
  /// @param byte_count The number of bytes to copy.
  /// @param queue Optional queue to perform the operation on. Providing a non null queue
  ///     changes the operation to occur asynchronously.
  /// @param block_on Optional event to wait for be performing the copy.
  /// @param completion Optional event to setup to mark completion of the copy operation.
  /// @return The number of bytes copied
  size_t copyBuffer(Buffer &dst, const Buffer &src, size_t byte_count, Queue *queue = nullptr,
                    Event *block_on = nullptr, Event *completion = nullptr);

  /// Copy data from @p src to @p dst with offsets, copying only @p byteCount bytes.
  /// Both buffers must support @p byteCount bytes and the given offsets.
  ///
  /// See @c Buffer class comments for details on asynchronous transfer.
  ///
  /// @param dst The destination buffer.
  /// @param dst_offset Offset into the destination buffer to copy into (bytes).
  /// @param src The target buffer.
  /// @param src_offset Offset into the source buffer to copy from (bytes).
  /// @param byte_count The number of bytes to copy.
  /// @param queue Optional queue to perform the operation on. Providing a non null queue
  ///     changes the operation to occur asynchronously.
  /// @param block_on Optional event to wait for be performing the copy.
  /// @param completion Optional event to setup to mark completion of the copy operation.
  /// @return The number of bytes copied
  size_t copyBuffer(Buffer &dst, size_t dst_offset, const Buffer &src, size_t src_offset, size_t byte_count,
                    Queue *queue = nullptr, Event *block_on = nullptr, Event *completion = nullptr);
}  // namespace gputil

#endif  // GPUBUFFER_H
