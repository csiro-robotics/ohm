// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUBUFFER_H_
#define GPUBUFFER_H_

#include "gpuconfig.h"

#include "gpupinmode.h"

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
    BF_Read = (1 << 0),
    /// Buffer memory can be written on the device.
    BF_Write = (1 << 1),
    /// Buffer is in host accessible memory on the device.
    /// Required for buffer pinning.
    BF_HostAccess = (1 << 2),

    /// Alias for combining read/write flags.
    BF_ReadWrite = BF_Read | BF_Write,

    /// Alias for host accessible read.
    BF_ReadHost = BF_Read | BF_HostAccess,

    /// Alias for host accessible write.
    BF_WriteHost = BF_Write | BF_HostAccess,

    /// Alias for host accessible read/write.
    BF_ReadWriteHost = BF_Read | BF_Write | BF_HostAccess
  };

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
  /// non-null  | any         | non-null      | Asynchronous call. @p completion tracks the completion of the transfer.s
  ///
  /// Note that using asychronous transfer may invoke a less optimal memory tranfer path (in OpenCL).
  class gputilAPI Buffer
  {
  public:
    /// Create an invalid buffer object.
    Buffer();

    /// Create a buffer for the given @p device and @p byteSize.
    /// @param device The GPU device to create the buffer on/for.
    /// @param byteSize Number of bytes to allocate. Actual allocation may be larger.
    /// @param flags See @c BufferFlag.
    Buffer(const Device &device, size_t byteSize, unsigned flags = BF_ReadWrite);

    /// R-Value constructor.
    Buffer(Buffer &&other);

    /// Deleted.
    Buffer(const Buffer &&other) = delete;

    /// Destructor.
    ~Buffer();

    /// Deleted because we can't have two buffers referencing the same address.
    /// Use @c swap().
    Buffer &operator=(const Buffer &) = delete;

    /// R-ralue assignment.
    /// @return *this
    Buffer &operator=(Buffer &&other);

    /// Create and initialise the @c Buffer object.
    ///
    /// The @c Buffer is created for the given @p device and initialised to the
    /// given @p byteSize. Read/write accessibility are set by @p flags.
    /// If the buffer has already been created, then the existing memory is release.
    /// This may change the @c Device associated with the @c Buffer.
    ///
    /// @param device The GPU device to create the buffer on/for.
    /// @param byteSize Number of bytes to allocate. Actual allocation may be larger.
    /// @param flags See @c BufferFlag.
    void create(const Device &device, size_t byteSize, unsigned flags = BF_Read);

    /// Release any existing memory and disassociate this @c Buffer from a @c Device.
    /// The @c Buffer object becomes invalid.
    void release();

    /// Swap this buffer with @p other. This swaps the underlying addresses and allocations.
    /// @param other The buffer to swap with.
    void swap(Buffer &other);

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
    inline size_t elementCount() const { return size() / sizeof(T); }

    /// Check the number of data elements of size @p bufferElementSize the buffer can hold.
    /// @param bufferElementSize The size or stride of a single element in the buffer.
    /// @return The number of elements the buffer has the capacity for.
    inline size_t elementCount(size_t bufferElementSize) const { return size() / bufferElementSize; }

    /// Resize the buffer to the @p newSize if necessary.
    /// This will shrink the buffer, but the actual size may end up larger than @p newSize
    /// to improve performance.
    ///
    /// The buffer is only reallocated if @c actualSize() is insufficient to meet
    /// the needs of @p newSize.
    ///
    /// Note: existing data in the buffer may be lost.
    ///
    /// @param newSize The requested size in bytes.
    /// @return The actual allocation size (bytes).
    size_t resize(size_t newSize);

    /// Resizes the buffer even if it already has the capacity for @p newSize.
    /// This method behaves like @c resize(), except that it will re-allocate
    /// when the best actual size for @p newSize is smaller than the current actual size.
    /// @param newSize The requested size in bytes.
    /// @return The actual allocation size (bytes).
    size_t forceResize(size_t newSize);

    /// Resize the buffer to support @p elementCount elements of type @c T.
    ///
    /// Note: existing data in the buffer may be lost.
    ///
    /// @param elementCount The number of elements required.
    /// @tparam T The data type defining the element size.
    /// @return The actual buffer size in elements of @p T.
    template <typename T>
    inline size_t elementsResize(size_t elementCount)
    {
      return resize(sizeof(T) * elementCount) / sizeof(T);
    }

    /// Clear the buffer contents to @p value.
    /// Behaves a bit like @c memset(). Uses @c fill() underneath.
    /// @param value The value to clear to.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously. See @c fill().
    inline void clear(int value, Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr)
    {
      fill(&value, sizeof(value), queue, blockOn, completion);
    }

    /// @overload.
    inline void clear(unsigned value, Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr)
    {
      fill(&value, sizeof(value), queue, blockOn, completion);
    }

    /// Fill the buffer with the given @p pattern.
    ///
    /// The @c fill() operation may optionally be performed asynchronously by passing
    /// a pointer to an empty @c event object. Note that asynchronous operation may not be
    /// faster as synchronous operation can use pinned pointers.
    ///
    /// @param pattern The memory pattern to fill with.
    /// @param patternSize The size of the data at @p pattern, in bytes.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param blockOn Optional event to wait for be performing the fill.
    /// @param completion Optional event to setup to mark completion of the fill operation.
    void fill(const void *pattern, size_t patternSize,
              Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

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
    /// @param patternSize The size of the data at @p pattern, in bytes.
    /// @param offset Offset into the buffer to write at (bytes).
    /// @param fillBytes Number of total bytes to fill in the buffer. Best to be a multiple of @p patternSize, but need not be.
    void fillPartial(const void *pattern, size_t patternSize, size_t fillBytes, size_t offset,
                     Queue *queue = nullptr);// too tricky for now: , Event *blockOn = nullptr, Event *completion = nullptr);

    /// Read data from the buffer.
    ///
    /// Address ranges are constrained by @p readByteCount, @p srcOffset and
    /// @c size() to ensure valid address ranges. This may limit the number of
    /// bytes actually read.
    ///
    /// @param dst The address to read data into.
    /// @param readByteCount The number of bytes to read. @p dst must support this byte count.
    /// @param size_t srcOffset The offset into this @c Buffer to start reading from (bytes).
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param blockOn Optional event to wait for be performing the read.
    /// @param completion Optional event to setup to mark completion of the read operation.
    /// @return The number of bytes read.
    size_t read(void *dst, size_t readByteCount, size_t srcOffset = 0,
                Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

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
    /// @param byteCount The number of bytes from @p src to write to this @c Buffer.
    /// @param dstOffset The offset into this @c Buffer to start writing at.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param blockOn Optional event to wait for be performing the write.
    /// @param completion Optional event to setup to mark completion of the write operation.
    /// @return The number of bytes written.
    size_t write(const void *src, size_t byteCount, size_t dstOffset = 0,
                 Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

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
    /// must be sized as follows <tt>elementCount * elementSize</tt>.
    ///
    /// @param dst The buffer to write into. Must be sized to suit.
    /// @param elementSize The element stride in @p dst. The same stride is used for @c this buffer when
    ///     @p bufferElementSize is zero.
    /// @param elementCount The number of elements to copy.
    /// @param bufferElementSize Optional element stride within @c this buffer when different from
    ///     @p elementSize.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param blockOn Optional event to wait for be performing the read.
    /// @param completion Optional event to setup to mark completion of the read operation.
    /// @return The number of elements read. May be less than @c elementCount if this buffer is not
    ///     large enough to hold @c elementCount.
    size_t readElements(void *dst, size_t elementSize, size_t elementCount,
                        size_t offsetElements, size_t bufferElementSize,
                        Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

    /// An overload of @c readElements() using the template sizes to determin the sizes.
    template <typename T>
    inline size_t readElements(T *dst, size_t elementCount, size_t offsetElements = 0,
                               Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr)
    {
      return readElements(dst, sizeof(T), elementCount, offsetElements, 0u, queue, blockOn, completion);
    }

    /// An overload of @c readElements() using the template sizes to determin the sizes.
    template <typename BUFFER_TYPE, typename T>
    inline size_t readElements(T *dst, size_t elementCount, size_t offsetElements = 0,
                               Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr)
    {
      return readElements(dst, sizeof(T), elementCount, offsetElements, sizeof(BUFFER_TYPE),
                          queue, blockOn, completion);
    }

    /// Write an array of data elements to the buffer.
    ///
    /// This method supports the same size discrepancy as @c readElements().
    ///
    /// The method will copy as many elements as are available in @c this buffer. The @p dst memory
    /// must be sized as follows <tt>elementCount * elementSize</tt>.
    ///
    /// @param src The buffer to read from.
    /// @param elementSize The element stride in @p src. The same stride is used for @c this buffer when
    ///     @p bufferElementSize is zero.
    /// @param elementCount The number of elements to copy.
    /// @param offsetElements The offset to start writing at.
    /// @param bufferElementSize Optional element stride within @c this buffer when different from
    ///     @p elementSize. Use zero when the size matches the @p elementSize.
    /// @param queue Optional queue to perform the operation on. Providing a non null queue
    ///     changes the operation to occur asynchronously.
    /// @param blockOn Optional event to wait for be performing the write.
    /// @param completion Optional event to setup to mark completion of the write operation.
    /// @return The number of elements written. May be less than @c elementCount if this buffer is not
    ///     large enough to hold @c elementCount.
    size_t writeElements(const void *src, size_t elementSize, size_t elementCount,
                         size_t offsetElements, size_t bufferElementSize,
                         Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

    /// An overload of @c writeElements() using the template sizes to determin the sizes.
    template <typename T>
    inline size_t writeElements(const T *src, size_t elementCount, size_t offsetElements = 0,
                                Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr)
    {
      return writeElements(src, sizeof(T), elementCount, offsetElements, 0u, queue, blockOn, completion);
    }

    /// An overload of @c writeElements() using the template sizes to determin the sizes.
    template <typename BUFFER_TYPE, typename T>
    inline size_t writeElements(const T *src, size_t elementCount, size_t offsetElements = 0,
                                Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr)
    {
      return writeElements(src, sizeof(T), elementCount, offsetElements, sizeof(BUFFER_TYPE),
                           queue, blockOn, completion);
    }

    /// Internal pointer for argument passing to the device function/kernel.
    ///
    /// For CUDA this is the CUDA address which can be passed directly as an argument.
    ///
    /// For OpenCL the type is @c cl_mem, which may be set as kernel argument.
    void *arg_() const;

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
    template <typename T> inline T arg() { return static_cast<T>(arg_()); }

    /// @c overload
    template <typename T> inline T arg() const { return static_cast<T>(arg_()); }

    void *pin(PinMode mode);
    void unpin(void *ptr, Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

    /// @internal
    inline BufferDetail *detail() const { return _imp; }

  private:
    BufferDetail *_imp;
  };

  /// Copy data from @p src to @p dst. All data from @p src are copied and @p dst must be at least as large as @p src.
  ///
  /// See @c Buffer class comments for details on asynchronous transfer.
  ///
  /// @param dst The destination buffer.
  /// @param src The target buffer.
  /// @param queue Optional queue to perform the operation on. Providing a non null queue
  ///     changes the operation to occur asynchronously.
  /// @param blockOn Optional event to wait for be performing the copy.
  /// @param completion Optional event to setup to mark completion of the copy operation.
  /// @return The number of bytes copied
  size_t copyBuffer(Buffer &dst, const Buffer &src,
                    Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

  /// Copy data from @p src to @p dst copying only @p byteCount bytes.
  /// Both buffers must support @p byteCount bytes.
  ///
  /// See @c Buffer class comments for details on asynchronous transfer.
  ///
  /// @param dst The destination buffer.
  /// @param src The target buffer.
  /// @param byteCount The number of bytes to copy.
  /// @param queue Optional queue to perform the operation on. Providing a non null queue
  ///     changes the operation to occur asynchronously.
  /// @param blockOn Optional event to wait for be performing the copy.
  /// @param completion Optional event to setup to mark completion of the copy operation.
  /// @return The number of bytes copied
  size_t copyBuffer(Buffer &dst, const Buffer &src, size_t byteCount,
                    Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);

  /// Copy data from @p src to @p dst with offsets, copying only @p byteCount bytes.
  /// Both buffers must support @p byteCount bytes and the given offsets.
  ///
  /// See @c Buffer class comments for details on asynchronous transfer.
  ///
  /// @param dst The destination buffer.
  /// @param dstOffset Offset into the destination buffer to copy into (bytes).
  /// @param src The target buffer.
  /// @param srcOffset Offset into the source buffer to copy from (bytes).
  /// @param byteCount The number of bytes to copy.
  /// @param queue Optional queue to perform the operation on. Providing a non null queue
  ///     changes the operation to occur asynchronously.
  /// @param blockOn Optional event to wait for be performing the copy.
  /// @param completion Optional event to setup to mark completion of the copy operation.
  /// @return The number of bytes copied
  size_t copyBuffer(Buffer &dst, size_t dstOffset, const Buffer &src, size_t srcOffset, size_t byteCount,
                    Queue *queue = nullptr, Event *blockOn = nullptr, Event *completion = nullptr);
}

#endif // GPUBUFFER_H_
