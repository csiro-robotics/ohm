// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpuBuffer.h"

#include "gputil/cuda/gpuBufferDetail.h"
#include "gputil/cuda/gpuEventDetail.h"
#include "gputil/cuda/gpuQueueDetail.h"

#include "gputil/gpuApiException.h"
#include "gputil/gpuQueue.h"
#include "gputil/gpuThrow.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <cinttypes>
#include <iostream>

using namespace gputil;

/// Pinned memory garbage collector for asynchronous pinned memory copies. We can't call a cuda function from the
/// asynchronous callback, so we instead add it to the collector, then periodically collect. We collect on:
/// - Any pin()
/// - Any unpin()
/// - Any Buffer destructor.
/// - PinnedGarbageCollector destructor (final cleanup).
class PinnedGarbageCollector
{
public:
  inline ~PinnedGarbageCollector()
  {
    std::unique_lock<std::mutex> guard(mutex_);
    cudaError_t err = cudaSuccess;
    for (int i = 0; i < 2; ++i)
    {
      for (void *ptr : garbage_[i])
      {
        err = cudaFreeHost(ptr);
        if (err)
        {
          std::cerr << "Error (" << err << ") collecting CUDA allocation: " << ApiException::errorCodeString(err)
                    << std::endl;
        }
      }
      garbage_[i].clear();
    }
  }

  inline void add(void *ptr)
  {
    std::unique_lock<std::mutex> guard(mutex_);
    garbage_[active_garbage_].push_back(ptr);
  }

  inline void collect()
  {
    // We can't have the mutex locked while calling cudaFreeHost() as as that call may wait on the add() call to finish
    // from a cuda callback.
    std::unique_lock<std::mutex> guard(mutex_);
    std::vector<void *> &to_delete = garbage_[active_garbage_];
    active_garbage_ = 1 - active_garbage_;
    guard.unlock();

    cudaError_t err = cudaSuccess;
    for (void *ptr : to_delete)
    {
      err = cudaFreeHost(ptr);
      GPUAPICHECK2(err, cudaSuccess);
    }
    to_delete.clear();
  }

private:
  std::mutex mutex_;
  // Mutex cannot be locked while freeing memory, so use a kind of double buffer to allow early unlock.
  std::vector<void *> garbage_[2];
  int active_garbage_ = 0;
};

namespace
{
  // Static pinned garbage collector.
  PinnedGarbageCollector gc;
}  // namespace

namespace gputil
{
  // cudaMemcpyKind copyKind(unsigned dstFlags, unsigned srcFlags)
  //{
  //  if (dstFlags & BF_Host)
  //  {
  //    if (srcFlags & BF_Host)
  //    {
  //      return cudaMemcpyHostToHost;
  //    }

  //    return cudaMemcpyDeviceToHost;
  //  }
  //  else if (srcFlags & BF_Host)
  //  {
  //    return cudaMemcpyHostToDevice;
  //  }

  //  return cudaMemcpyDeviceToDevice;
  //}

  inline cudaStream_t selectStream(Queue *queue)
  {
    return (queue && queue->internal()) ? queue->internal()->obj() : nullptr;
  }

  void releasePinnedMemoryCallback(cudaStream_t /*stream*/, cudaError_t status, void *pinned_ptr)
  {
    GPUAPICHECK2(status, cudaSuccess);
    // Cannot call CUDA functions from this callback, so we can't free pinned_ptr. Instead, push it into the garbage.
    gc.add(pinned_ptr);
  }

  /// Internal copy command. Manages synchronous vs. asynchronous code paths.
  ///
  /// See @c Buffer class comments for details on how @p queue, @p block_on, and @c completion behave.
  ///
  /// @param dst The destination address.
  /// @param src The source address.
  /// @param byte_count Number of bytes to copy.
  /// @param kind The type of memory copy to execute.
  /// @param queue Event queue to resolve the cuda stream from. Null for default.
  /// @param block_on Event to wait for before copying.
  /// @param completion Event to set to mark completion.
  /// @return @c true when the synchronous, blocking code path was taken, @c false for asynchronous copy.
  bool bufferCopy(void *dst, const void *src, size_t byte_count, cudaMemcpyKind kind, Queue *queue, Event *block_on,
                  Event *completion)
  {
    cudaError_t err = cudaSuccess;
    // Manage asynchronous.
    if (queue)
    {
      // Async copy.
      // Resolve the stream.
      cudaStream_t stream = selectStream(queue);
      if (block_on && block_on->isValid())
      {
        err = cudaStreamWaitEvent(stream, block_on->detail()->obj(), 0);
        GPUAPICHECK(err, cudaSuccess, true);
      }

      err = cudaMemcpyAsync(dst, src, byte_count, kind, stream);
      GPUAPICHECK(err, cudaSuccess, true);

      if (completion)
      {
        err = cudaEventRecord(completion->detail()->obj(), stream);
        GPUAPICHECK(err, cudaSuccess, true);
      }
      return false;
    }

    if (block_on)
    {
      block_on->wait();
    }

    err = cudaMemcpy(dst, src, byte_count, kind);
    GPUAPICHECK(err, cudaSuccess, true);

    if (completion)
    {
      // Release completion event. Should not have been provided without queue argument.
      completion->release();
    }

    return true;
  }


  /// Internal memset command for device memory. Manages synchronous vs. asynchronous code paths.
  ///
  /// See @c Buffer class comments for details on how @p queue, @p block_on, and @c completion behave.
  ///
  /// @param mem The destination address.
  /// @param pattern Clear pattern.
  /// @param count Number of bytes to set.
  /// @param kind The type of memory copy to execute.
  /// @param queue Event queue to resolve the cuda stream from. Null for default.
  /// @param block_on Event to wait for before copying.
  /// @param completion Event to set to mark completion.
  /// @return @c true when the synchronous, blocking code path was taken, @c false for asynchronous copy.
  bool bufferSet(void *mem, int value, size_t count, Queue *queue, Event *block_on, Event *completion)
  {
    cudaError_t err;
    if (queue)
    {
      cudaStream_t stream = queue->internal()->obj();

      if (block_on && block_on->isValid())
      {
        err = cudaStreamWaitEvent(stream, block_on->detail()->obj(), 0);
        GPUAPICHECK(err, cudaSuccess, true);
      }

      err = cudaMemsetAsync(mem, value, count, stream);
      GPUAPICHECK(err, cudaSuccess, true);


      if (completion)
      {
        err = cudaEventRecord(completion->detail()->obj(), stream);
        GPUAPICHECK(err, cudaSuccess, true);
      }
      return false;
    }

    if (block_on)
    {
      block_on->wait();
    }

    err = cudaMemset(mem, value, count);
    GPUAPICHECK(err, cudaSuccess, true);

    if (completion)
    {
      // Release completion event. Should not have been provided without queue argument.
      completion->release();
    }
    return true;
  }


  uint8_t *pin(BufferDetail &buf, PinMode mode)
  {
    // TODO(KS): explore how CUDA actually deals with memory pinning. This setup is a bit weak.
    unsigned flags = 0;
    if (mode == kPinWrite)
    {
      flags |= cudaHostAllocWriteCombined;
    }
    void *pinned = nullptr;
    cudaError_t err = cudaHostAlloc(&pinned, buf.alloc_size, flags);
    GPUAPICHECK(err, cudaSuccess, nullptr);
    // Copy from GPU to host.
    // Currently only support synchronous mem copy.
    // TODO(KS): We do not need to copy the buffer content if we are writing to the entire buffer. However, we do not
    // know that ahead of time. To fix this, we need to maintain a list regions in the buffer which have been written
    // to an upload those back to the GPU. This will obviate the need for the following mem copy when pinning for write.
    err = cudaMemcpy(pinned, buf.mem, buf.alloc_size, cudaMemcpyDeviceToHost);
    GPUAPICHECK(err, cudaSuccess, nullptr);

    gc.collect();

    return static_cast<uint8_t *>(pinned);
  }


  void unpin(BufferDetail &imp, void *pinned_ptr, PinMode mode, Queue *queue, Event *block_on, Event *completion)
  {
    if (completion)
    {
      completion->release();
    }

    if (!pinned_ptr)
    {
      return;
    }

    cudaError_t err;
    if (mode == kPinWrite)
    {
      if (bufferCopy(imp.mem, pinned_ptr, imp.alloc_size, cudaMemcpyHostToDevice, queue, block_on, completion))
      {
        // Synchronous copy. Release host memory.
        err = cudaFreeHost(pinned_ptr);
        GPUAPICHECK2(err, cudaSuccess);
      }
      else
      {
        // Async copy. Setup a callback to release the host memory.
        cudaStream_t stream = queue->internal()->obj();
        err = cudaStreamAddCallback(stream, &releasePinnedMemoryCallback, pinned_ptr, 0);
        GPUAPICHECK2(err, cudaSuccess);
      }
    }

    gc.collect();
  }

  cudaError_t bufferAlloc(BufferDetail *buf, size_t alloc_size)
  {
    cudaError_t err;

    err = cudaMalloc(&buf->mem, alloc_size);

    if (err == cudaSuccess)
    {
      buf->alloc_size = alloc_size;
    }
    else
    {
      buf->alloc_size = 0;
    }
    return err;
  }

  void bufferFree(BufferDetail *buf)
  {
    if (buf && buf->mem)
    {
      cudaFree(buf->mem);
      buf->mem = nullptr;
      buf->alloc_size = 0;
    }
  }
}  // namespace gputil

Buffer::Buffer()
  : imp_(new BufferDetail)
{
  imp_->flags = 0;
}


Buffer::Buffer(const Device &device, size_t byte_size, unsigned flags)
  : imp_(new BufferDetail)
{
  create(device, byte_size, flags);
}


Buffer::Buffer(Buffer &&other) noexcept
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Buffer::~Buffer()
{
  release();
  delete imp_;
  gc.collect();
}


Buffer &Buffer::operator=(Buffer &&other) noexcept
{
  bufferFree(imp_);
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}


void Buffer::create(const Device &device, size_t byte_size, unsigned flags)
{
  release();
  imp_->device = device;
  imp_->flags = flags;

  resize(byte_size);
}


void Buffer::release()
{
  bufferFree(imp_);
}


void Buffer::swap(Buffer &other) noexcept
{
  std::swap(imp_, other.imp_);
}


bool Buffer::isValid() const
{
  return imp_->mem != nullptr;
}


unsigned Buffer::flags() const
{
  return imp_->flags;
}


size_t Buffer::size() const
{
  return imp_->alloc_size;
}


size_t Buffer::actualSize() const
{
  return imp_->alloc_size;
}


size_t Buffer::resize(size_t new_size)
{
  if (new_size == imp_->alloc_size)
  {
    return imp_->alloc_size;
  }

  bufferFree(imp_);
  cudaError_t err = bufferAlloc(imp_, new_size);
  if (err != cudaSuccess)
  {
    imp_->mem = nullptr;
    imp_->alloc_size = 0;
    GPUTHROW(ApiException(err), 0);
  }
  imp_->alloc_size = new_size;

  return size();
}


size_t Buffer::forceResize(size_t new_size)
{
  resize(new_size);
  return actualSize();
}


void Buffer::fill(const void *pattern, size_t pattern_size, Queue *queue, Event *block_on, Event *completion)
{
  if (isValid())
  {
    if (pattern_size == sizeof(int))
    {
      bufferSet(imp_->mem, *(const int *)pattern, imp_->alloc_size, queue, block_on, completion);
    }
    else
    {
      size_t wrote = 0;
      uint8_t *dst_mem = static_cast<uint8_t *>(imp_->mem);
      while (wrote + pattern_size < imp_->alloc_size)
      {
        bufferCopy(dst_mem + wrote, pattern, pattern_size, cudaMemcpyHostToDevice, queue, block_on, nullptr);
        wrote += pattern_size;
      }

      // Last copy.
      if (wrote < imp_->alloc_size)
      {
        bufferCopy(dst_mem + wrote, pattern, imp_->alloc_size - wrote, cudaMemcpyHostToDevice, queue, block_on,
                   completion);
        wrote += imp_->alloc_size - wrote;
      }
    }
  }
}


void Buffer::fillPartial(const void *pattern, size_t pattern_size, size_t fill_bytes, size_t offset, Queue *queue)
{
  if (isValid())
  {
    if (offset > imp_->alloc_size)
    {
      return;
    }

    if (offset + fill_bytes > imp_->alloc_size)
    {
      fill_bytes = imp_->alloc_size - offset;
    }

    uint8_t *dst_mem = static_cast<uint8_t *>(imp_->mem) + offset;
    if (pattern_size == sizeof(int))
    {
      bufferSet(dst_mem, *(int *)pattern, fill_bytes, queue, nullptr, nullptr);
    }
    else
    {
      size_t wrote = 0;
      while (wrote + pattern_size < fill_bytes)
      {
        bufferCopy(dst_mem + wrote, pattern, pattern_size, cudaMemcpyHostToDevice, queue, nullptr, nullptr);
        wrote += pattern_size;
      }

      // Last copy.
      if (wrote < fill_bytes)
      {
        bufferCopy(dst_mem + wrote + offset, pattern, fill_bytes - wrote, cudaMemcpyHostToDevice, queue, nullptr,
                   nullptr);
        wrote += fill_bytes - wrote;
      }
    }
  }
}


size_t Buffer::read(void *dst, size_t read_byte_count, size_t src_offset, Queue *queue, Event *block_on,
                    Event *completion)
{
  size_t copy_bytes = 0;
  if (imp_ && imp_->mem)
  {
    if (src_offset >= imp_->alloc_size)
    {
      return 0;
    }

    copy_bytes = read_byte_count;
    if (copy_bytes > imp_->alloc_size - src_offset)
    {
      copy_bytes = imp_->alloc_size - src_offset;
    }

    bufferCopy(dst, reinterpret_cast<const uint8_t *>(imp_->mem) + src_offset, copy_bytes, cudaMemcpyDeviceToHost,
               queue, block_on, completion);
  }
  return copy_bytes;
}


size_t Buffer::write(const void *src, size_t write_byte_count, size_t dst_offset, Queue *queue, Event *block_on,
                     Event *completion)
{
  size_t copy_bytes = 0;
  if (imp_ && imp_->mem)
  {
    if (dst_offset >= imp_->alloc_size)
    {
      return 0;
    }

    copy_bytes = write_byte_count;
    if (write_byte_count > imp_->alloc_size - dst_offset)
    {
      copy_bytes = imp_->alloc_size - dst_offset;
    }

    bufferCopy(reinterpret_cast<uint8_t *>(imp_->mem) + dst_offset, src, copy_bytes, cudaMemcpyHostToDevice, queue,
               block_on, completion);
  }
  return copy_bytes;
}


size_t Buffer::readElements(void *dst, size_t element_size, size_t element_count, size_t offset_elements,
                            size_t buffer_element_size, Queue *queue, Event *block_on, Event *completion)
{
  if (element_size == buffer_element_size || buffer_element_size == 0)
  {
    return read(dst, element_size * element_count, offset_elements * element_size, queue, block_on, completion);
  }

  const uint8_t *src = reinterpret_cast<const uint8_t *>(imp_->mem);
  uint8_t *dst2 = reinterpret_cast<uint8_t *>(dst);
  size_t copy_size = std::min(element_size, buffer_element_size);
  size_t src_offset = 0;

  src += offset_elements * buffer_element_size;
  for (size_t i = 0; i < element_count && src_offset <= imp_->alloc_size; ++i)
  {
    const bool final_copy = i + 1 == element_count || src_offset + buffer_element_size > imp_->alloc_size;
    bufferCopy(dst2, src + src_offset, copy_size, cudaMemcpyDeviceToHost, queue, block_on,
               (!final_copy) ? nullptr : completion);

    dst2 += element_size;
    src_offset += buffer_element_size;
  }

  return src_offset;
}


size_t Buffer::writeElements(const void *src, size_t element_size, size_t element_count, size_t offset_elements,
                             size_t buffer_element_size, Queue *queue, Event *block_on, Event *completion)
{
  if (element_size == buffer_element_size || buffer_element_size == 0)
  {
    return write(src, element_size * element_count, offset_elements * element_size, queue, block_on, completion);
  }

  uint8_t *dst = reinterpret_cast<uint8_t *>(imp_->mem);
  const uint8_t *src2 = reinterpret_cast<const uint8_t *>(src);
  size_t copy_size = std::min(element_size, buffer_element_size);
  size_t dst_offset = 0;

  dst += offset_elements * buffer_element_size;
  for (size_t i = 0; i < element_count && dst_offset <= imp_->alloc_size; ++i)
  {
    const bool final_copy = i + 1 == element_count || dst_offset + buffer_element_size > imp_->alloc_size;
    bufferCopy(dst + dst_offset, src2, copy_size, cudaMemcpyDeviceToHost, queue, block_on,
               (!final_copy) ? nullptr : completion);

    dst += element_size;
    dst_offset += buffer_element_size;
  }

  return dst_offset;
}


void *Buffer::argPtr() const
{
  return &imp_->mem;
}


void *Buffer::address() const
{
  return imp_->mem;
}


// void *Buffer::pin(PinMode mode)
// {
//   return (imp_) ? gputil::pin(*imp_, mode) : nullptr;
// }


// void Buffer::unpin(void *ptr, Queue *queue, Event *block_on, Event *completion)
// {
//   if (imp_)
//   {
//     gputil::unpin(*imp_, ptr, queue, block_on, completion);
//   }
// }


namespace gputil
{
  size_t copyBuffer(Buffer &dst, const Buffer &src, Queue *queue, Event *block_on, Event *completion)
  {
    return copyBuffer(dst, 0, src, 0, src.size(), queue, block_on, completion);
  }


  size_t copyBuffer(Buffer &dst, const Buffer &src, size_t byte_count, Queue *queue, Event *block_on, Event *completion)
  {
    return copyBuffer(dst, 0, src, 0, byte_count, queue, block_on, completion);
  }


  size_t copyBuffer(Buffer &dst, size_t dst_offset, const Buffer &src, size_t src_offset, size_t byte_count,
                    Queue *queue, Event *block_on, Event *completion)
  {
    BufferDetail *dst_detail = dst.detail();
    BufferDetail *src_detail = src.detail();
    if (!src_detail || !dst_detail)
    {
      return 0;
    }

    // Check offsets.
    if (dst_detail->alloc_size < dst_offset)
    {
      return 0;
    }

    if (src_detail->alloc_size < src_offset)
    {
      return 0;
    }

    // Check sizes after offset.
    byte_count = std::min(byte_count, dst_detail->alloc_size - dst_offset);
    byte_count = std::min(byte_count, src_detail->alloc_size - src_offset);

    bufferCopy(dst_detail->mem, src_detail->mem, byte_count, cudaMemcpyDeviceToDevice, queue, block_on, completion);

    return byte_count;
  }
}  // namespace gputil
