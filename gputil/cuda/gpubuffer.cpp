// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpubuffer.h"

#include "cuda/gpubufferdetail.h"
#include "cuda/gpueventdetail.h"
#include "cuda/gpuqueuedetail.h"

#include "gpuapiexception.h"
#include "gpuqueue.h"
#include "gputhrow.h"

#include <algorithm>
#include <cinttypes>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace gputil;

namespace gputil
{
  //cudaMemcpyKind copyKind(unsigned dstFlags, unsigned srcFlags)
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

  void releasePinnedMemoryCallback(cudaStream_t stream, cudaError_t status, void *pinnedPtr)
  {
    GPUAPICHECK2(status, cudaSuccess);
    cudaError_t err = cudaFreeHost(pinnedPtr);
    GPUAPICHECK2(err, cudaSuccess);
  }

  /// Internal copy command. Manages synchronous vs. asynchronous code paths.
  ///
  /// See @c Buffer class comments for details on how @p queue, @p blockOn, and @c completion behave.
  ///
  /// @param dst The destination address.
  /// @param src The source address.
  /// @param byteCount Number of bytes to copy.
  /// @param kind The type of memory copy to execute.
  /// @param queue Event queue to resolve the cuda stream from. Null for default.
  /// @param blockOn Event to wait for before copying.
  /// @param completion Event to set to mark completion.
  /// @return @c true when the synchronous, blocking code path was taken, @c false for asynchronous copy.
  bool bufferCopy(void *dst, const void *src, size_t byteCount, cudaMemcpyKind kind,
                  Queue *queue, Event *blockOn, Event *completion)
  {
    cudaError_t err = cudaSuccess;
    // Manage asynchronous.
    if (queue)
    {
      // Async copy.
      // Resolve the stream.
      cudaStream_t stream = selectStream(queue);
      if (blockOn && blockOn->isValid())
      {
        err = cudaStreamWaitEvent(stream, blockOn->detail()->obj(), 0);
        GPUAPICHECK(err, cudaSuccess, true);
      }

      err = cudaMemcpyAsync(dst, src, byteCount, cudaMemcpyDeviceToHost, stream);

      if (completion)
      {
        err = cudaEventRecord(completion->detail()->obj(), stream);
        GPUAPICHECK(err, cudaSuccess, true);
      }
      return false;
    }

    if (blockOn)
    {
      blockOn->wait();
    }

    err = cudaMemcpy(dst, src, byteCount, kind);
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
  /// See @c Buffer class comments for details on how @p queue, @p blockOn, and @c completion behave.
  ///
  /// @param mem The destination address.
  /// @param pattern Clear pattern.
  /// @param count Number of bytes to set.
  /// @param kind The type of memory copy to execute.
  /// @param queue Event queue to resolve the cuda stream from. Null for default.
  /// @param blockOn Event to wait for before copying.
  /// @param completion Event to set to mark completion.
  /// @return @c true when the synchronous, blocking code path was taken, @c false for asynchronous copy.
  bool bufferSet(void *mem, int value, size_t count, Queue *queue, Event *blockOn, Event *completion)
  {
    cudaError_t err;
    if (queue)
    {
      cudaStream_t stream = queue->internal()->obj();

      if (blockOn && blockOn->isValid())
      {
        err = cudaStreamWaitEvent(stream, blockOn->detail()->obj(), 0);
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

    if (blockOn)
    {
      blockOn->wait();
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
    unsigned flags = 0;
    if (mode == PinWrite)
    {
      flags |= cudaHostAllocWriteCombined;
    }
    void *pinned = nullptr;
    cudaError_t err = cudaHostAlloc(&pinned, buf.allocSize, flags);
    GPUAPICHECK(err, cudaSuccess, nullptr);
    if (mode == PinRead)
    {
      // Copy from GPU to host.
      // Currently only support synchronous mem copy.
      err = cudaMemcpy(pinned, buf.mem, buf.allocSize, cudaMemcpyDeviceToHost);
      GPUAPICHECK(err, cudaSuccess, nullptr);
    }
    return (uint8_t *)pinned;
  }


  void unpin(BufferDetail &imp, void *pinnedPtr, PinMode mode,
             Queue *queue, Event *blockOn, Event *completion)
  {
    if (completion)
    {
      completion->release();
    }

    cudaError_t err;
    if (mode == PinWrite)
    {
      if (bufferCopy(imp.mem, pinnedPtr, imp.allocSize, cudaMemcpyHostToDevice, queue, blockOn, completion))
      {
        // Synchronous copy. Release host memory.
        err = cudaFreeHost(pinnedPtr);
        GPUAPICHECK2(err, cudaSuccess);
      }
      else
      {
        // Async copy. Setup a callback to release the host memory.
        cudaStream_t stream = queue->internal()->obj();
        err = cudaStreamAddCallback(stream, &releasePinnedMemoryCallback, pinnedPtr, 0);
        GPUAPICHECK2(err, cudaSuccess);
      }
    }
  }

  cudaError_t bufferAlloc(BufferDetail *buf, size_t allocSize)
  {
    cudaError_t err;

    err = cudaMalloc(&buf->mem, allocSize);

    if (err == cudaSuccess)
    {
      buf->allocSize = allocSize;
    }
    else
    {
      buf->allocSize = 0;
    }
    return err;
  }

  void bufferFree(BufferDetail *buf)
  {
    if (buf && buf->mem)
    {
      cudaFree(buf->mem);
      buf->mem = nullptr;
      buf->allocSize = 0;
    }
  }
}

Buffer::Buffer()
  : _imp(new BufferDetail)
{
  _imp->flags = 0;
}


Buffer::Buffer(const Device &device, size_t byteSize, unsigned flags)
  : _imp(new BufferDetail)
{
  create(device, byteSize, flags);
}


Buffer::Buffer(Buffer &&other)
  : _imp(other._imp)
{
  other._imp = nullptr;
}


Buffer::~Buffer()
{
  release();
  delete _imp;
}


Buffer &Buffer::operator=(Buffer &&other)
{
  bufferFree(_imp);
  delete _imp;
  _imp = other._imp;
  other._imp = nullptr;
  return *this;
}


void Buffer::create(const Device &device, size_t byteSize, unsigned flags)
{
  release();
  _imp->device = device;
  _imp->flags = flags;

  resize(byteSize);
}


void Buffer::release()
{
  bufferFree(_imp);
}


void Buffer::swap(Buffer &other)
{
  std::swap(_imp, other._imp);
}


bool Buffer::isValid() const
{
  return _imp->mem != nullptr;
}


unsigned Buffer::flags() const
{
  return _imp->flags;
}


size_t Buffer::size() const
{
  return _imp->allocSize;
}


size_t Buffer::actualSize() const
{
  return _imp->allocSize;
}


size_t Buffer::resize(size_t newSize)
{
  if (newSize == _imp->allocSize)
  {
    return _imp->allocSize;
  }

  bufferFree(_imp);
  cudaError_t err = bufferAlloc(_imp, newSize);
  if (err != cudaSuccess)
  {
    _imp->mem = nullptr;
    _imp->allocSize = 0;
    GPUTHROW(ApiException(err), 0);
  }
  _imp->allocSize = newSize;

  return size();
}


size_t Buffer::forceResize(size_t newSize)
{
  resize(newSize);
  return actualSize();
}



void Buffer::fill(const void *pattern, size_t patternSize,
                  Queue *queue, Event *blockOn, Event *completion)
{
  if (isValid())
  {
    cudaError_t err = cudaSuccess;
    if (patternSize == sizeof(int))
    {
      bufferSet(_imp->mem, *(const int *)pattern, _imp->allocSize, queue, blockOn, completion);
    }
    else
    {
      size_t wrote = 0;
      uint8_t *dstMem = static_cast<uint8_t *>(_imp->mem);
      while (wrote + patternSize < _imp->allocSize)
      {
        bufferCopy(dstMem + wrote, pattern, patternSize, cudaMemcpyHostToDevice,
                   queue, blockOn, nullptr);
        wrote += patternSize;
      }

      // Last copy.
      if (wrote < _imp->allocSize)
      {
        bufferCopy(dstMem + wrote, pattern, _imp->allocSize - wrote, cudaMemcpyHostToDevice,
                   queue, blockOn, completion);
        wrote += _imp->allocSize - wrote;
      }
    }
  }
}


void Buffer::fillPartial(const void *pattern, size_t patternSize, size_t fillBytes, size_t offset, Queue *queue)
{
  if (isValid())
  {
    if (offset > _imp->allocSize)
    {
      return;
    }

    if (offset + fillBytes > _imp->allocSize)
    {
      fillBytes = _imp->allocSize - offset;
    }

    cudaError_t err = cudaSuccess;
    uint8_t *dstMem = static_cast<uint8_t *>(_imp->mem) + offset;
    if (patternSize == sizeof(int))
    {
      bufferSet(dstMem, *(int *)pattern, fillBytes, queue, nullptr, nullptr);
    }
    else
    {
      size_t wrote = 0;
      while (wrote + patternSize < fillBytes)
      {
        bufferCopy(dstMem + wrote, pattern, patternSize, cudaMemcpyHostToDevice,
                   queue, nullptr, nullptr);
        wrote += patternSize;
      }

      // Last copy.
      if (wrote < fillBytes)
      {
        bufferCopy(dstMem + wrote + offset, pattern, fillBytes - wrote, cudaMemcpyHostToDevice,
                   queue, nullptr, nullptr);
        wrote += fillBytes - wrote;
      }
    }
  }
}


size_t Buffer::read(void *dst, size_t readByteCount, size_t srcOffset,
                    Queue *queue, Event *blockOn, Event *completion)
{
  size_t copyBytes = 0;
  if (_imp && _imp->mem)
  {
    if (srcOffset >= _imp->allocSize)
    {
      return 0;
    }

    copyBytes = readByteCount;
    if (copyBytes > _imp->allocSize - srcOffset)
    {
      copyBytes = _imp->allocSize - srcOffset;
    }

    bufferCopy(dst, _imp->mem, copyBytes, cudaMemcpyDeviceToHost, queue, blockOn, completion);
  }
  return copyBytes;
}


size_t Buffer::write(const void *src, size_t writeByteCount, size_t dstOffset,
                     Queue *queue, Event *blockOn, Event *completion)
{
  size_t copyBytes = 0;
  if (_imp && _imp->mem)
  {
    if (dstOffset >= _imp->allocSize)
    {
      return 0;
    }

    copyBytes = writeByteCount;
    if (writeByteCount > _imp->allocSize - dstOffset)
    {
      copyBytes = _imp->allocSize - dstOffset;
    }

    bufferCopy(_imp->mem, src, copyBytes, cudaMemcpyHostToDevice, queue, blockOn, completion);
  }
  return copyBytes;
}


size_t Buffer::readElements(void *dst, size_t elementSize, size_t elementCount, size_t offsetElements, size_t bufferElementSize,
                            Queue *queue, Event *blockOn, Event *completion)
{
  if (elementSize == bufferElementSize || bufferElementSize == 0)
  {
    return read(dst, elementSize * elementCount, offsetElements * elementSize, queue, blockOn, completion);
  }

  const uint8_t *src = reinterpret_cast<const uint8_t *>(_imp->mem);
  uint8_t *dst2 = reinterpret_cast<uint8_t *>(dst);
  size_t copySize = std::min(elementSize, bufferElementSize);
  size_t srcOffset = 0;
  cudaError_t err = cudaSuccess;

  src += offsetElements * bufferElementSize;
  for (size_t i = 0; i < elementCount && srcOffset <= _imp->allocSize; ++i)
  {
    const bool finalCopy = i + 1 == elementCount || srcOffset + bufferElementSize > _imp->allocSize;
    bufferCopy(dst2, src + srcOffset, copySize, cudaMemcpyDeviceToHost,
               queue, blockOn, (!finalCopy) ? nullptr : completion);

    dst2 += elementSize;
    srcOffset += bufferElementSize;
  }

  return srcOffset;
}


size_t Buffer::writeElements(const void *src, size_t elementSize, size_t elementCount, size_t offsetElements, size_t bufferElementSize,
                             Queue *queue, Event *blockOn, Event *completion)
{
  if (elementSize == bufferElementSize || bufferElementSize == 0)
  {
    return write(src, elementSize * elementCount, offsetElements * elementSize, queue, blockOn, completion);
  }

  uint8_t *dst = reinterpret_cast<uint8_t *>(_imp->mem);
  const uint8_t *src2 = reinterpret_cast<const uint8_t *>(src);
  size_t copySize = std::min(elementSize, bufferElementSize);
  size_t dstOffset = 0;
  cudaError_t err = cudaSuccess;

  dst += offsetElements * bufferElementSize;
  for (size_t i = 0; i < elementCount && dstOffset <= _imp->allocSize; ++i)
  {
    const bool finalCopy = i + 1 == elementCount || dstOffset + bufferElementSize > _imp->allocSize;
    bufferCopy(dst + dstOffset, src2, copySize, cudaMemcpyDeviceToHost,
               queue, blockOn, (!finalCopy) ? nullptr : completion);

    dst += elementSize;
    dstOffset += bufferElementSize;
  }

  return dstOffset;
}


void *Buffer::arg_() const
{
  return _imp->mem;
}


void *Buffer::pin(PinMode mode)
{
  return nullptr;
}


void Buffer::unpin(void *ptr, Queue *queue, Event *blockOn, Event *completion)
{
}


namespace gputil
{
  size_t copyBuffer(Buffer &dst, const Buffer &src, Queue *queue, Event *blockOn, Event *completion)
  {
    return copyBuffer(dst, 0, src, 0, src.size(), queue, blockOn, completion);
  }


  size_t copyBuffer(Buffer &dst, const Buffer &src, size_t byteCount,
                    Queue *queue, Event *blockOn, Event *completion)
  {
    return copyBuffer(dst, 0, src, 0, byteCount, queue, blockOn, completion);
  }


  size_t copyBuffer(Buffer &dst, size_t dstOffset, const Buffer &src, size_t srcOffset, size_t byteCount,
                    Queue *queue, Event *blockOn, Event *completion)
  {
    BufferDetail *dstDetail = dst.detail();
    BufferDetail *srcDetail = src.detail();
    if (!srcDetail || !dstDetail)
    {
      return 0;
    }

    // Check offsets.
    if (dstDetail->allocSize < dstOffset)
    {
      return 0;
    }

    if (srcDetail->allocSize < srcOffset)
    {
      return 0;
    }

    // Check sizes after offset.
    byteCount = std::min(byteCount, dstDetail->allocSize - dstOffset);
    byteCount = std::min(byteCount, srcDetail->allocSize - srcOffset);

    bufferCopy(dstDetail->mem, srcDetail->mem, byteCount, cudaMemcpyDeviceToDevice, queue, blockOn, completion);

    return byteCount;
  }
}
