// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gputil/gpubuffer.h"

#include "gpuapiexception.h"
#include "gpubufferdetail.h"
#include "gpudevicedetail.h"
#include "gpuqueuedetail.h"

#include "cl/gpueventdetail.h"

#include <algorithm>
#include <cinttypes>
#include <clu/clu.h>
#include <clu/clubuffer.h>
#include <cstring>

#define USE_PINNED 1

using namespace gputil;

namespace
{
  /// Helper function to select either the command queue from @p explicitQueue or the default command queue for @p device.
  /// @param device The device holding the default command queue.
  /// @param explicitQueue The preferred command queue. May be null.
  inline cl_command_queue selectQueue(Device &device, Queue *explicitQueue)
  {
    if (explicitQueue)
    {
      return explicitQueue->internal()->queue();
    }
    return device.detail()->queue();
  }


  size_t resizeBuffer(Buffer &buffer, BufferDetail &imp, size_t newSize, bool force)
  {
    size_t actualSize = buffer.actualSize();
    size_t bestSize = clu::bestAllocationSize(imp.device.detail()->context, newSize);
    if (!force && actualSize >= bestSize || force && actualSize == bestSize)
    {
      imp.requestedSize = newSize;
      return bestSize;
    }

    // Needs resize.
    imp.buffer = cl::Buffer();
    actualSize = 0;

    cl_mem_flags clFlags = 0;

    if (imp.flags & BF_Read)
    {
      if (imp.flags & BF_Write)
      {
        clFlags = CL_MEM_READ_WRITE;
      }
      else
      {
        clFlags = CL_MEM_READ_ONLY;
      }
    }
    else if (imp.flags & BF_Write)
    {
      clFlags = CL_MEM_WRITE_ONLY;
    }

    if (imp.flags & BF_HostAccess)
    {
      clFlags = CL_MEM_ALLOC_HOST_PTR;
    }

    cl_int clerr = 0;
    clu::ensureBufferSize<uint8_t>(imp.buffer, clFlags, imp.device.detail()->context, newSize, &clerr);
    GPUAPICHECK(clerr, CL_SUCCESS, 0);

    imp.requestedSize = newSize;
    actualSize = buffer.actualSize();

    return actualSize;
  }


  size_t actualSize(BufferDetail &imp)
  {
    size_t bufferSize = 0;
    if (imp.buffer())
    {
      clGetMemObjectInfo(imp.buffer(), CL_MEM_SIZE, sizeof(bufferSize), &bufferSize, nullptr);
    }
    return bufferSize;
  }
}


namespace gputil
{
  uint8_t *pin(BufferDetail &imp, PinMode mode)
  {
    cl_command_queue clQueue = selectQueue(imp.device, nullptr);
    cl_map_flags mapFlags = 0;

    switch (mode)
    {
    case PinRead:
      mapFlags = CL_MAP_READ;
      break;
    case PinWrite:
      mapFlags = CL_MAP_WRITE;
      break;
    case PinReadWrite:
      mapFlags = CL_MAP_WRITE | CL_MAP_WRITE;
      break;
    }

    cl_int clerr = CL_SUCCESS;
    uint8_t *pinnedPtr = (uint8_t *)clEnqueueMapBuffer(clQueue, imp.buffer(),
                                                       CL_TRUE, mapFlags, 0, actualSize(imp),
                                                       0, nullptr,
                                                       nullptr, &clerr);
                                                      //  &event, &clerr);
    GPUAPICHECK(clerr, CL_SUCCESS, nullptr);

    return pinnedPtr;
  }


  void unpin(BufferDetail &imp, void *pinnedPtr, Queue *explicitQueue, Event *blockOn, Event *completion)
  {
    if (completion)
    {
      completion->release();
    }

    if (pinnedPtr)
    {
      cl_command_queue clQueue = selectQueue(imp.device, explicitQueue);
      cl_int clerr = CL_SUCCESS;

      cl_event event;
      cl_event *eventPtr = (!explicitQueue || completion) ? &event : nullptr;
      int blockOnCount = (blockOn && blockOn->isValid()) ? 1 : 0;
      cl_event blockOnOcl = (blockOnCount) ? blockOn->detail()->event : nullptr;

      clerr = clEnqueueUnmapMemObject(clQueue, imp.buffer(), pinnedPtr,
                                      blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                                      eventPtr);

      GPUAPICHECK2(clerr, CL_SUCCESS);

      if (completion)
      {
        completion->detail()->event = event;
      }
      else if (eventPtr)
      {
        // Wait for DMA to complete.
        clerr = clWaitForEvents(1, eventPtr);
        GPUAPICHECK2(clerr, CL_SUCCESS);
        if (clerr != CL_SUCCESS)
        GPUAPICHECK2(clerr, CL_SUCCESS);
      }
    }
  }
}

Buffer::Buffer()
  : _imp(new BufferDetail)
{
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
  if (_imp)
  {
    delete _imp;
  }
}


Buffer &Buffer::operator=(Buffer &&other)
{
  delete _imp;
  _imp = other._imp;
  other._imp = nullptr;
  return *this;
}


void Buffer::create(const Device &device, size_t byteSize, unsigned flags)
{
  if (isValid())
  {
    release();
  }
  _imp->device = device;
  _imp->flags = flags;
  resize(byteSize);
}


void Buffer::release()
{
  _imp->device = Device();
  _imp->buffer = cl::Buffer();
  _imp->flags = 0;
}


void Buffer::swap(Buffer &other)
{
  std::swap(_imp, other._imp);
}


bool Buffer::isValid() const
{
  // Can't check _imp->buffer here as it's valid to have a zero sized buffer.
  return _imp && _imp->device.isValid();
}


unsigned Buffer::flags() const
{
  return _imp->flags;
}


size_t Buffer::size() const
{
  return _imp->requestedSize;
}

size_t Buffer::actualSize() const
{
  return ::actualSize(*_imp);
}


size_t Buffer::resize(size_t newSize)
{
  return resizeBuffer(*this, *_imp, newSize, false);
}


size_t Buffer::forceResize(size_t newSize)
{
  return resizeBuffer(*this, *_imp, newSize, true);
}


void Buffer::fill(const void *pattern, size_t patternSize,
                  Queue *queue, Event *blockOn, Event *completion)
{
  if (completion)
  {
    completion->release();
  }

  if (isValid())
  {
    cl_int clerr = CL_SUCCESS;
    cl_command_queue clQueue = selectQueue(_imp->device, queue);

    // Note: clEnqueueFillBuffer() appears to be faster than memory mapping. That is, it doesn't
    // suffer the same performance issues as clEnqueueReadBuffer()/clEnqueueWriteBuffer().
    GPUAPICHECK2(clerr, CL_SUCCESS);

    int blockOnCount = (blockOn && blockOn->isValid()) ? 1 : 0;
    cl_event blockOnOcl = (blockOnCount) ? blockOn->detail()->event : nullptr;
    clerr = clEnqueueFillBuffer(clQueue, _imp->buffer(),
                                pattern, patternSize, 0, actualSize(),
                                blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                                (queue && completion) ? &completion->detail()->event : nullptr);
    GPUAPICHECK2(clerr, CL_SUCCESS);

    if (!queue)
    {
      // Blocking operation.
      clerr = clFinish(clQueue);
      GPUAPICHECK2(clerr, CL_SUCCESS);
    }
  }
}


void Buffer::fillPartial(const void *pattern, size_t patternSize, size_t fillBytes, size_t offset,
                         Queue *queue)
{
  // Contrain the byte counts.
  const size_t thisBufferSize = size();
  if (offset >= thisBufferSize)
  {
    return;
  }

  fillBytes = std::min(fillBytes, thisBufferSize - offset);

#ifdef USE_PINNED
  if (_imp->flags & BF_HostAccess)
  {
    // Using pinned memory. Make multiple memcpy calls.
    if (uint8_t *dstMem = ::pin(*_imp, PinWrite))
    {
      size_t filled = 0u;
      for (; filled < fillBytes; filled += patternSize)
      {
        memcpy(dstMem + offset + filled, pattern, patternSize);
      }

      if (filled < fillBytes)
      {
        // Finish the last (partial pattern) write.
        size_t lastWriteSize = fillBytes - filled;
        memcpy(dstMem + offset + filled, pattern, lastWriteSize);
      }

      ::unpin(*_imp, dstMem, queue);
    }
    return;
  }
#endif // USE_PINNED

  // Not pinned memory. Make multiple writes.
  size_t filled = 0u;
  for (; filled < fillBytes; filled += patternSize)
  {
    write(pattern, patternSize, offset + filled, queue);
  }

  if (filled < fillBytes)
  {
    // Finish the last (partial pattern) write.
    size_t lastWriteSize = fillBytes - filled;
    write(pattern, lastWriteSize, offset + filled, queue);
  }
}


size_t Buffer::read(void *dst, size_t readByteCount, size_t srcOffset,
                    Queue *queue, Event *blockOn, Event *completion)
{
  // Constrain the byte counts.
  const size_t thisBufferSize = size();
  if (completion)
  {
    completion->release();
  }

  if (srcOffset >= thisBufferSize)
  {
    return 0;
  }

  cl_int clerr;
  const size_t copyBytes = std::min(readByteCount, thisBufferSize - srcOffset);

#if USE_PINNED
  // Can only use pinned memory for non-asynchronous transfer.
  if ((_imp->flags & BF_HostAccess) && !queue)
  {
    if (blockOn && blockOn->isValid())
    {
      blockOn->wait();
    }

    if (uint8_t *srcMem = ::pin(*_imp, PinRead))
    {
      memcpy(dst, srcMem + srcOffset, copyBytes);
      ::unpin(*_imp, srcMem, queue);
      return copyBytes;
    }
  }
#endif // USE_PINNED

  cl_command_queue clQueue = selectQueue(_imp->device, queue);
  int blockOnCount = (blockOn && blockOn->isValid()) ? 1 : 0;
  cl_event blockOnOcl = (blockOnCount) ? blockOn->detail()->event : nullptr;

  // Non-blocking when an explicit queue has been provided.
  #if 1
  clerr = clEnqueueReadBuffer(clQueue, _imp->buffer(),
                              (!queue) ? CL_TRUE : CL_FALSE,
                              srcOffset, copyBytes, dst,
                              blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                              (queue && completion) ? &completion->detail()->event : nullptr);
  // if (queue && completion)
  // {
  //   clWaitForEvents(1, &completion->detail()->event);
  // }
  #else  // #
  clerr = clEnqueueReadBuffer(clQueue, _imp->buffer(),
                              CL_TRUE,
                              srcOffset, copyBytes, dst,
                              blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                              nullptr);
  #endif // #

  GPUAPICHECK(clerr, CL_SUCCESS, 0u);

  return copyBytes;
}


size_t Buffer::write(const void *src, size_t byteCount, size_t dstOffset,
                     Queue *queue, Event *blockOn, Event *completion)

{
  // Contrain the byte counts.
  const size_t thisBufferSize = size();
  if (completion)
  {
    completion->release();
  }

  if (dstOffset >= thisBufferSize)
  {
    return 0;
  }

  const size_t copyBytes = std::min(byteCount, thisBufferSize - dstOffset);
  cl_int clerr;

#ifdef USE_PINNED
  if (_imp->flags & BF_HostAccess)
  {
    if (uint8_t *dstMem = ::pin(*_imp, PinWrite))
    {
      memcpy(dstMem + dstOffset, src, copyBytes);
      ::unpin(*_imp, dstMem, queue, blockOn, completion);
    }
    return copyBytes;
  }
#endif // USE_PINNED

  cl_command_queue clQueue = selectQueue(_imp->device, queue);
  int blockOnCount = (blockOn && blockOn->isValid()) ? 1 : 0;
  cl_event blockOnOcl = (blockOnCount) ? blockOn->detail()->event : nullptr;

  // Non-blocking when an explicit queue has been provided.
  clerr = clEnqueueWriteBuffer(clQueue, _imp->buffer(),
                               (!queue) ? CL_TRUE : CL_FALSE,
                               dstOffset, copyBytes, src,
                               blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                               (queue && completion) ? &completion->detail()->event : nullptr);

  GPUAPICHECK(clerr, CL_SUCCESS, 0u);

  return copyBytes;
}


size_t Buffer::readElements(void *dst, size_t elementSize, size_t elementCount, size_t offsetElements, size_t bufferElementSize,
                            Queue *queue, Event *blockOn, Event *completion)
{
  if (completion)
  {
    completion->release();
  }

  if (!isValid() && !dst)
  {
    return 0u;
  }

  if (!bufferElementSize || elementSize == bufferElementSize)
  {
    // Matching element size. Use normal write.
    return read(dst, elementSize * elementCount, elementSize * offsetElements, queue);
  }

  // Calculate the read offset based on the passed bufferElementSize.
  const size_t byteReadOffset = offsetElements * bufferElementSize;
  if (byteReadOffset >= size())
  {
    return 0u;
  }

  // Element size mismatch. Use piecewise write.
  const size_t copySize = std::min(elementSize, bufferElementSize);
  const size_t copyElementCount = std::min(elementCount, (size() - byteReadOffset) / bufferElementSize);
  uint8_t *dstMem = static_cast<uint8_t *>(dst);

  if (copyElementCount)
  {
#if USE_PINNED
    // Mapped memory only when not using an explicit queue as queue implies a non-blocking call.
    if (!queue && (_imp->flags & BF_HostAccess))
    {
      if (blockOn && blockOn->isValid())
      {
        blockOn->wait();
      }

      if (uint8_t *pinnedPtr = ::pin(*_imp, PinRead))
      {
        uint8_t *srcMem = pinnedPtr;
        srcMem += byteReadOffset;
        for (size_t i = 0; i < copyElementCount; ++i)
        {
          memcpy(dstMem, srcMem, copySize);
          dstMem += elementSize;
          srcMem += bufferElementSize;
        }

        ::unpin(*_imp, pinnedPtr, queue);
        return copyElementCount;
      }
    }
#endif // USE_PINNED

    size_t bufferOffset = byteReadOffset;
    cl_int clerr = CL_SUCCESS;
    cl_int clerr2;
    cl_command_queue clQueue = selectQueue(_imp->device, queue);
    int blockOnCount = (blockOn && blockOn->isValid()) ? 1 : 0;
    cl_event blockOnOcl = (blockOnCount) ? blockOn->detail()->event : nullptr;

    // Unfortunately I can't find whether the copy order is guaranteed for clEnqueueWriteBuffer(). To support the
    // completeion event, we'll set barrier after the copy instructions and wait on that.
    for (size_t i = 0; i < copyElementCount; ++i)
    {
      clerr2 = clEnqueueReadBuffer(clQueue, _imp->buffer(), CL_FALSE,
                                  bufferOffset, copySize, dstMem,
                                  blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                                  nullptr);
      dstMem += elementSize;
      bufferOffset += bufferElementSize;

      clerr = (clerr) ? clerr : clerr2;
    }

    if (queue && completion)
    {
      // Set a barrier to ensure the copy commands complete and wait on that event.
      clEnqueueBarrierWithWaitList(clQueue, 0, nullptr, &completion->detail()->event);
    }

    // Blocking operation when no explicit queue has been provided.
    if (!queue)
    {
      clerr2 = clFinish(clQueue);
      clerr = (clerr) ? clerr : clerr2;
    }

    GPUAPICHECK(clerr, CL_SUCCESS, 0);
  }

  return copyElementCount;
}


size_t Buffer::writeElements(const void *src, size_t elementSize, size_t elementCount, size_t offsetElements, size_t bufferElementSize,
                             Queue *queue, Event *blockOn, Event *completion)
{
  if (completion)
  {
    completion->release();
  }

  if (!isValid() && !src)
  {
    return 0u;
  }

  if (!bufferElementSize || elementSize == bufferElementSize)
  {
    // Matching element size. Use normal write.
    return write(src, elementSize * elementCount, offsetElements * elementSize, queue);
  }

  const size_t byteWriteOffset = offsetElements * bufferElementSize;
  if (byteWriteOffset >= size())
  {
    return 0u;
  }

  // Element size mismatch. Use piecewise write.
  const size_t copySize = std::min(elementSize, bufferElementSize);
  const size_t copyElementCount = std::min(elementCount, (size() - byteWriteOffset) / bufferElementSize);
  const size_t clearByteCount = (bufferElementSize > elementSize) ? bufferElementSize - elementSize : 0u;
  // const_cast because OpenCL API does not support const.
  const uint8_t *srcMem = static_cast<const uint8_t *>(src);

  if (copyElementCount)
  {
#if USE_PINNED
    // Mapped memory only when not using an explicit queue as queue implies a non-blocking call.
    if (!queue && (_imp->flags & BF_HostAccess))
    {
      if (blockOn && blockOn->isValid())
      {
        blockOn->wait();
      }

      if (uint8_t *pinnedPtr = ::pin(*_imp, PinWrite))
      {
        uint8_t *dstMem = pinnedPtr;
        dstMem += byteWriteOffset;
        for (size_t i = 0; i < copyElementCount; ++i)
        {
          memcpy(dstMem, srcMem, copySize);
          // Clear any extra bytes.
          if (clearByteCount)
          {
            memset(dstMem + elementSize, 0, clearByteCount);
          }
          dstMem += bufferElementSize;
          srcMem += elementSize;
        }

        ::unpin(*_imp, pinnedPtr, queue);
        return copyElementCount;
      }
    }
#endif // USE_PINNED

    size_t bufferOffset = byteWriteOffset;
    cl_int clerr = CL_SUCCESS;
    cl_int clerr2;
    cl_command_queue clQueue = selectQueue(_imp->device, queue);
    int blockOnCount = (blockOn && blockOn->isValid()) ? 1 : 0;
    cl_event blockOnOcl = (blockOnCount) ? blockOn->detail()->event : nullptr;

    // Unfortunately I can't find whether the copy order is guaranteed for clEnqueueWriteBuffer(). To support the
    // completeion event, we'll set barrier after the copy instructions and wait on that.
    for (size_t i = 0; i < copyElementCount; ++i)
    {
      clerr2 = clEnqueueWriteBuffer(clQueue, _imp->buffer(), CL_FALSE,
                                    bufferOffset, copySize, srcMem,
                                    blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                                    nullptr);
      srcMem += elementSize;
      bufferOffset += bufferElementSize;

      clerr = (clerr) ? clerr : clerr2;
    }

    if (queue && completion)
    {
      // Set a barrier to ensure the copy commands complete and wait on that event.
      clEnqueueBarrierWithWaitList(clQueue, 0, nullptr, &completion->detail()->event);
    }

    // Blocking operation when no explicit queue has been provided.
    if (!queue)
    {
      clerr2 = clFinish(clQueue);
      clerr = (clerr) ? clerr : clerr2;
    }

    GPUAPICHECK(clerr, CL_SUCCESS, 0);
  }

  return copyElementCount;
}


void *Buffer::arg_() const
{
  if (_imp)
  {
    return _imp->buffer();
  }

  return nullptr;
}


void *Buffer::pin(PinMode mode)
{
  return ::pin(*_imp, mode);
}


void Buffer::unpin(void *ptr, Queue *queue, Event *blockOn, Event *completion)
{
  ::unpin(*_imp, ptr, queue, blockOn, completion);
}


namespace gputil
{
  size_t copyBuffer(Buffer &dst, const Buffer &src, Queue *queue, Event *blockOn, Event *completion)
  {
    return copyBuffer(dst, 0, src, 0, src.size(), queue);
  }


  size_t copyBuffer(Buffer &dst, const Buffer &src, size_t byteCount, Queue *queue, Event *blockOn, Event *completion)
  {
    return copyBuffer(dst, 0, src, 0, byteCount, queue);
  }


  size_t copyBuffer(Buffer &dst, size_t dstOffset, const Buffer &src, size_t srcOffset, size_t byteCount,
                    Queue *queue, Event *blockOn, Event *completion)
  {
    const size_t dstSize = dst.size();
    const size_t srcSize = src.size();

    if (completion)
    {
      completion->release();
    }

    // Check offsets.
    if (dstSize < dstOffset)
    {
      return 0;
    }

    if (srcSize < srcOffset)
    {
      return 0;
    }

    // Check sizes after offset.
    byteCount = std::min(byteCount, dstSize - dstOffset);
    byteCount = std::min(byteCount, srcSize - srcOffset);

    cl_mem srcMemCL = src.arg<cl_mem>();
    cl_mem dstMemCL = dst.arg<cl_mem>();

    cl_int clerr;
    cl_command_queue clQueue = selectQueue(src.detail()->device, queue);
    int blockOnCount = (blockOn && blockOn->isValid()) ? 1 : 0;
    cl_event blockOnOcl = (blockOnCount) ? blockOn->detail()->event : nullptr;

    clerr = clEnqueueCopyBuffer(clQueue,
                                srcMemCL, dstMemCL, srcOffset, dstOffset,
                                byteCount,
                                blockOnCount, (blockOnCount) ? &blockOnOcl : nullptr,
                                (queue && completion) ? &completion->detail()->event : nullptr);

    GPUAPICHECK(clerr, CL_SUCCESS, 0);

    // Block if no explicit queue provided.
    if (!queue)
    {
      clerr = clFinish(clQueue);
      GPUAPICHECK(clerr, CL_SUCCESS, 0u);
    }

    return byteCount;
  }
}
