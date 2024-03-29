// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gputil/gpuBuffer.h"

#include "gpuApiException.h"
#include "gpuBufferDetail.h"
#include "gpuDeviceDetail.h"
#include "gpuQueueDetail.h"

#include "cl/gpuEventDetail.h"

#include <clu/clu.h>
#include <clu/cluBuffer.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>

#define USE_PINNED 1

namespace
{
/// Helper function to select either the command queue from @p explicitQueue or the default command queue for @p
/// device.
/// @param device The device holding the default command queue.
/// @param explicit_queue The preferred command queue. May be null.
inline cl_command_queue selectQueue(gputil::Device &device, gputil::Queue *explicit_queue)
{
  if (explicit_queue)
  {
    return explicit_queue->internal()->queue();
  }
  return device.detail()->default_queue.internal()->queue();
}


size_t resizeBuffer(gputil::Buffer &buffer, gputil::BufferDetail &imp, size_t new_size, bool force)
{
  const size_t initial_size = buffer.actualSize();
  size_t best_size = clu::bestAllocationSize(imp.device.detail()->context, new_size);
  if ((!force && initial_size >= best_size) || (force && initial_size == best_size))
  {
    imp.requested_size = new_size;
    return best_size;
  }

  // Needs resize.
  imp.buffer = cl::Buffer();

  cl_mem_flags cl_flags = 0;

  if (imp.flags & gputil::kBfRead)
  {
    if (imp.flags & gputil::kBfWrite)
    {
      cl_flags = CL_MEM_READ_WRITE;  // NOLINT(hicpp-signed-bitwise)
    }
    else
    {
      cl_flags = CL_MEM_READ_ONLY;  // NOLINT(hicpp-signed-bitwise)
    }
  }
  else if (imp.flags & gputil::kBfWrite)
  {
    cl_flags = CL_MEM_WRITE_ONLY;  // NOLINT(hicpp-signed-bitwise)
  }

  if (imp.request_flags & gputil::kBfHostAccess)
  {
    cl_flags = CL_MEM_ALLOC_HOST_PTR;  // NOLINT(hicpp-signed-bitwise)
  }

  cl_int clerr = 0;
  clu::ensureBufferSize<uint8_t>(imp.buffer, cl_flags, imp.device.detail()->context, new_size, &clerr);
  GPUAPICHECK(clerr, CL_SUCCESS, 0);

  // Validate the CL_MEM_ALLOC_HOST_PTR flag worked.
  cl_mem_flags actual_flags = 0;
  clGetMemObjectInfo(imp.buffer(), CL_MEM_FLAGS, sizeof(actual_flags), &actual_flags, nullptr);

  imp.flags = imp.request_flags;
  // NOLINTNEXTLINE(hicpp-signed-bitwise)
  if ((imp.request_flags & gputil::kBfHostAccess) && !(actual_flags & CL_MEM_ALLOC_HOST_PTR))
  {
    // Failed to allocate in host memory.
    imp.flags &= ~gputil::kBfHostAccess;
    // std::cout << "Failed host access " << std::endl;
  }

  imp.requested_size = new_size;
  const size_t actual_size = buffer.actualSize();

  return actual_size;
}


size_t actualSize(const gputil::BufferDetail &imp)
{
  size_t buffer_size = 0;
  if (imp.buffer())
  {
    clGetMemObjectInfo(imp.buffer(), CL_MEM_SIZE, sizeof(buffer_size), &buffer_size, nullptr);
  }
  return buffer_size;
}
}  // namespace


namespace gputil
{
uint8_t *pin(BufferDetail &imp, PinMode mode)
{
  cl_command_queue queue_cl = selectQueue(imp.device, nullptr);
  cl_map_flags map_flags = 0;

  switch (mode)
  {
  default:
  case kPinNone:
    break;
  case kPinRead:
    map_flags = CL_MAP_READ;  // NOLINT(hicpp-signed-bitwise)
    break;
  case kPinWrite:
    map_flags = CL_MAP_WRITE;  // NOLINT(hicpp-signed-bitwise)
    break;
  case kPinReadWrite:
    map_flags = CL_MAP_READ | CL_MAP_WRITE;  // NOLINT(hicpp-signed-bitwise)
    break;
  }

  cl_int clerr = CL_SUCCESS;
  auto *pinned_ptr = static_cast<uint8_t *>(
    clEnqueueMapBuffer(queue_cl, imp.buffer(), CL_TRUE, map_flags, 0, actualSize(imp), 0, nullptr, nullptr, &clerr));
  //  &event, &clerr);
  GPUAPICHECK(clerr, CL_SUCCESS, nullptr);

  return pinned_ptr;
}


void unpin(BufferDetail &imp, void *pinned_ptr, Queue *explicit_queue, Event *block_on, Event *completion)
{
  if (completion)
  {
    completion->release();
  }

  if (pinned_ptr)
  {
    cl_command_queue queue_cl = selectQueue(imp.device, explicit_queue);
    cl_int clerr = CL_SUCCESS;

    cl_event event{};
    cl_event *event_ptr = (!explicit_queue || completion) ? &event : nullptr;
    int block_on_count = (block_on && block_on->isValid()) ? 1 : 0;
    cl_event block_on_ocl = (block_on_count) ? block_on->detail()->event : nullptr;

    clerr = clEnqueueUnmapMemObject(queue_cl, imp.buffer(), pinned_ptr, block_on_count,
                                    (block_on_count) ? &block_on_ocl : nullptr, event_ptr);

    GPUAPICHECK2(clerr, CL_SUCCESS);

    if (completion)
    {
      completion->detail()->event = event;
    }
    else if (event_ptr)
    {
      // Wait for DMA to complete.
      clerr = clWaitForEvents(1, event_ptr);
      GPUAPICHECK2(clerr, CL_SUCCESS);
      if (clerr != CL_SUCCESS)
      {
        GPUAPICHECK2(clerr, CL_SUCCESS);
      }
    }
  }
}

Buffer::Buffer()
  : imp_(new BufferDetail)
{}


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
  delete imp_;
}


Buffer &Buffer::operator=(Buffer &&other) noexcept
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}


void Buffer::create(const Device &device, size_t byte_size, unsigned flags)
{
  if (isValid())
  {
    release();
  }
  imp_->device = device;
  imp_->request_flags = imp_->flags = flags;
  resize(byte_size);
}


void Buffer::release()
{
  imp_->device = Device();
  imp_->buffer = cl::Buffer();
  imp_->request_flags = imp_->flags = 0;
}


void Buffer::swap(Buffer &other) noexcept
{
  std::swap(imp_, other.imp_);
}


bool Buffer::isValid() const
{
  // Can't check _imp->buffer here as it's valid to have a zero sized buffer.
  return imp_ && imp_->device.isValid();
}


unsigned Buffer::flags() const
{
  return imp_->flags;
}


size_t Buffer::size() const
{
  return imp_->requested_size;
}

size_t Buffer::actualSize() const
{
  return ::actualSize(*imp_);
}


size_t Buffer::resize(size_t new_size)
{
  return resizeBuffer(*this, *imp_, new_size, false);
}


size_t Buffer::forceResize(size_t new_size)
{
  return resizeBuffer(*this, *imp_, new_size, true);
}


void Buffer::fill(const void *pattern, size_t pattern_size, Queue *queue, Event *block_on, Event *completion)
{
  if (completion)
  {
    completion->release();
  }

  if (isValid())
  {
    cl_int clerr = CL_SUCCESS;
    cl_command_queue ocl_queue = selectQueue(imp_->device, queue);

    // Note: clEnqueueFillBuffer() appears to be faster than memory mapping. That is, it doesn't
    // suffer the same performance issues as clEnqueueReadBuffer()/clEnqueueWriteBuffer().
    GPUAPICHECK2(clerr, CL_SUCCESS);

    const int block_on_count = (block_on && block_on->isValid()) ? 1 : 0;
    cl_event block_on_ocl = (block_on_count) ? block_on->detail()->event : nullptr;
    clerr = clEnqueueFillBuffer(ocl_queue, imp_->buffer(), pattern, pattern_size, 0, actualSize(), block_on_count,
                                (block_on_count) ? &block_on_ocl : nullptr,
                                (queue && completion) ? &completion->detail()->event : nullptr);
    GPUAPICHECK2(clerr, CL_SUCCESS);

    if (!queue || queue->internal()->force_synchronous)
    {
      // Blocking operation.
      clerr = clFinish(ocl_queue);
      GPUAPICHECK2(clerr, CL_SUCCESS);
    }
  }
}


void Buffer::fillPartial(const void *pattern, size_t pattern_size, size_t fill_bytes, size_t offset, Queue *queue)
{
  // Contrain the byte counts.
  const size_t this_buffer_size = size();
  if (offset >= this_buffer_size)
  {
    return;
  }

  fill_bytes = std::min(fill_bytes, this_buffer_size - offset);

#ifdef USE_PINNED
  if (imp_->flags & kBfHostAccess)
  {
    // Using pinned memory. Make multiple memcpy calls.
    if (uint8_t *dst_mem = gputil::pin(*imp_, kPinWrite))
    {
      size_t filled = 0u;
      for (; filled < fill_bytes; filled += pattern_size)
      {
        memcpy(dst_mem + offset + filled, pattern, pattern_size);
      }

      if (filled < fill_bytes)
      {
        // Finish the last (partial pattern) write.
        const size_t last_write_size = fill_bytes - filled;
        memcpy(dst_mem + offset + filled, pattern, last_write_size);
      }

      gputil::unpin(*imp_, dst_mem, queue);
    }
    return;
  }
#endif  // USE_PINNED

  // Not pinned memory. Make multiple writes.
  size_t filled = 0u;
  for (; filled < fill_bytes; filled += pattern_size)
  {
    write(pattern, pattern_size, offset + filled, queue);
  }

  if (filled < fill_bytes)
  {
    // Finish the last (partial pattern) write.
    size_t last_write_size = fill_bytes - filled;
    write(pattern, last_write_size, offset + filled, queue);
  }
}


size_t Buffer::read(void *dst, size_t read_byte_count, size_t src_offset, Queue *queue, Event *block_on,
                    Event *completion)
{
  // Constrain the byte counts.
  const size_t this_buffer_size = size();
  if (completion)
  {
    completion->release();
  }

  if (src_offset >= this_buffer_size)
  {
    return 0;
  }

  cl_int clerr = CL_SUCCESS;
  const size_t copy_bytes = std::min(read_byte_count, this_buffer_size - src_offset);

#if USE_PINNED
  // Can only use pinned memory for non-asynchronous transfer.
  if ((imp_->flags & kBfHostAccess) && !queue)
  {
    if (block_on && block_on->isValid())
    {
      block_on->wait();
    }

    if (uint8_t *src_mem = gputil::pin(*imp_, kPinRead))
    {
      memcpy(dst, src_mem + src_offset, copy_bytes);
      gputil::unpin(*imp_, src_mem, queue);
      return copy_bytes;
    }
  }
#endif  // USE_PINNED

  cl_command_queue ocl_queue = selectQueue(imp_->device, queue);
  int block_on_count = (block_on && block_on->isValid()) ? 1 : 0;
  cl_event block_on_ocl = (block_on_count) ? block_on->detail()->event : nullptr;

  // Non-blocking when an explicit queue has been provided.
  const cl_bool synchronous = (!queue || queue->internal()->force_synchronous) ? CL_TRUE : CL_FALSE;
  clerr = clEnqueueReadBuffer(ocl_queue, imp_->buffer(), synchronous, src_offset, copy_bytes, dst, block_on_count,
                              (block_on_count) ? &block_on_ocl : nullptr,
                              (queue && completion) ? &completion->detail()->event : nullptr);

  GPUAPICHECK(clerr, CL_SUCCESS, 0u);

  return copy_bytes;
}


size_t Buffer::write(const void *src, size_t byte_count, size_t dst_offset, Queue *queue, Event *block_on,
                     Event *completion)

{
  // Contrain the byte counts.
  const size_t this_buffer_size = size();
  if (completion)
  {
    completion->release();
  }

  if (dst_offset >= this_buffer_size)
  {
    return 0;
  }

  const size_t copy_bytes = std::min(byte_count, this_buffer_size - dst_offset);
  cl_int clerr = CL_SUCCESS;

#ifdef USE_PINNED
  if (imp_->flags & kBfHostAccess)
  {
    if (uint8_t *dst_mem = gputil::pin(*imp_, kPinWrite))
    {
      memcpy(dst_mem + dst_offset, src, copy_bytes);
      gputil::unpin(*imp_, dst_mem, queue, block_on, completion);
    }
    return copy_bytes;
  }
#endif  // USE_PINNED

  cl_command_queue queue_cl = selectQueue(imp_->device, queue);
  const int block_on_count = (block_on && block_on->isValid()) ? 1 : 0;
  cl_event block_on_ocl = (block_on_count) ? block_on->detail()->event : nullptr;

  // Non-blocking when an explicit queue has been provided.
  const cl_bool synchronous = (!queue || queue->internal()->force_synchronous) ? CL_TRUE : CL_FALSE;
  clerr = clEnqueueWriteBuffer(queue_cl, imp_->buffer(), synchronous, dst_offset, copy_bytes, src, block_on_count,
                               (block_on_count) ? &block_on_ocl : nullptr,
                               (queue && completion) ? &completion->detail()->event : nullptr);

  GPUAPICHECK(clerr, CL_SUCCESS, 0u);

  return copy_bytes;
}


size_t Buffer::readElements(void *dst, size_t element_size, size_t element_count, size_t offset_elements,
                            size_t buffer_element_size, Queue *queue, Event *block_on, Event *completion)
{
  if (completion)
  {
    completion->release();
  }

  if (!isValid() && !dst)
  {
    return 0u;
  }

  if (!buffer_element_size || element_size == buffer_element_size)
  {
    // Matching element size. Use normal write.
    return read(dst, element_size * element_count, element_size * offset_elements, queue, block_on, completion);
  }

  // Calculate the read offset based on the passed bufferElementSize.
  const size_t byte_read_offset = offset_elements * buffer_element_size;
  if (byte_read_offset >= size())
  {
    return 0u;
  }

  // Element size mismatch. Use piecewise write.
  const size_t copy_size = std::min(element_size, buffer_element_size);
  const size_t copy_element_count = std::min(element_count, (size() - byte_read_offset) / buffer_element_size);
  auto *dst_mem = static_cast<uint8_t *>(dst);

  if (copy_element_count)
  {
#if USE_PINNED
    // Mapped memory only when not using an explicit queue as queue implies a non-blocking call.
    if (!queue && (imp_->flags & kBfHostAccess))
    {
      if (block_on && block_on->isValid())
      {
        block_on->wait();
      }

      if (uint8_t *pinned_ptr = gputil::pin(*imp_, kPinRead))
      {
        uint8_t *src_mem = pinned_ptr;
        src_mem += byte_read_offset;
        for (size_t i = 0; i < copy_element_count; ++i)
        {
          memcpy(dst_mem, src_mem, copy_size);
          dst_mem += element_size;
          src_mem += buffer_element_size;
        }

        gputil::unpin(*imp_, pinned_ptr, queue);
        return copy_element_count;
      }
    }
#endif  // USE_PINNED

    size_t buffer_offset = byte_read_offset;
    cl_int clerr = CL_SUCCESS;
    cl_int clerr2 = CL_SUCCESS;
    cl_command_queue queue_cl = selectQueue(imp_->device, queue);
    const int block_on_count = (block_on && block_on->isValid()) ? 1 : 0;
    cl_event block_on_ocl = (block_on_count) ? block_on->detail()->event : nullptr;

    // Unfortunately I can't find whether the copy order is guaranteed for clEnqueueReadBuffer(). To support the
    // completeion event, we'll set barrier after the copy instructions and wait on that.
    const cl_bool synchronous = (!queue || queue->internal()->force_synchronous) ? CL_TRUE : CL_FALSE;
    for (size_t i = 0; i < copy_element_count; ++i)
    {
      clerr2 = clEnqueueReadBuffer(queue_cl, imp_->buffer(), synchronous, buffer_offset, copy_size, dst_mem,
                                   block_on_count, (block_on_count) ? &block_on_ocl : nullptr, nullptr);
      dst_mem += element_size;
      buffer_offset += buffer_element_size;

      clerr = (clerr) ? clerr : clerr2;
    }

    if (queue && completion)
    {
      // Set a barrier to ensure the copy commands complete and wait on that event.
      clEnqueueBarrierWithWaitList(queue_cl, 0, nullptr, &completion->detail()->event);
    }

    // Blocking operation when no explicit queue has been provided.
    if (!queue)
    {
      clerr2 = clFinish(queue_cl);
      clerr = (clerr) ? clerr : clerr2;
    }

    GPUAPICHECK(clerr, CL_SUCCESS, 0);
  }

  return copy_element_count;
}


size_t Buffer::writeElements(const void *src, size_t element_size, size_t element_count, size_t offset_elements,
                             size_t buffer_element_size, Queue *queue, Event *block_on, Event *completion)
{
  if (completion)
  {
    completion->release();
  }

  if (!isValid() && !src)
  {
    return 0u;
  }

  if (!buffer_element_size || element_size == buffer_element_size)
  {
    // Matching element size. Use normal write.
    return write(src, element_size * element_count, offset_elements * element_size, queue, block_on, completion);
  }

  const size_t byte_write_offset = offset_elements * buffer_element_size;
  if (byte_write_offset >= size())
  {
    return 0u;
  }

  // Element size mismatch. Use piecewise write.
  const size_t copy_size = std::min(element_size, buffer_element_size);
  const size_t copy_element_count = std::min(element_count, (size() - byte_write_offset) / buffer_element_size);
  const size_t clear_byte_count = (buffer_element_size > element_size) ? buffer_element_size - element_size : 0u;
  // const_cast because OpenCL API does not support const.
  const auto *src_mem = static_cast<const uint8_t *>(src);

  if (copy_element_count)
  {
#if USE_PINNED
    // Mapped memory only when not using an explicit queue as queue implies a non-blocking call.
    if (!queue && (imp_->flags & kBfHostAccess))
    {
      if (block_on && block_on->isValid())
      {
        block_on->wait();
      }

      if (uint8_t *pinned_ptr = gputil::pin(*imp_, kPinWrite))
      {
        uint8_t *dst_mem = pinned_ptr;
        dst_mem += byte_write_offset;
        for (size_t i = 0; i < copy_element_count; ++i)
        {
          memcpy(dst_mem, src_mem, copy_size);
          // Clear any extra bytes.
          if (clear_byte_count)
          {
            memset(dst_mem + element_size, 0, clear_byte_count);
          }
          dst_mem += buffer_element_size;
          src_mem += element_size;
        }

        gputil::unpin(*imp_, pinned_ptr, queue);
        return copy_element_count;
      }
    }
#endif  // USE_PINNED

    size_t buffer_offset = byte_write_offset;
    cl_int clerr = CL_SUCCESS;
    cl_int clerr2 = CL_SUCCESS;
    cl_command_queue queue_cl = selectQueue(imp_->device, queue);
    const int block_on_count = (block_on && block_on->isValid()) ? 1 : 0;
    cl_event block_on_ocl = (block_on_count) ? block_on->detail()->event : nullptr;

    // Unfortunately I can't find whether the copy order is guaranteed for clEnqueueWriteBuffer(). To support the
    // completeion event, we'll set barrier after the copy instructions and wait on that.
    const cl_bool synchronous = (!queue || queue->internal()->force_synchronous) ? CL_TRUE : CL_FALSE;
    for (size_t i = 0; i < copy_element_count; ++i)
    {
      clerr2 = clEnqueueWriteBuffer(queue_cl, imp_->buffer(), synchronous, buffer_offset, copy_size, src_mem,
                                    block_on_count, (block_on_count) ? &block_on_ocl : nullptr, nullptr);
      src_mem += element_size;
      buffer_offset += buffer_element_size;

      clerr = (clerr) ? clerr : clerr2;
    }

    if (queue && completion)
    {
      // Set a barrier to ensure the copy commands complete and wait on that event.
      clEnqueueBarrierWithWaitList(queue_cl, 0, nullptr, &completion->detail()->event);
    }

    // Blocking operation when no explicit queue has been provided.
    if (!queue)
    {
      clerr2 = clFinish(queue_cl);
      clerr = (clerr) ? clerr : clerr2;
    }

    GPUAPICHECK(clerr, CL_SUCCESS, 0);
  }

  return copy_element_count;
}


void *Buffer::argPtr() const
{
  if (imp_)
  {
    return imp_->buffer();
  }

  return nullptr;
}


void *Buffer::address() const
{
  return argPtr();
}


size_t copyBuffer(Buffer &dst, const Buffer &src, Queue *queue, Event *block_on, Event *completion)
{
  return copyBuffer(dst, 0, src, 0, src.size(), queue, block_on, completion);
}


size_t copyBuffer(Buffer &dst, const Buffer &src, size_t byte_count, Queue *queue, Event *block_on, Event *completion)
{
  return copyBuffer(dst, 0, src, 0, byte_count, queue, block_on, completion);
}


size_t copyBuffer(Buffer &dst, size_t dst_offset, const Buffer &src, size_t src_offset, size_t byte_count, Queue *queue,
                  Event *block_on, Event *completion)
{
  const size_t dst_size = dst.size();
  const size_t src_size = src.size();

  if (completion)
  {
    completion->release();
  }

  // Check offsets.
  if (dst_size < dst_offset)
  {
    return 0;
  }

  if (src_size < src_offset)
  {
    return 0;
  }

  // Check sizes after offset.
  byte_count = std::min(byte_count, dst_size - dst_offset);
  byte_count = std::min(byte_count, src_size - src_offset);

  // Disable linting for auto on the following - it requires `auto *` because cl_mem is a pointer typedef, but this
  // is somewhat misleading because cl_mem doesn't look like a pointer type.
  cl_mem src_mem_cl = src.arg<cl_mem>();  // NOLINT(modernize-use-auto)
  cl_mem dst_mem_cl = dst.arg<cl_mem>();  // NOLINT(modernize-use-auto)

  cl_int clerr = CL_SUCCESS;
  cl_command_queue queue_cl = selectQueue(src.detail()->device, queue);
  const int block_on_count = (block_on && block_on->isValid()) ? 1 : 0;
  cl_event block_on_ocl = (block_on_count) ? block_on->detail()->event : nullptr;

  clerr = clEnqueueCopyBuffer(queue_cl, src_mem_cl, dst_mem_cl, src_offset, dst_offset, byte_count, block_on_count,
                              (block_on_count) ? &block_on_ocl : nullptr,
                              (queue && completion) ? &completion->detail()->event : nullptr);

  GPUAPICHECK(clerr, CL_SUCCESS, 0);

  // Block if no explicit queue provided.
  if (!queue || queue->internal()->force_synchronous)
  {
    clerr = clFinish(queue_cl);
    GPUAPICHECK(clerr, CL_SUCCESS, 0u);
  }

  return byte_count;
}
}  // namespace gputil
