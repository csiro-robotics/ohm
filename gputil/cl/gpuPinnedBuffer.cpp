// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuPinnedBuffer.h"

#include "gpuBuffer.h"
#include "gpuBufferDetail.h"

#include <algorithm>

using namespace gputil;


PinnedBuffer::PinnedBuffer()
  : buffer_(nullptr)
  , pinned_(nullptr)
  , mode_(kPinRead)
{}


PinnedBuffer::PinnedBuffer(Buffer &buffer, PinMode mode)
  : buffer_(&buffer)
  , pinned_(nullptr)
  , mode_(mode)
{
  pin();
}


PinnedBuffer::PinnedBuffer(PinnedBuffer &&other) noexcept
  : buffer_(other.buffer_)
  , pinned_(other.pinned_)
  , mode_(other.mode_)
{
  other.buffer_ = nullptr;
  other.pinned_ = nullptr;
}


PinnedBuffer::~PinnedBuffer()
{
  unpin();
}


bool PinnedBuffer::isPinned() const
{
  return pinned_ != nullptr;
}


void PinnedBuffer::pin()
{
  if (!pinned_ && buffer_)
  {
    pinned_ = ::pin(*buffer_->detail(), mode_);
  }
}


void PinnedBuffer::unpin(Queue *queue, Event *block_on, Event *completion)
{
  if (pinned_ && buffer_)
  {
    ::unpin(*buffer_->detail(), pinned_, queue, block_on, completion);
    pinned_ = nullptr;
  }
}


size_t PinnedBuffer::read(void *dst, size_t byte_count, size_t src_offset) const
{
  if (buffer_)
  {
    if (pinned_)
    {
      const uint8_t *src_mem = static_cast<const uint8_t *>(pinned_);
      uint8_t *dst_mem = static_cast<uint8_t *>(dst);

      byte_count = std::min(byte_count, buffer_->size() - src_offset);
      src_mem += src_offset;
      memcpy(dst_mem, src_mem, byte_count);
      return byte_count;
    }

    return buffer_->read(dst, byte_count, src_offset);
  }

  return 0u;
}


size_t PinnedBuffer::write(const void *src, size_t byte_count, size_t dst_offset)
{
  if (buffer_)
  {
    if (pinned_)
    {
      const uint8_t *src_mem = static_cast<const uint8_t *>(src);
      uint8_t *dst_mem = static_cast<uint8_t *>(pinned_);

      byte_count = std::min(byte_count, buffer_->size() - dst_offset);
      dst_mem += dst_offset;
      memcpy(dst_mem, src_mem, byte_count);
      return byte_count;
    }

    return buffer_->write(src, byte_count, dst_offset);
  }

  return 0u;
}


size_t PinnedBuffer::readElements(void *dst, size_t element_size, size_t element_count, size_t offset_elements,
                                  size_t buffer_element_size)
{
  if (!pinned_)
  {
    return buffer_->readElements(dst, element_size, element_count, offset_elements, buffer_element_size);
  }

  const uint8_t *src_mem = static_cast<const uint8_t *>(pinned_);
  uint8_t *dst_mem = static_cast<uint8_t *>(dst);
  if (element_size == buffer_element_size || buffer_element_size == 0)
  {
    const size_t src_offset = offset_elements * element_size;

    if (src_offset + element_size >= buffer_->size())
    {
      // Can't even copy one element from the requested offset.
      return 0;
    }

    const size_t byte_count = std::min(element_count * element_size, buffer_->size() - src_offset);
    memcpy(dst_mem, src_mem + src_offset, byte_count);
    return byte_count / element_count;
  }

  // Size-mismatch. Iterative copy.
  const uint8_t *src_end = src_mem + buffer_->size();
  const size_t element_copy_size = std::min(element_size, buffer_element_size);
  src_mem += offset_elements * buffer_element_size;
  size_t copy_count = 0;
  for (size_t i = 0; i < element_count && src_mem < src_end; ++i)
  {
    memcpy(dst_mem, src_mem, element_copy_size);
    src_mem += buffer_element_size;
    dst_mem += element_size;
    ++copy_count;
  }

  return copy_count;
}

size_t PinnedBuffer::writeElements(const void *src, size_t element_size, size_t element_count, size_t offset_elements,
                                   size_t buffer_element_size)
{
  if (!pinned_)
  {
    return buffer_->writeElements(src, element_size, element_count, offset_elements, buffer_element_size);
  }

  uint8_t *dst_mem = static_cast<uint8_t *>(pinned_);
  const uint8_t *src_mem = static_cast<const uint8_t *>(src);
  if (element_size == buffer_element_size || buffer_element_size == 0)
  {
    const size_t dst_offset = offset_elements * element_size;

    if (dst_offset + element_size >= buffer_->size())
    {
      // Can't even copy one element from to requested offset.
      return 0;
    }

    const size_t byte_count = std::min(element_count * element_size, buffer_->size() - dst_offset);
    memcpy(dst_mem + dst_offset, src_mem, byte_count);
    return byte_count / element_count;
  }

  // Size-mismatch. Iterative copy.
  const uint8_t *dst_end = dst_mem + buffer_->size();
  const size_t element_copy_size = std::min(element_size, buffer_element_size);
  dst_mem += offset_elements * buffer_element_size;
  size_t copy_count = 0;
  for (size_t i = 0; i < element_count && dst_mem < dst_end; ++i)
  {
    memcpy(dst_mem, src_mem, element_copy_size);
    dst_mem += buffer_element_size;
    src_mem += element_size;
    ++copy_count;
  }

  return copy_count;
}


PinnedBuffer &PinnedBuffer::operator=(PinnedBuffer &&other) noexcept
{
  unpin();
  buffer_ = other.buffer_;
  pinned_ = other.pinned_;
  mode_ = other.mode_;
  other.buffer_ = nullptr;
  other.pinned_ = nullptr;
  return *this;
}
