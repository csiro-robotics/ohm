// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuPinnedBuffer.h"

#include "cuda/gpuBufferDetail.h"
#include "gpuApiException.h"
#include "gpuBuffer.h"
#include "gpuThrow.h"

#include <cuda.h>
#include <cuda_runtime.h>

using namespace gputil;

PinnedBuffer::PinnedBuffer(Buffer &buffer, PinMode mode)
  : buffer_(&buffer)
  , pinned_(nullptr)
  , mode_(mode)
{
  pin();
}


PinnedBuffer::PinnedBuffer(PinnedBuffer &&other)
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
  if (buffer_ && !pinned_)
  {
    BufferDetail *imp = buffer_->detail();
    pinned_ = gputil::pin(*imp, mode_);
  }
}


void PinnedBuffer::unpin(Queue *queue, Event *block_on, Event *completion)
{
  if (buffer_ && pinned_)
  {
    gputil::unpin(*buffer_->detail(), pinned_, mode_);
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
      cudaError_t err = cudaSuccess;

      byte_count = std::min(byte_count, buffer_->size() - src_offset);
      src_mem += src_offset;
      err = cudaMemcpy(dst_mem, src_mem, byte_count, cudaMemcpyHostToHost);
      GPUAPICHECK(err, cudaSuccess, 0);
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
      cudaError_t err = cudaSuccess;

      byte_count = std::min(byte_count, buffer_->size() - dst_offset);
      dst_mem += dst_offset;
      err = cudaMemcpy(dst_mem, src_mem, byte_count, cudaMemcpyHostToHost);
      GPUAPICHECK(err, cudaSuccess, 0u);
      return byte_count;
    }

    return buffer_->write(src, byte_count, dst_offset);
  }

  return 0u;
}


PinnedBuffer &PinnedBuffer::operator=(PinnedBuffer &&other)
{
  unpin();
  buffer_ = other.buffer_;
  pinned_ = other.pinned_;
  mode_ = other.mode_;
  other.buffer_ = nullptr;
  other.pinned_ = nullptr;
  return *this;
}
