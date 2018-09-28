// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpupinnedbuffer.h"

#include "gpubuffer.h"
#include "gpubufferdetail.h"

#include <algorithm>

using namespace gputil;

PinnedBuffer::PinnedBuffer(Buffer &buffer, PinMode mode)
  : _buffer(&buffer)
  , _pinned(nullptr)
  , _mode(mode)
{
  pin();
}


PinnedBuffer::PinnedBuffer(PinnedBuffer &&other)
  : _buffer(other._buffer)
  , _pinned(other._pinned)
  , _mode(other._mode)
{
  other._buffer = nullptr;
  other._pinned = nullptr;
}


PinnedBuffer::~PinnedBuffer()
{
  unpin();
}


bool PinnedBuffer::isPinned() const
{
  return _pinned != nullptr;
}


void PinnedBuffer::pin()
{
  if (!_pinned && _buffer)
  {
    BufferDetail *imp = _buffer->detail();
    _pinned = ::pin(*_buffer->detail(), _mode);
  }
}


void PinnedBuffer::unpin(Queue *queue, Event *blockOn, Event *completion)
{
  if (_pinned && _buffer)
  {
    ::unpin(*_buffer->detail(), _pinned, queue, blockOn, completion);
    _pinned = nullptr;
  }
}


size_t PinnedBuffer::read(void *dst, size_t byteCount, size_t srcOffset) const
{
  if (_buffer)
  {
    if (_pinned)
    {
      const uint8_t *srcMem = static_cast<const uint8_t *>(_pinned);
      uint8_t *dstMem = static_cast<uint8_t *>(dst);

      byteCount = std::min(byteCount, _buffer->size() - srcOffset);
      srcMem += srcOffset;
      memcpy(dstMem, srcMem, byteCount);
      return byteCount;
    }

    return _buffer->read(dst, byteCount, srcOffset);
  }

  return 0u;
}


size_t PinnedBuffer::write(const void *src, size_t byteCount, size_t dstOffset)
{
  if (_buffer)
  {
    if (_pinned)
    {
      const uint8_t *srcMem = static_cast<const uint8_t *>(src);
      uint8_t *dstMem = static_cast<uint8_t *>(_pinned);

      byteCount = std::min(byteCount, _buffer->size() - dstOffset);
      dstMem += dstOffset;
      memcpy(dstMem, srcMem, byteCount);
      return byteCount;
    }

    return _buffer->write(src, byteCount, dstOffset);
  }

  return 0u;
}


PinnedBuffer &PinnedBuffer::operator=(PinnedBuffer &&other)
{
  unpin();
  _buffer =other._buffer;
  _pinned = other._pinned;
  _mode = other._mode;
  other._buffer = nullptr;
  other._pinned = nullptr;
  return *this;
}
