#include "pointsringbuffer.h"

#include <cassert>
#include <cstring>

template <typename T>
PointsRingBufferT<T>::PointsRingBufferT(size_t initialCapacity)
  : _buffer(initialCapacity)
  , _writeCursor(0)
  , _count(0)
{
}


template <typename T>
PointsRingBufferT<T>::~PointsRingBufferT()
{
}


template <typename T>
void PointsRingBufferT<T>::clear()
{
  ScopedSpinLock guard(_lock);
  _writeCursor = _count = 0;
}


template <typename T>
void PointsRingBufferT<T>::reserve(size_t size)
{
  ScopedSpinLock guard(_lock);
  _buffer.resize(size);
  if (size < _count)
  {
    // This is likely to disrupt the buffer continuity.
    _count = size;
    _writeCursor = 0;
  }
}


template <typename T>
void PointsRingBufferT<T>::writePoints(const T *points, size_t count)
{
  ScopedSpinLock guard(_lock);
  size_t srcOffset = (count <= _buffer.size()) ? 0 : count - _buffer.size();
  // First write is from the current cursor to the end of the buffer.
  // Ignore write cursor when count exceeds the buffer capacity.
  size_t writeIndex0 = (srcOffset == 0) ? _writeCursor : 0;
  size_t writeCount0 = (srcOffset == 0) ? std::min(count, _buffer.size() - writeIndex0) : count - srcOffset;
  // Second write covers the remaining data. Don't use when not overflowing the end
  // or when new data exceeds the capacity (no need in the latter case).
  size_t writeCount1 = (srcOffset == 0 && writeCount0 < count) ? count - writeCount0 : 0;

  memcpy(_buffer.data() + writeIndex0, points + srcOffset, writeCount0 * sizeof(*points));
  _writeCursor = writeIndex0 + writeCount0;
  if (writeCount1)
  {
    memcpy(_buffer.data(), points + srcOffset + writeCount0, writeCount1 * sizeof(*points));
    _writeCursor = writeCount1;
  }

  _count = std::min(_buffer.size(), _count + writeCount0 + writeCount1);
}


template <typename T>
size_t PointsRingBufferT<T>::append(const T &point)
{
  ScopedSpinLock guard(_lock);
  if (!_buffer.empty())
  {
    _buffer[_writeCursor] = point;
    _writeCursor = (_writeCursor + 1) % _buffer.size();
    _count = std::min(_count + 1, _buffer.size());
  }
  return _count;
}


template <typename T>
size_t PointsRingBufferT<T>::updateLast(const T &point)
{
  ScopedSpinLock guard(_lock);
  if (!_buffer.empty())
  {
    size_t writeCursor = _writeCursor;
    if (_count > 0)
    {
      // Last entry overwrite. Modify the write cursor to the previous entry.
      // _count and _writeCursor do not change.
      writeCursor = (writeCursor > 0) ? writeCursor - 1 : _buffer.size() - 1;
    }
    else
    {
      // Add a new entry.
      _writeCursor = (_writeCursor + 1) % _buffer.size();
      _count = std::min(_count + 1, _buffer.size());
    }
    _buffer[writeCursor] = point;
  }
  return _count;
}


template <typename T>
size_t PointsRingBufferT<T>::readPoints(T *dst, size_t maxPoints) const
{
  ScopedSpinLock guard(_lock);
  size_t count = std::min(_count, maxPoints);
  if (count)
  {
    memcpy(dst, _buffer.data(), count * sizeof(*dst));
    return count;
  }
  return 0;
}


template <typename T>
size_t PointsRingBufferT<T>::readPoints(PointsArray &points) const
{
  ScopedSpinLock guard(_lock);
  points.reserve(util::ceilPowerOf2(_count));
  points.resize(_count);
  if (!points.empty())
  {
    memcpy(points.data(), _buffer.data(), _count * sizeof(*points.data()));
  }
  return points.size();
}
