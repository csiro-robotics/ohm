#include "PointsRingBuffer.h"

#include <cassert>
#include <cstring>

template <typename T>
PointsRingBufferT<T>::PointsRingBufferT(size_t initial_capacity)
  : buffer_(initial_capacity)
  , write_cursor_(0)
  , count_(0)
{
}


template <typename T>
PointsRingBufferT<T>::~PointsRingBufferT()
{
}


template <typename T>
void PointsRingBufferT<T>::clear()
{
  ScopedSpinLock guard(lock_);
  write_cursor_ = count_ = 0;
}


template <typename T>
void PointsRingBufferT<T>::reserve(size_t size)
{
  ScopedSpinLock guard(lock_);
  buffer_.resize(size);
  if (size < count_)
  {
    // This is likely to disrupt the buffer continuity.
    count_ = size;
    write_cursor_ = 0;
  }
}


template <typename T>
void PointsRingBufferT<T>::writePoints(const T *points, size_t count)
{
  ScopedSpinLock guard(lock_);
  size_t src_offset = (count <= buffer_.size()) ? 0 : count - buffer_.size();
  // First write is from the current cursor to the end of the buffer.
  // Ignore write cursor when count exceeds the buffer capacity.
  size_t write_index0 = (src_offset == 0) ? write_cursor_ : 0;
  size_t write_count0 = (src_offset == 0) ? std::min(count, buffer_.size() - write_index0) : count - src_offset;
  // Second write covers the remaining data. Don't use when not overflowing the end
  // or when new data exceeds the capacity (no need in the latter case).
  size_t write_count1 = (src_offset == 0 && write_count0 < count) ? count - write_count0 : 0;

  memcpy(buffer_.data() + write_index0, points + src_offset, write_count0 * sizeof(*points));
  write_cursor_ = write_index0 + write_count0;
  if (write_count1)
  {
    memcpy(buffer_.data(), points + src_offset + write_count0, write_count1 * sizeof(*points));
    write_cursor_ = write_count1;
  }

  count_ = std::min(buffer_.size(), count_ + write_count0 + write_count1);
}


template <typename T>
size_t PointsRingBufferT<T>::append(const T &point)
{
  ScopedSpinLock guard(lock_);
  if (!buffer_.empty())
  {
    buffer_[write_cursor_] = point;
    write_cursor_ = (write_cursor_ + 1) % buffer_.size();
    count_ = std::min(count_ + 1, buffer_.size());
  }
  return count_;
}


template <typename T>
size_t PointsRingBufferT<T>::updateLast(const T &point)
{
  ScopedSpinLock guard(lock_);
  if (!buffer_.empty())
  {
    size_t write_cursor = write_cursor_;
    if (count_ > 0)
    {
      // Last entry overwrite. Modify the write cursor to the previous entry.
      // _count and _writeCursor do not change.
      write_cursor = (write_cursor > 0) ? write_cursor - 1 : buffer_.size() - 1;
    }
    else
    {
      // Add a new entry.
      write_cursor_ = (write_cursor_ + 1) % buffer_.size();
      count_ = std::min(count_ + 1, buffer_.size());
    }
    buffer_[write_cursor] = point;
  }
  return count_;
}


template <typename T>
size_t PointsRingBufferT<T>::readPoints(T *dst, size_t max_points) const
{
  ScopedSpinLock guard(lock_);
  size_t count = std::min(count_, max_points);
  if (count)
  {
    memcpy(dst, buffer_.data(), count * sizeof(*dst));
    return count;
  }
  return 0;
}


template <typename T>
size_t PointsRingBufferT<T>::readPoints(PointsArray &points) const
{
  ScopedSpinLock guard(lock_);
  points.reserve(util::ceilPowerOf2(count_));
  points.resize(count_);
  if (!points.empty())
  {
    memcpy(points.data(), buffer_.data(), count_ * sizeof(*points.data()));
  }
  return points.size();
}
