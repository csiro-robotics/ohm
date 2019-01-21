#include "PointsDoubleBuffer.h"

#include <cassert>
#include <cstring>

namespace ohm
{
  template <typename T>
  PointsDoubleBufferT<T>::PointsDoubleBufferT()
    : write_buffer_index_(0)
  {}


  template <typename T>
  PointsDoubleBufferT<T>::~PointsDoubleBufferT()
  {}


  template <typename T>
  void PointsDoubleBufferT<T>::switchBuffers()
  {
    ScopedSpinLock guard(lock_);
    write_buffer_index_ = readBufferIndex();
    guard.unlock();
    buffers_[writeBufferIndex()].clear();
  }


  template <typename T>
  void PointsDoubleBufferT<T>::copyPoints(const T *points, size_t count)
  {
    PointsArray &buffer = buffers_[writeBufferIndex()];
    buffer.reserve(util::ceilPowerOf2(count));
    buffer.resize(count);
    if (count)
    {
      memcpy(buffer.data(), points, count * sizeof(*points));
    }
  }


  template <typename T>
  void PointsDoubleBufferT<T>::presizeWritePoints(size_t target_points)
  {
    PointsArray &buffer = buffers_[writeBufferIndex()];
    buffer.reserve(util::ceilPowerOf2(target_points));
    buffer.resize(target_points);
  }


  template <typename T>
  void PointsDoubleBufferT<T>::writePoints(const T *points, size_t count, size_t offset)
  {
    PointsArray &buffer = buffers_[writeBufferIndex()];
    if (count)
    {
      assert(count + offset <= buffer.size());
      memcpy(buffer.data() + offset, points, sizeof(*points) * count);
    }
  }


  template <typename T>
  void PointsDoubleBufferT<T>::append(const T &point)
  {
    PointsArray &buffer = buffers_[writeBufferIndex()];
    buffer.push_back(point);
  }


  template <typename T>
  void PointsDoubleBufferT<T>::append(const T *points, size_t count)
  {
    PointsArray &buffer = buffers_[writeBufferIndex()];
    for (size_t i = 0; i < count; ++i)
    {
      buffer.push_back(points[i]);
    }
  }


  template <typename T>
  void PointsDoubleBufferT<T>::clear()
  {
    PointsArray &buffer = buffers_[writeBufferIndex()];
    buffer.clear();
    switchBuffers();
  }


  template <typename T>
  size_t PointsDoubleBufferT<T>::readLastNPoints(T *dst, size_t point_count) const
  {
    ScopedSpinLock guard(lock_);
    const PointsArray &buffer = buffers_[readBufferIndex()];
    point_count = std::min(point_count, buffer.size());
    if (point_count)
    {
      size_t read_offset = buffer.size() - point_count;
      memcpy(dst, buffer.data() + read_offset, point_count * sizeof(*dst));
    }
    return point_count;
  }


  template <typename T>
  size_t PointsDoubleBufferT<T>::readPoints(T *dst, size_t max_points) const
  {
    ScopedSpinLock guard(lock_);
    const PointsArray &buffer = buffers_[readBufferIndex()];
    if (!buffer.empty())
    {
      size_t count = std::min(buffer.size(), max_points);
      memcpy(dst, buffer.data(), count * sizeof(*dst));
      return count;
    }
    return 0;
  }


  template <typename T>
  size_t PointsDoubleBufferT<T>::readPoints(PointsArray &points) const
  {
    ScopedSpinLock guard(lock_);
    const PointsArray &buffer = buffers_[readBufferIndex()];
    points.reserve(util::ceilPowerOf2(buffer.size()));
    points.resize(buffer.size());
    if (!points.empty())
    {
      memcpy(points.data(), buffer.data(), buffer.size() * sizeof(*points.data()));
    }
    return points.size();
  }
}  // namespace ohm
