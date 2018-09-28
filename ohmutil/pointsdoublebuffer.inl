#include "pointsdoublebuffer.h"

#include <cassert>
#include <cstring>

template <typename T>
PointsDoubleBufferT<T>::PointsDoubleBufferT()
  : _writeBufferIndex(0)
{
}


template <typename T>
PointsDoubleBufferT<T>::~PointsDoubleBufferT()
{
}


template <typename T>
void PointsDoubleBufferT<T>::switchBuffers()
{
  ScopedSpinLock guard(&_lock);
  _writeBufferIndex = readBufferIndex();
  guard.unlock();
  _buffers[writeBufferIndex()].clear();
}


template <typename T>
void PointsDoubleBufferT<T>::copyPoints(const T *points, size_t count)
{
  PointsArray &buffer = _buffers[writeBufferIndex()];
  buffer.reserve(util::ceilPowerOf2(count));
  buffer.resize(count);
  if (count)
  {
    memcpy(buffer.data(), points, count * sizeof(*points));
  }
}


template <typename T>
void PointsDoubleBufferT<T>::presizeWritePoints(size_t targetPoints)
{
  PointsArray &buffer = _buffers[writeBufferIndex()];
  buffer.reserve(util::ceilPowerOf2(targetPoints));
  buffer.resize(targetPoints);
}


template <typename T>
void PointsDoubleBufferT<T>::writePoints(const T *points, size_t count, size_t offset)
{
  PointsArray &buffer = _buffers[writeBufferIndex()];
  if (count)
  {
    assert(count + offset <= buffer.size());
    memcpy(buffer.data() + offset, points, sizeof(*points) * count);
  }
}


template <typename T>
void PointsDoubleBufferT<T>::append(const T &point)
{
  PointsArray &buffer = _buffers[writeBufferIndex()];
  buffer.push_back(point);
}


template <typename T>
void PointsDoubleBufferT<T>::append(const T *points, size_t count)
{
  PointsArray &buffer = _buffers[writeBufferIndex()];
  for (size_t i = 0; i < count; ++i)
  {
    buffer.push_back(points[i]);
  }
}


template <typename T>
void PointsDoubleBufferT<T>::clear()
{
  PointsArray &buffer = _buffers[writeBufferIndex()];
  buffer.clear();
  switchBuffers();
}


template <typename T>
size_t PointsDoubleBufferT<T>::readLastNPoints(T *dst, size_t pointCount) const
{
  ScopedSpinLock guard(&_lock);
  const PointsArray &buffer = _buffers[readBufferIndex()];
  pointCount = std::min(pointCount, buffer.size());
  if (pointCount)
  {
    size_t readOffset = buffer.size() - pointCount;
    memcpy(dst, buffer.data() + readOffset, pointCount * sizeof(*dst));
  }
  return pointCount;
}


template <typename T>
size_t PointsDoubleBufferT<T>::readPoints(T *dst, size_t maxPoints) const
{
  ScopedSpinLock guard(&_lock);
  const PointsArray &buffer = _buffers[readBufferIndex()];
  if (!buffer.empty())
  {
    size_t count = std::min(buffer.size(), maxPoints);
    memcpy(dst, buffer.data(), count * sizeof(*dst));
    return count;
  }
  return 0;
}


template <typename T>
size_t PointsDoubleBufferT<T>::readPoints(PointsArray &points) const
{
  ScopedSpinLock guard(&_lock);
  const PointsArray &buffer = _buffers[readBufferIndex()];
  points.reserve(util::ceilPowerOf2(buffer.size()));
  points.resize(buffer.size());
  if (!points.empty())
  {
    memcpy(points.data(), buffer.data(), buffer.size() * sizeof(*points.data()));
  }
  return points.size();
}
