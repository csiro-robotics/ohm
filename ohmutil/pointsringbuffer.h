#ifndef POINTSRINGBUFFER_H_
#define POINTSRINGBUFFER_H_

//#include "ohmutilexport.h"

#include "spinlock.h"

#include <glm/glm.hpp>

#include <vector>

/// A thread safe point cloud data buffer.
///
/// Maintains seperate read/write buffers and switches once writing is
/// complete.
template <typename T>
class PointsRingBufferT
{
public:
  typedef std::vector<T> PointsArray;

  PointsRingBufferT(size_t initialCapacity = 0);
  ~PointsRingBufferT();

  void clear();
  void reserve(size_t size);

  inline size_t capacity() const { return _buffer.size(); }

  /// Write into the pre-sized write buffer. The buffer must have already been correctly
  /// sized with @p presizeWritePoints().
  ///
  /// Call @c switchBuffers() after to expose these points to read calls.
  /// @param points The points array to write from.
  /// @param count The number of points to write from @p points.
  /// @param offset The index offset into the write buffer to start writing @p points at.
  void writePoints(const T *points, size_t count);

  /// Append a point to the ring buffer.
  /// @param point The point to append.
  /// @return The number of points in the buffer after the call.
  size_t append(const T &point);

  /// Updates the most recent point in the ring buffer, or adds a new one if empty.
  /// @param point The new point data.
  /// @return The number of points in the buffer after the call.
  size_t updateLast(const T &point);

  /// Read @p pointCount number of points from the end of the read buffer.
  size_t readPoints(T *dst, size_t maxPoints) const;
  size_t readPoints(PointsArray &points) const;

private:

  mutable SpinLock _lock;
  PointsArray _buffer;
  size_t _writeCursor;
  size_t _count;
};

/// A single precision implementation of the @c PointsRingBufferT&lt;T&gt;.
class PointsRingBuffer : public PointsRingBufferT<float>
{
public:
  inline PointsRingBuffer(size_t initialCapacity = 0) : PointsRingBufferT<float>(initialCapacity) {}
};

/// A single precision implementation of the @c PointsRingBufferT&lt;T&gt;.
class PointsRingBufferD : public PointsRingBufferT<double>
{
public:
  inline PointsRingBufferD(size_t initialCapacity = 0) : PointsRingBufferT<double>(initialCapacity) {}
};

#include "pointsringbuffer.inl"

#endif // POINTSRINGBUFFER_H_
