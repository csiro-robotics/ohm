#ifndef OHMUTIL_POINTSRINGBUFFER_H
#define OHMUTIL_POINTSRINGBUFFER_H

//#include "OhmUtilExport.h"

#include "SpinLock.h"

#include <glm/glm.hpp>

#include <vector>

// Deprecated - don't document
#ifndef DOXYGEN_SHOULD_SKIP_THIS

namespace ohm
{
/// A thread safe point cloud data buffer.
///
/// Maintains seperate read/write buffers and switches once writing is
/// complete.
template <typename T>
class PointsRingBufferT
{
public:
  typedef std::vector<T> PointsArray;

  PointsRingBufferT(size_t initial_capacity = 0);
  ~PointsRingBufferT();

  void clear();
  void reserve(size_t size);

  inline size_t capacity() const { return buffer_.size(); }

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
  size_t readPoints(T *dst, size_t max_points) const;
  size_t readPoints(PointsArray &points) const;

private:
  mutable SpinLock lock_;
  PointsArray buffer_;
  size_t write_cursor_;
  size_t count_;
};

/// A single precision implementation of the @c PointsRingBufferT&lt;T&gt;.
class PointsRingBuffer : public PointsRingBufferT<float>
{
public:
  inline PointsRingBuffer(size_t initial_capacity = 0)
    : PointsRingBufferT<float>(initial_capacity)
  {}
};

/// A single precision implementation of the @c PointsRingBufferT&lt;T&gt;.
class PointsRingBufferD : public PointsRingBufferT<double>
{
public:
  inline PointsRingBufferD(size_t initial_capacity = 0)
    : PointsRingBufferT<double>(initial_capacity)
  {}
};

}  // namespace ohm

#include "PointsRingBuffer.inl"

#endif  // DOXYGEN_SHOULD_SKIP_THIS

#endif  // OHMUTIL_POINTSRINGBUFFER_H
