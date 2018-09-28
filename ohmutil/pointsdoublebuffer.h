#ifndef POINTSDOUBLEBUFFER_H_
#define POINTSDOUBLEBUFFER_H_

//#include "ohmutilexport.h"

#include "spinlock.h"

#include <glm/glm.hpp>

#include <vector>

/// A thread safe point cloud data buffer.
///
/// Maintains separate read/write buffers and switches once writing is
/// complete.
template <typename T>
class PointsDoubleBufferT
{
public:
  typedef std::vector<T> PointsArray;

  PointsDoubleBufferT();
  ~PointsDoubleBufferT();

  void switchBuffers();

  /// Copy the given set of points into the write buffer.
  /// The target buffer is cleared first, and only holds @p points after calling.
  ///
  /// Call @c switchBuffers() after to expose these points to read calls.
  /// @param points Array of points to copy from.
  /// @param count Number of points in @p points.
  void copyPoints(const T *points, size_t count);

  /// Resize the current write buffer to cater for the given number of points.
  /// Use @c writePoints() to write into the reserved buffer.
  /// @param targetPoints The number of points to resize to.
  void presizeWritePoints(size_t targetPoints);
  /// Write into the pre-sized write buffer. The buffer must have already been correctly
  /// sized with @p presizeWritePoints().
  ///
  /// Call @c switchBuffers() after to expose these points to read calls.
  /// @param points The points array to write from.
  /// @param count The number of points to write from @p points.
  /// @param offset The index offset into the write buffer to start writing @p points at.
  void writePoints(const T *points, size_t count, size_t offset = 0);

  inline void writePoint(const T &point, size_t offset) { writePoints(&point, 1, offset); }

  /// Append a point to the current write buffer. Does not need presizing.
  void append(const T &point);

  /// Append a point to the current write buffer. Does not need presizing.
  void append(const T *point, size_t count);

  /// Clears the write buffer then switches buffers.
  void clear();

  /// Read @p pointCount number of points from the end of the read buffer.
  size_t readLastNPoints(T *dst, size_t pointCount) const;
  size_t readPoints(T *dst, size_t maxPoints) const;
  size_t readPoints(PointsArray &points) const;

private:
  inline int readBufferIndex() const { return 1 - _writeBufferIndex; }
  inline int writeBufferIndex() const { return _writeBufferIndex; }

  mutable SpinLock _lock;
  PointsArray _buffers[2];
  int _writeBufferIndex;
};

#include "pointsdoublebuffer.inl"

#endif // POINTSDOUBLEBUFFER_H_
