#ifndef OHMUTIL_POINTSDOUBLEBUFFER_H
#define OHMUTIL_POINTSDOUBLEBUFFER_H

//#include "OhmUtilExport.h"

#include "SpinLock.h"

#include <vector>

namespace ohm
{
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
    /// @param target_points The number of points to resize to.
    void presizeWritePoints(size_t target_points);
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
    size_t readLastNPoints(T *dst, size_t point_count) const;
    size_t readPoints(T *dst, size_t max_points) const;
    size_t readPoints(PointsArray &points) const;

  private:
    inline int readBufferIndex() const { return 1 - write_buffer_index_; }
    inline int writeBufferIndex() const { return write_buffer_index_; }

    mutable SpinLock lock_;
    PointsArray buffers_[2];
    int write_buffer_index_;
  };
}  // namespace ohm

#include "PointsDoubleBuffer.inl"

#endif  // OHMUTIL_POINTSDOUBLEBUFFER_H
