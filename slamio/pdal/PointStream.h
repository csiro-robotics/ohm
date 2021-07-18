// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_PDAL_POINTSTREAM_H_
#define SLAMIO_PDAL_POINTSTREAM_H_

#include "slamio/SlamIOConfig.h"

#include "slamio/DataChannel.h"
#include "slamio/Points.h"

#include <pdal/Dimension.hpp>
#include <pdal/PointTable.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>

namespace slamio
{
class PointStream : public pdal::StreamPointTable
{
public:
  explicit PointStream(size_t buffer_capacity, DataChannel require = DataChannel::Position | DataChannel::Time);

  /// Called when execute() is started.  Typically used to set buffer size
  /// when all dimensions are known.
  void finalize() override;

  bool nextPoint(CloudPoint &sample);

  inline bool isValid() const { return valid_dimensions_; }

  /// True once we have data available for reading.
  inline bool haveData() const { return have_data_; }

  /// True once loading has been completed (@c markLoadComplete() called) and the read index is at the end of the
  /// read buffer.
  inline bool done()
  {
    if (loading_complete_)
    {
      std::unique_lock<std::mutex> guard(buffer_mutex_);
      return next_read_ >= buffers_[1 - write_index_].size();
    }
    return false;
  }

  /// Mark for abort. No more data points are stored.
  inline void abort()
  {
    abort_ = true;
    flip_wait_.notify_all();
  }

  /// Mark loading as done: only to be called from the loading thread.
  inline void markLoadComplete() { loading_complete_ = true; }

  inline bool hasTimestamp() const { return (available_channels_ & DataChannel::Time) != DataChannel::None; }
  inline bool hasNormals() const { return (available_channels_ & DataChannel::Normals) != DataChannel::None; }
  inline bool hasColour() const { return (available_channels_ & DataChannel::Colour) != DataChannel::None; }
  inline bool hasIntensity() const { return (available_channels_ & DataChannel::Intensity) != DataChannel::None; }

protected:
  // Not supported
  inline char *getPoint(pdal::PointId /* idx */) override { return nullptr; }

  void setFieldInternal(pdal::Dimension::Id dim, pdal::PointId idx, const void *val) override;

  /// Called whenever the buffer capacity is filled before starting on the next block.
  void reset() override;

private:
  // Double buffer to allow background thread streaming.
  // Use w channel for timetstamp
  std::array<std::vector<CloudPoint>, 2> buffers_;
  std::array<pdal::Dimension::Type, 3> position_channel_types_{};
  std::array<pdal::Dimension::Type, 3> normal_channel_types_{};
  std::array<pdal::Dimension::Type, 3> colour_channel_types_{};
  pdal::Dimension::Id time_dimension_{ pdal::Dimension::Id::Unknown };
  pdal::Dimension::Type time_channel_type_{};
  pdal::Dimension::Type intensity_channel_type_{};
  std::atomic_uint next_read_{ 0 };
  std::atomic_int write_index_{ 0 };
  pdal::PointLayout layout_;
  std::mutex buffer_mutex_;
  std::condition_variable flip_wait_;
  std::atomic_bool have_data_{ false };
  std::atomic_bool loading_complete_{ false };
  std::atomic_bool abort_{ false };
  std::atomic_bool valid_dimensions_{ false };
  DataChannel available_channels_ = DataChannel::None;
  DataChannel required_channels_ = DataChannel::Position | DataChannel::Time;
};
}  // namespace slamio

#endif  // SLAMIO_PDAL_POINTSTREAM_H_
