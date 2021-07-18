// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointStream.h"

namespace slamio
{
PointStream::PointStream(size_t buffer_capacity, DataChannel require)
  : pdal::StreamPointTable(layout_, buffer_capacity)
  , required_channels_(require)
{
  // Register for the data we are interested in
  buffers_[0].reserve(buffer_capacity);
  buffers_[1].reserve(buffer_capacity);
  // Start with flipping allowed.
  flip_wait_.notify_one();
}

void PointStream::finalize()
{
  if (!layout_.finalized())
  {
    // Validate the dimensions.
    available_channels_ = DataChannel::None;

    bool have_dim = true;
    for (int i = 0; i < 3; ++i)
    {
      const auto *pos_dim = layout_.dimDetail(pdal::Dimension::Id(i + int(pdal::Dimension::Id::X)));
      if (pos_dim)
      {
        position_channel_types_[i] = pos_dim->type();
        have_dim = have_dim && (position_channel_types_[i] == pdal::Dimension::Type::Double ||
                                position_channel_types_[i] == pdal::Dimension::Type::Float);
      }
      else
      {
        have_dim = false;
      }
    }
    available_channels_ |= (have_dim) ? DataChannel::Position : DataChannel::None;

    have_dim = true;
    for (int i = 0; i < 3; ++i)
    {
      const auto *pos_dim = layout_.dimDetail(pdal::Dimension::Id(i + int(pdal::Dimension::Id::NormalX)));
      if (pos_dim)
      {
        normal_channel_types_[i] = pos_dim->type();
        have_dim = have_dim && (normal_channel_types_[i] == pdal::Dimension::Type::Double ||
                                normal_channel_types_[i] == pdal::Dimension::Type::Float);
      }
      else
      {
        have_dim = false;
      }
    }
    available_channels_ |= (have_dim) ? DataChannel::Normals : DataChannel::None;

    have_dim = true;
    for (int i = 0; i < 3; ++i)
    {
      const auto *colour_dim = layout_.dimDetail(pdal::Dimension::Id(i + int(pdal::Dimension::Id::Red)));
      if (colour_dim)
      {
        colour_channel_types_[i] = colour_dim->type();
        have_dim = have_dim && (colour_channel_types_[i] == pdal::Dimension::Type::Unsigned8 ||
                                colour_channel_types_[i] == pdal::Dimension::Type::Unsigned16 ||
                                colour_channel_types_[i] == pdal::Dimension::Type::Unsigned32);
      }
      else
      {
        have_dim = false;
      }
    }
    available_channels_ |= (have_dim) ? DataChannel::Colour : DataChannel::None;

    const auto *intensity_dim = layout_.dimDetail(pdal::Dimension::Id::Intensity);
    if (intensity_dim)
    {
      intensity_channel_type_ = intensity_dim->type();
      if (intensity_channel_type_ == pdal::Dimension::Type::Unsigned8 ||
          intensity_channel_type_ == pdal::Dimension::Type::Unsigned16 ||
          intensity_channel_type_ == pdal::Dimension::Type::Unsigned32 ||
          intensity_channel_type_ == pdal::Dimension::Type::Double ||
          intensity_channel_type_ == pdal::Dimension::Type::Float)
      {
        available_channels_ |= DataChannel::Intensity;
      }
    }

    // Resolve time dimension.
    time_dimension_ = pdal::Dimension::Id::Unknown;
    // First try resolve by Dimension ID
    const std::array<pdal::Dimension::Id, 3> time_ids = { pdal::Dimension::Id::GpsTime,
                                                          pdal::Dimension::Id::InternalTime,
                                                          pdal::Dimension::Id::OffsetTime };
    for (const auto &time_dim : time_ids)
    {
      if (layout_.hasDim(time_dim))
      {
        time_dimension_ = time_dim;
        break;
      }
    }

    // Not found. Try resolve by name.
    if (time_dimension_ == pdal::Dimension::Id::Unknown)
    {
      const std::array<const std::string, 2> time_dim_names = { "time", "timestamp" };
      for (const auto &time_name : time_dim_names)
      {
        time_dimension_ = layout_.findDim(time_name);
        if (time_dimension_ != pdal::Dimension::Id::Unknown)
        {
          break;
        }
      }
    }

    if (time_dimension_ != pdal::Dimension::Id::Unknown)
    {
      available_channels_ |= DataChannel::Time;
      time_channel_type_ = layout_.dimDetail(time_dimension_)->type();
    }

    valid_dimensions_ = (available_channels_ & required_channels_) == required_channels_;

    layout_.finalize();
    pdal::StreamPointTable::finalize();
  }
}

bool PointStream::nextPoint(CloudPoint &point)
{
  std::unique_lock<std::mutex> guard(buffer_mutex_);
  const int read_buffer = 1 - write_index_;
  const unsigned read_index = next_read_;
  // Ensure colour pointer is valid, pointing to a stack variable if null.
  bool have_read = false;
  if (next_read_ < buffers_[read_buffer].size())
  {
    point = buffers_[read_buffer][read_index];
    ++next_read_;
    have_read = true;
  }
  guard.unlock();
  flip_wait_.notify_one();
  return have_read;
}

void PointStream::setFieldInternal(pdal::Dimension::Id dim, pdal::PointId idx, const void *val)
{
  if (!abort_)
  {
    auto &point_buffer = buffers_[write_index_];
    while (point_buffer.size() <= idx)
    {
      point_buffer.emplace_back(CloudPoint{});
    }

    auto &sample = point_buffer[idx];

    switch (dim)
    {
    case pdal::Dimension::Id::X:
    case pdal::Dimension::Id::Y:
    case pdal::Dimension::Id::Z: {
      const int pos_index = int(dim) - int(pdal::Dimension::Id::X);
      if (position_channel_types_[pos_index] == pdal::Dimension::Type::Double)
      {
        sample.position[pos_index] = *static_cast<const double *>(val);
      }
      else
      {
        sample.position[pos_index] = double(*static_cast<const float *>(val));
      }
    }
    break;
    case pdal::Dimension::Id::NormalX:
    case pdal::Dimension::Id::NormalY:
    case pdal::Dimension::Id::NormalZ: {
      const int pos_index = int(dim) - int(pdal::Dimension::Id::NormalX);
      if (normal_channel_types_[pos_index] == pdal::Dimension::Type::Double)
      {
        sample.normal[pos_index] = *static_cast<const double *>(val);
      }
      else
      {
        sample.normal[pos_index] = double(*static_cast<const float *>(val));
      }
    }
    break;
    case pdal::Dimension::Id::Red:
    case pdal::Dimension::Id::Green:
    case pdal::Dimension::Id::Blue: {
      const int colour_index = int(dim) - int(pdal::Dimension::Id::Red);
      switch (colour_channel_types_[colour_index])
      {
      case pdal::Dimension::Type::Signed8:  // Ignore sign
      case pdal::Dimension::Type::Unsigned8:
        sample.colour[colour_index] = *static_cast<const uint8_t *>(val);
        break;
      case pdal::Dimension::Type::Signed16:  // Ignore sign
      case pdal::Dimension::Type::Unsigned16:
        sample.colour[colour_index] =
          uint8_t(255.0 * double(*static_cast<const uint16_t *>(val)) / double(std::numeric_limits<uint16_t>::max()));
        break;
      case pdal::Dimension::Type::Signed32:  // Ignore sign
      case pdal::Dimension::Type::Unsigned32:
        sample.colour[colour_index] =
          uint8_t(255.0 * double(*static_cast<const uint32_t *>(val)) / double(std::numeric_limits<uint32_t>::max()));
        break;
      default:
        break;
      }
    }
    break;
    case pdal::Dimension::Id::Intensity:
      switch (intensity_channel_type_)
      {
      case pdal::Dimension::Type::Unsigned8:
        sample.intensity = float(*static_cast<const uint8_t *>(val));
        break;
      case pdal::Dimension::Type::Unsigned16:
        sample.intensity = float(*static_cast<const uint16_t *>(val));
        break;
      case pdal::Dimension::Type::Unsigned32:
        sample.intensity = float(*static_cast<const uint32_t *>(val));
        break;
      case pdal::Dimension::Type::Float:
        sample.intensity = *static_cast<const float *>(val);
        break;
      case pdal::Dimension::Type::Double:
        sample.intensity = float(*static_cast<const double *>(val));
        break;
      default:
        break;
      }
      break;
    default:
      if (dim == time_dimension_)
      {
        if (time_channel_type_ == pdal::Dimension::Type::Double)
        {
          sample.timestamp = *static_cast<const double *>(val);
        }
        else
        {
          sample.timestamp = double(*static_cast<const float *>(val));
        }
      }
      break;
    }
  }
}

/// Called whenever the buffer capacity is filled before starting on the next block.
void PointStream::reset()
{
  if (!abort_)
  {
    std::unique_lock<std::mutex> guard(buffer_mutex_);
    const int read_buffer = 1 - write_index_;
    flip_wait_.wait(guard, [this, read_buffer]() { return abort_ || next_read_ >= buffers_[read_buffer].size(); });
    write_index_ = read_buffer;
    buffers_[read_buffer].clear();
    next_read_ = 0;
    have_data_ = true;
  }
}
}  // namespace slamio
