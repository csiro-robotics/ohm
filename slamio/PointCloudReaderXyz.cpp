// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloudReaderXyz.h"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace slamio
{
PointCloudReaderXyz::PointCloudReaderXyz() = default;
PointCloudReaderXyz::~PointCloudReaderXyz()
{
  close();
}


DataChannel PointCloudReaderXyz::availableChannels() const
{
  return available_channels_;
}

DataChannel PointCloudReaderXyz::desiredChannels() const
{
  return desired_channels_;
}

void PointCloudReaderXyz::setDesiredChannels(DataChannel channels)
{
  desired_channels_ = channels;
}

bool PointCloudReaderXyz::isOpen()
{
  return file_in_.is_open();
}

bool PointCloudReaderXyz::open(const char *filename)
{
  close();
  file_in_.open(filename, std::ios::binary);

  if (!file_in_.is_open())
  {
    close();
    return false;
  }

  // Try read the first data line. Looking to decode headings.
  if (!readHeadings())
  {
    // No headings available.
    close();
    return false;
  }

  if (desired_channels_ == DataChannel::None)
  {
    desired_channels_ = DataChannel::Position | DataChannel::Time;
  }

  return true;
}

void PointCloudReaderXyz::close()
{
  file_in_.close();
  available_channels_ = DataChannel::None;
}

bool PointCloudReaderXyz::streaming() const
{
  return true;
}

uint64_t PointCloudReaderXyz::pointCount() const
{
  return 0;  // Not known.
}

bool PointCloudReaderXyz::readNext(CloudPoint &point)
{
  if (!eof_)
  {
    if (!std::getline(file_in_, data_line_))
    {
      // End of file.
      eof_ = true;
      return false;
    }

    // sscanf is far faster than using stream operators, but the unknown number of data items on a line mandate streams
    std::istringstream istr(data_line_);

    for (auto &value : values_buffer_)
    {
      istr >> value;
    }

    if (!istr.fail())
    {
      point.timestamp = values_buffer_[time_index_];
      point.position.x = values_buffer_[x_index_];
      point.position.y = values_buffer_[y_index_];
      point.position.z = values_buffer_[z_index_];
      return true;
    }
  }

  return false;
}

uint64_t PointCloudReaderXyz::readChunk(CloudPoint *point, uint64_t count)
{
  uint64_t read_count = 0;

  for (uint64_t i = 0; i < count; ++i)
  {
    if (readNext(point[i]))
    {
      ++read_count;
    }
    else
    {
      break;
    }
  }

  return read_count;
}


namespace
{
const size_t heading_not_found = ~size_t(0u);


size_t headingIndex(const std::string &name, const std::vector<std::string> &headings)
{
  for (size_t i = 0; i < headings.size(); ++i)
  {
    if (name == headings[i])
    {
      return i;
    }
  }

  return heading_not_found;
}


size_t headingIndex(const std::vector<std::string> &names, const std::vector<std::string> &headings)
{
  for (const auto &name : names)
  {
    const size_t name_index = headingIndex(name, headings);
    if (name_index != heading_not_found)
    {
      return name_index;
    }
  }
  return heading_not_found;
}
}  // namespace


bool PointCloudReaderXyz::readHeadings()
{
  if (!eof_)
  {
    if (!std::getline(file_in_, data_line_))
    {
      // End of file.
      eof_ = true;
      return false;
    }
  }

  // Parse headings.
  std::istringstream istr(data_line_);
  std::vector<std::string> headings;
  std::string token;
  while (!istr.fail())
  {
    istr >> token;
    if (!istr.fail())
    {
      // Convert to lower.
      std::transform(token.begin(), token.end(), token.begin(), tolower);
      headings.emplace_back(token);
    }
  }

  // Resolve heading data items.
  std::vector<std::string> time_fields;
  size_t time_field_name_count = 0;
  auto &&time_field_names = timeFieldNames(time_field_name_count);
  time_fields.resize(time_field_name_count);
  for (size_t i = 0; i < time_field_name_count; ++i)
  {
    time_fields[i] = time_field_names[i];
  }
  time_index_ = headingIndex(time_fields, headings);

  x_index_ = headingIndex("x", headings);
  y_index_ = headingIndex("y", headings);
  z_index_ = headingIndex("z", headings);

  nx_index_ = headingIndex("nx", headings);
  ny_index_ = headingIndex("ny", headings);
  nz_index_ = headingIndex("nz", headings);

  available_channels_ = DataChannel::None;
  size_t required_values_count = 0;
  if (time_index_ != heading_not_found)
  {
    available_channels_ |= DataChannel::Time;
    required_values_count = std::max(required_values_count, time_index_ + 1);
  }
  if (x_index_ != heading_not_found && y_index_ != heading_not_found && z_index_ != heading_not_found)
  {
    available_channels_ |= DataChannel::Position;
    required_values_count = std::max(required_values_count, x_index_ + 1);
    required_values_count = std::max(required_values_count, y_index_ + 1);
    required_values_count = std::max(required_values_count, z_index_ + 1);
  }
  if (nx_index_ != heading_not_found && ny_index_ != heading_not_found && nz_index_ != heading_not_found)
  {
    available_channels_ |= DataChannel::Normal;
    required_values_count = std::max(required_values_count, nx_index_ + 1);
    required_values_count = std::max(required_values_count, ny_index_ + 1);
    required_values_count = std::max(required_values_count, nz_index_ + 1);
  }

  values_buffer_.resize(required_values_count);

  return available_channels_ == (DataChannel::Position | DataChannel::Time);
}
}  // namespace slamio
