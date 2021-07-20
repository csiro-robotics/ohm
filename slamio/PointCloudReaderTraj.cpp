// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloudReaderTraj.h"

#include "miniply/miniply.h"
namespace slamio
{
PointCloudReaderTraj::PointCloudReaderTraj() = default;
PointCloudReaderTraj::~PointCloudReaderTraj() = default;


DataChannel PointCloudReaderTraj::availableChannels() const
{
  return DataChannel::Position | DataChannel::Time;
}

DataChannel PointCloudReaderTraj::desiredChannels() const
{
  return desired_channels_;
}

void PointCloudReaderTraj::setDesiredChannels(DataChannel channels)
{
  desired_channels_ = channels;
}

bool PointCloudReaderTraj::isOpen()
{
  return file_in_.is_open();
}

bool PointCloudReaderTraj::open(const char *filename)
{
  close();
  file_in_.open(filename, std::ios::binary);

  if (!file_in_.is_open())
  {
    close();
    return false;
  }

  // Try read the first data line. It may be valid or it may be headings.
  CloudPoint point;
  auto reset_pos = file_in_.tellg();
  if (!readNext(point))
  {
    reset_pos = file_in_.tellg();
    if (!readNext(point))
    {
      // Data not ok.
      close();
      return false;
    }
    else
    {
      // Second line ok.
    }
  }
  else
  {
    // First line is ok.
  }

  // Reset to the first valid data line.
  file_in_.seekg(reset_pos);

  if (desired_channels_ == DataChannel::None)
  {
    desired_channels_ = DataChannel::Position | DataChannel::Time;
  }

  return true;
}

void PointCloudReaderTraj::close()
{
  file_in_.close();
  desired_channels_ = DataChannel::None;
}

bool PointCloudReaderTraj::streaming() const
{
  return true;
}

uint64_t PointCloudReaderTraj::pointCount() const
{
  return 0;  // Not known.
}

bool PointCloudReaderTraj::readNext(CloudPoint &point)
{
  if (!eof_)
  {
    if (!std::getline(file_in_, data_line_))
    {
      // End of file.
      eof_ = true;
      return false;
    }

    // sscanf is far faster than using stream operators.
#ifdef _MSC_VER
    if (sscanf_s(data_line_.c_str(), "%lg %lg %lg %lg", &point.timestamp, &point.position.x, &point.position.y,
                 &point.position.z) == 4)
#else // _MSC_VER
    if (sscanf(data_line_.c_str(), "%lg %lg %lg %lg", &point.timestamp, &point.position.x, &point.position.y,
               &point.position.z) == 4)
#endif  // _MSC_VER
{
  return true;
}
  }

  return false;
}

uint64_t PointCloudReaderTraj::readChunk(CloudPoint *point, uint64_t count)
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
}  // namespace slamio
