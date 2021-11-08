// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_POINTCLOUDREADERXYZ_H_
#define SLAMIO_POINTCLOUDREADERXYZ_H_

#include "SlamIOConfig.h"

#include "PointCloudReader.h"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace miniply
{
class PLYReader;
}

namespace slamio
{
/// XYZ point cloud text file reader.
///
/// Text file format assumptions:
/// - First line is a headings line.
/// - First line specifies the available @c DataChannel items.
/// - One data item per line.
/// - Only interested in the following @c DataChannel items:
///   - @c DataChannel::Time (required)
///   - @c DataChannel::Position (required)
///   - @c DataChannel::Normals (optional)
/// - @c DataChannel::Time is derived from the following headings in preferential order (case insensitive):
///   - gps_time
///   - gpstime
///   - internal_time
///   - internaltime
///   - offset_time
///   - offsettime
///   - timestamp
///   - time
/// - Data in a line are space delimited
class PointCloudReaderXyz : public PointCloudReader
{
public:
  PointCloudReaderXyz();
  ~PointCloudReaderXyz();

  DataChannel availableChannels() const override;
  DataChannel desiredChannels() const override;
  void setDesiredChannels(DataChannel channels) override;

  bool isOpen() override;
  bool open(const char *filename) override;
  void close() override;

  bool streaming() const override;

  uint64_t pointCount() const override;
  bool readNext(CloudPoint &point) override;
  uint64_t readChunk(CloudPoint *point, uint64_t count) override;

private:
  using FilePtr = std::unique_ptr<FILE, int (*)(FILE *)>;

  bool readHeadings();

  std::ifstream file_in_;
  std::string data_line_;
  bool eof_ = false;
  DataChannel desired_channels_ = DataChannel::Position | DataChannel::Time;
  DataChannel available_channels_ = DataChannel::None;
  std::vector<double> values_buffer_;
  size_t time_index_ = ~0u;
  size_t x_index_ = ~0u;
  size_t y_index_ = ~0u;
  size_t z_index_ = ~0u;
  size_t nx_index_ = ~0u;
  size_t ny_index_ = ~0u;
  size_t nz_index_ = ~0u;
};
}  // namespace slamio

#endif  // SLAMIO_POINTCLOUDREADERXYZ_H_
