// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_POINTCLOUDREADERTRAJ_H_
#define SLAMIO_POINTCLOUDREADERTRAJ_H_

#include "SlamIOConfig.h"

#include "PointCloudReader.h"

#include <fstream>
#include <memory>
#include <string>

namespace miniply
{
class PLYReader;
}

namespace slamio
{
/// Text file trajectory reader.
///
/// Text file format assumptions:
/// - First like may or may not be a headings line. Ignored on failure to read as numeric values.
/// - One data item per line.
/// - Data line format is : `time x y z <additional_fields>`
/// - Data in a line are space delimited
/// - @c `<additional_fields>` are ignored
class slamio_API PointCloudReaderTraj : public PointCloudReader
{
public:
  PointCloudReaderTraj();
  ~PointCloudReaderTraj();

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

  std::ifstream file_in_;
  std::string data_line_;
  bool eof_ = false;
  DataChannel desired_channels_ = DataChannel::Position | DataChannel::Time;
};
}  // namespace slamio

#endif  // SLAMIO_POINTCLOUDREADERTRAJ_H_
