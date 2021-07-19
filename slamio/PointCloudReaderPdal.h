// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef SLAMIO_POINTCLOUDREADERPDAL_H_
#define SLAMIO_POINTCLOUDREADERPDAL_H_

#include "slamio/SlamIOConfig.h"

#include "PointCloudReader.h"

#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>

#include <memory>
#include <thread>

namespace slamio
{
class PointStream;

/// A point cloud loader which uses PDAL to load files. Requires PDAL 1.7 as it aims to support streaming via a
/// background thread.
///
/// Supports reading all file types the installed PDAL library supports.
///
/// Note: streaming cannot be interrupted once started. This means that the program cannot exit until file loading
/// has completed.
class PointCloudReaderPdal : public PointCloudReader
{
public:
  PointCloudReaderPdal();
  ~PointCloudReaderPdal();

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
  std::unique_ptr<pdal::StageFactory> pdal_factory_;
  std::shared_ptr<pdal::Streamable> cloud_reader_;
  std::unique_ptr<PointStream> point_stream_;
  pdal::point_count_t point_count_ = 0;
  DataChannel available_channels_ = DataChannel::None;
  DataChannel desired_channels_ = DataChannel::None;
  std::thread sample_thread_;
};
}  // namespace slamio

#endif  // SLAMIO_POINTCLOUDREADERPDAL_H_
