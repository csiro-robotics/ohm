// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_POINTCLOUDREADERMINIPLY_H_
#define SLAMIO_POINTCLOUDREADERMINIPLY_H_

#include "SlamIOConfig.h"

#include "PointCloudReader.h"

#include <memory>
#include <string>
#include <vector>

namespace miniply
{
class PLYReader;
}

namespace slamio
{
class PointCloudReaderMiniPly : public PointCloudReader
{
public:
  PointCloudReaderMiniPly();
  ~PointCloudReaderMiniPly();

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
  void readSamples();

  struct Samples
  {
    std::vector<glm::dvec3> positions;
    std::vector<glm::dvec3> normals;
    std::vector<glm::vec4> colours;
    std::vector<double> timestamps;
    std::vector<float> intensities;

    inline void clear()
    {
      positions.clear();
      normals.clear();
      colours.clear();
      timestamps.clear();
      intensities.clear();
    }

    inline void release()
    {
      clear();
      positions.shrink_to_fit();
      normals.shrink_to_fit();
      colours.shrink_to_fit();
      timestamps.shrink_to_fit();
      intensities.shrink_to_fit();
    }

    void consume(uint64_t consume);
  };

  uint64_t point_count_ = 0;
  uint64_t read_count_ = 0;
  uint64_t samples_cursor_ = 0;
  Samples samples_{};
  DataChannel available_channels_ = DataChannel::None;
  DataChannel desired_channels_ = DataChannel::None;
  std::unique_ptr<miniply::PLYReader> reader_;
  std::string time_field_name_;
  std::string alpha_field_name_;
};
}  // namespace slamio

#endif  // SLAMIO_POINTCLOUDREADERMINIPLY_H_
