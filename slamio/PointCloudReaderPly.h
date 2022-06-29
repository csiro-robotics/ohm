// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_POINTCLOUDREADERPLY_H_
#define SLAMIO_POINTCLOUDREADERPLY_H_

#include "SlamIOConfig.h"

#include "PointCloudReader.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace slamio
{
struct RPlyHandle;

/// A PLY point cloud loading using miniply.
class slamio_API PointCloudReaderPly : public PointCloudReader
{
public:
  /// Set of relevant PLY point properties to try read.
  enum class PlyProperty : unsigned
  {
    kTimestamp,     ///< Point timestamp.
    kX,             ///< X position coordinate.
    kY,             ///< Y position coordinate.
    kZ,             ///< Z position coordinate.
    kNX,            ///< Point normal vector X component.
    kNY,            ///< Point normal vector Y component.
    kNZ,            ///< Point normal vector X component.
    kR,             ///< Point cloud red channel.
    kG,             ///< Point cloud green channel.
    kB,             ///< Point cloud blue channel.
    kA,             ///< Point cloud alpha channel (optional).
    kIntensity,     ///< Point intensity channel.
    kReturnNumber,  ///< Point return number.
    kCount          ///< Number of items in the @c PlyProperty enumeration.
  };

  /// User data structure while reading from rply library.
  struct slamio_API ReadSampleData
  {
    /// Next point property values.
    std::array<double, unsigned(PlyProperty::kCount)> properties;
    /// Scale factor to apply when loading properties for type conversion. E.g., convert uint16 colour to range [0, 1].
    std::array<double, unsigned(PlyProperty::kCount)> scale_factor;
    /// Flags indicating which proeprties we have
    unsigned have_property_flags = 0;
    /// Last loaded sample point.
    CloudPoint sample;
  };

  PointCloudReaderPly();
  ~PointCloudReaderPly();

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
  bool readHeader();

  uint64_t point_count_ = 0;
  long next_point_index_ = 0;
  ReadSampleData read_sample_;
  DataChannel available_channels_ = DataChannel::None;
  DataChannel desired_channels_ = DataChannel::Time | DataChannel::Position;
  std::unique_ptr<RPlyHandle> ply_handle_;
};
}  // namespace slamio

#endif  // SLAMIO_POINTCLOUDREADERPLY_H_
