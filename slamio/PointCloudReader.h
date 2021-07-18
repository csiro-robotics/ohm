// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_POINTCLOUDREADER_H_
#define SLAMIO_POINTCLOUDREADER_H_

#include "SlamIOConfig.h"

#include "DataChannel.h"
#include "Points.h"

namespace slamio
{
class slamio_API PointCloudReader
{
public:
  virtual ~PointCloudReader();

  virtual DataChannel availableChannels() const = 0;
  virtual DataChannel desiredChannels() const = 0;
  virtual void setDesiredChannels(DataChannel channels) = 0;
  inline void addDesiredChannel(DataChannel channel) { setDesiredChannels(desiredChannels() | channel); }
  inline void removeDesiredChannel(DataChannel channel) { setDesiredChannels(desiredChannels() & ~channel); }

  virtual bool isOpen() = 0;
  virtual bool open(const char *filename) = 0;
  virtual void close() = 0;

  virtual bool streaming() const = 0;

  virtual uint64_t pointCount() const = 0;
  virtual bool readNext(CloudPoint &point) = 0;
  virtual uint64_t readChunk(CloudPoint *point, uint64_t count) = 0;
};
}  // namespace slamio

#endif  // SLAMIO_POINTCLOUDREADER_H_
