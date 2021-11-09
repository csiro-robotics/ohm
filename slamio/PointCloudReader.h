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

#include <memory>

namespace slamio
{
/// Query the list of standard time field names to search for in input data.
///
/// This returns a pointer to an array of C strings with contains elements equal to @p count .
/// Ownership of the strings is retained by the function (no need to release).
///
/// The list of time fields to search for is (in order):
/// - gps_time
/// - gpstime
/// - internal_time
/// - internaltime
/// - offset_time
/// - offsettime
/// - timestamp
/// - time
///
/// Any search using these values should be case insensitive.
const char slamio_API *const *timeFieldNames(size_t &count);

/// Abstract interface for a point cloud loader. These can be created from functions in @c SlamIO.h
///
/// Typical usage:
/// - Create a @c PointCloudReader via functions in @c SlamIO.h
/// - (optional) Set the @c setDesiredChannels() to flag which data channels to load. Load all present if not set.
/// - Call @c open()
/// - Iteratively call @c readNext() and/or @c readChunk()
/// - @c close() when done (or destroy)
class slamio_API PointCloudReader
{
public:
  /// Virtual, empty destructor.
  virtual ~PointCloudReader();

  /// Query the available data channels (flags).
  virtual DataChannel availableChannels() const = 0;
  /// Query the desired data channels (flags). Maybe @c DataChannel::None to use all available.
  virtual DataChannel desiredChannels() const = 0;
  /// Set the desired data channels (flags). Maybe @c DataChannel::None to use all available.
  /// @param channels The desired channel flags.
  virtual void setDesiredChannels(DataChannel channels) = 0;
  /// Add a desired data channel.
  /// @param channel The data channel to add.
  inline void addDesiredChannel(DataChannel channel) { setDesiredChannels(desiredChannels() | channel); }
  /// Remove a desired data channel.
  /// @param channel The data channel to remove.
  inline void removeDesiredChannel(DataChannel channel) { setDesiredChannels(desiredChannels() & ~channel); }

  /// Query if @c open() has been successfully called.
  /// @return True if open.
  virtual bool isOpen() = 0;
  /// Attempt to open @p filename.
  /// @param filename The file name/path to open.
  /// @return True on success.
  virtual bool open(const char *filename) = 0;
  /// Close the currently open file. Safe to call when not open.
  virtual void close() = 0;

  /// Is data loaded progressively or via a streaming thread? Non-streaming readers may block on calling @c open() .
  /// @return True if streaming load.
  virtual bool streaming() const = 0;

  /// Return the number of points in the cloud. Only valid after calling @c open() .
  /// May be zero if unknown until loading has completed.
  /// @return The number of points in the open cloud file if known, zero if unknown.
  virtual uint64_t pointCount() const = 0;

  /// Read the next data point.
  /// @param point The structure to load into.
  /// @return True if a point has been successfully loaded into @p point . False when there are no more data to load.
  virtual bool readNext(CloudPoint &point) = 0;
  /// Read multiple points
  /// @param points Address to load points into. Must have sufficient space to store @c count elements.
  /// @param count The number of points to try load into @p points .
  /// @return The number of points successfully loaded. Will be less than @c count once the end of file is reached.
  virtual uint64_t readChunk(CloudPoint *points, uint64_t count) = 0;
};

/// @c PointCloudReader pointer typedef.
using PointCloudReaderPtr = std::shared_ptr<PointCloudReader>;
}  // namespace slamio

#endif  // SLAMIO_POINTCLOUDREADER_H_
