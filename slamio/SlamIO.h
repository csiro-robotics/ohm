// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_SLAMIO_H_
#define SLAMIO_SLAMIO_H_

#include "SlamIOConfig.h"

#include <memory>

namespace slamio
{
class PointCloudReader;
using PointCloudReaderPtr = std::shared_ptr<PointCloudReader>;

/// Create a @c PointCloudReader from the given filename @p extension . The appropriate reader is created based on the
/// extension.
/// @note This does not call @c PointCloudReader::open() .
/// @param extension The file type extension. No leading `.` is required but may be provided.
/// @return The appropriate reader or a @c nullptr on failure.
PointCloudReaderPtr slamio_API createCloudReader(const char *extension);

/// Create a @c PointCloudReader from the given @p filename . The appropriate reader is created based on the extension.
/// @note This does not call @c PointCloudReader::open() .
/// @param filename The point cloud file to read.
/// @return The appropriate reader or a @c nullptr on failure.
PointCloudReaderPtr slamio_API createCloudReaderFromFilename(const char *filename);
}  // namespace slamio

#endif  // SLAMIO_SLAMIO_H_
