// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "SlamIO.h"

#include "PointCloudReaderMiniPly.h"
#include "PointCloudReaderTraj.h"
#include "PointCloudReaderXyz.h"

#if SLAMIO_HAVE_PDAL
#include "PointCloudReaderPdal.h"
#endif  // SLAMIO_HAVE_PDAL

#include <string>

namespace
{
std::string getFileExtension(const std::string &file)
{
  const size_t last_dot = file.find_last_of(".");
  if (last_dot != std::string::npos)
  {
    return file.substr(last_dot + 1);
  }

  return "";
}
}  // namespace

namespace slamio
{
PointCloudReaderPtr createCloudReader(const char *extension_c)
{
  PointCloudReaderPtr reader;

  std::string extension;
  if (extension_c && extension_c[0] == '.')
  {
    // Skip `.` character.
    extension = extension_c + 1;
  }
  else
  {
    extension = extension_c;
  }

  if (extension == "ply")
  {
    reader = std::make_shared<PointCloudReaderMiniPly>();
  }
  else if (extension == "txt")
  {
    reader = std::make_shared<PointCloudReaderTraj>();
  }
  else if (extension == "xyz")
  {
    reader = std::make_shared<PointCloudReaderXyz>();
  }
#if SLAMIO_HAVE_PDAL
  else
  {
    reader = std::make_shared<PointCloudReaderPdal>();
  }
#endif  // SLAMIO_HAVE_PDAL

  return reader;
}

PointCloudReaderPtr createCloudReaderFromFilename(const char *filename)
{
  const auto extension = getFileExtension(filename);
  return createCloudReader(extension.c_str());
}
}  // namespace slamio
