// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloudReaderPdal.h"

// Internal stream support.
#include "pdal/PointStream.h"

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/io/BpfReader.hpp>
#include <pdal/io/LasReader.hpp>

namespace
{
const size_t kCloudStreamBufferSize = 10000u;

std::string getFileExtension(const std::string &file)
{
  const size_t last_dot = file.find_last_of('.');
  if (last_dot != std::string::npos)
  {
    return file.substr(last_dot + 1);
  }

  return "";
}

std::shared_ptr<pdal::Streamable> createReader(pdal::StageFactory &factory, const std::string &file_name)
{
  const std::string ext = getFileExtension(file_name);
  std::string reader_type;
  pdal::Options options;

  reader_type = ext;

  if (ext == "laz")
  {
    reader_type = "las";
    options.add("compression", "EITHER");
  }
  else if (ext == "xyz")
  {
    reader_type = "text";
  }

  reader_type = "readers." + reader_type;
  std::shared_ptr<pdal::Stage> reader(factory.createStage(reader_type),  //
                                      [&factory](pdal::Stage *stage) { factory.destroyStage(stage); });

  if (!reader)
  {
    std::cerr << "PDAL reader for " << reader_type << " not available" << std::endl;
    return nullptr;
  }

  if (!reader->pipelineStreamable())
  {
    std::cout << "PDAL reader for " << reader_type << " does not support streaming" << std::endl;
    return nullptr;
  }

  auto streamable_reader = std::dynamic_pointer_cast<pdal::Streamable>(reader);

  if (streamable_reader)
  {
    options.add("filename", file_name);
    reader->setOptions(options);
  }

  return streamable_reader;
}

template <typename T, typename Func>
bool tryPdalPointCount(std::shared_ptr<pdal::Streamable> reader, Func func, pdal::point_count_t &point_count)
{
  if (T *r = dynamic_cast<T *>(reader.get()))
  {
    const auto count = (r->*func)();
    if (count < std::numeric_limits<decltype(count)>::max())
    {
      point_count = count;
      return true;
    }
  }
  return false;
}


pdal::point_count_t pdalPointCount(std::shared_ptr<pdal::Streamable> reader)
{
  // Doesn't seem to be a consistent way to read the point count. `Reader::count()` didn't work with las, but casting
  // and calling `LasReader::getNumPoints()` does.

  pdal::point_count_t point_count = 0;
  if (tryPdalPointCount<pdal::BpfReader>(reader, &pdal::BpfReader::numPoints, point_count))
  {
    return point_count;
  }

  if (tryPdalPointCount<pdal::LasReader>(reader, &pdal::LasReader::getNumPoints, point_count))
  {
    return point_count;
  }

  if (tryPdalPointCount<pdal::Reader>(reader, &pdal::Reader::count, point_count))
  {
    return point_count;
  }

  return point_count;
}
}  // namespace

namespace slamio
{
PointCloudReaderPdal::PointCloudReaderPdal()
  : pdal_factory_(std::make_unique<pdal::StageFactory>())
{}

PointCloudReaderPdal::~PointCloudReaderPdal()
{
  close();
}

DataChannel PointCloudReaderPdal::availableChannels() const
{
  return available_channels_;
}
DataChannel PointCloudReaderPdal::desiredChannels() const
{
  return desired_channels_;
}
void PointCloudReaderPdal::setDesiredChannels(DataChannel channels)
{
  desired_channels_ = channels;
}

bool PointCloudReaderPdal::isOpen()
{
  return cloud_reader_ != nullptr;
}

bool PointCloudReaderPdal::open(const char *filename)
{
  // Get the reader name for the file being loaded.
  cloud_reader_ = createReader(*pdal_factory_, filename);
  if (!cloud_reader_)
  {
    close();
    return false;
  }

  const DataChannel required =
    (desired_channels_ == DataChannel::None) ? (DataChannel::Position | DataChannel::Time) : desired_channels_;
  point_stream_ = std::make_unique<PointStream>(kCloudStreamBufferSize, required);
  cloud_reader_->prepare(*point_stream_);
  point_count_ = pdalPointCount(cloud_reader_);

  point_stream_->finalize();
  if (!point_stream_->isValid())
  {
    std::cerr << "Unable to resolve time field in " << filename << std::endl;
    std::cerr << "Require point X, Y, Z and time fields" << std::endl;
    close();
    return false;
  }

  // Determine available data channels.
  available_channels_ |= DataChannel::Position;
  available_channels_ |= (point_stream_->hasTimestamp()) ? DataChannel::Time : DataChannel::None;
  available_channels_ |= (point_stream_->hasNormals()) ? DataChannel::Normals : DataChannel::None;
  available_channels_ |= (point_stream_->hasColour()) ? DataChannel::Colour : DataChannel::None;
  available_channels_ |= (point_stream_->hasIntensity()) ? DataChannel::Intensity : DataChannel::None;

  // Set the desired channels if not set yet.
  if (desired_channels_ == DataChannel::None)
  {
    desired_channels_ = available_channels_;
  }

  sample_thread_ = std::thread([this]() {  //
    cloud_reader_->execute(*point_stream_);
    point_stream_->markLoadComplete();
  });

  return true;
}

void PointCloudReaderPdal::close()
{
  if (point_stream_)
  {
    point_stream_->abort();
  }
  if (cloud_reader_)
  {
    if (sample_thread_.joinable())
    {
      sample_thread_.join();
    }
  }
  cloud_reader_ = nullptr;
  point_count_ = 0;
}


bool PointCloudReaderPdal::streaming() const
{
  return true;
}


uint64_t PointCloudReaderPdal::pointCount() const
{
  return point_count_;
}


bool PointCloudReaderPdal::readNext(CloudPoint &point)
{
  bool have_read = point_stream_->nextPoint(point);
  while (!point_stream_->done() && !have_read)
  {
    std::this_thread::yield();
    have_read = point_stream_->nextPoint(point);
  }

  return have_read;
}


uint64_t PointCloudReaderPdal::readChunk(CloudPoint *point, uint64_t count)
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
