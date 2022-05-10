// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloudReaderPdal.h"

#if SLAMIO_HAVE_PDAL_STREAMS
#ifdef _MSC_VER
#pragma warning(push)
// Disable warning about inheritance via dominance from pdal code.
#pragma warning(disable : 4250)
#endif  // _MSC_VER
// Internal stream support.
#include "pdal/PointStream.h"
#endif  // SLAMIO_HAVE_PDAL_STREAMS

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#if SLAMIO_HAVE_PDAL_STREAMS
#include <pdal/io/BpfReader.hpp>
#include <pdal/io/LasReader.hpp>
#endif  // SLAMIO_HAVE_PDAL_STREAMS

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

slamio::PointCloudReaderPdal::PdalReaderPtr createReader(pdal::StageFactory &factory, const std::string &file_name)
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

#if SLAMIO_HAVE_PDAL_STREAMS
  if (!reader->pipelineStreamable())
  {
    std::cout << "PDAL reader for " << reader_type << " does not support streaming" << std::endl;
    return nullptr;
  }

  auto streamable_reader = std::dynamic_pointer_cast<pdal::Streamable>(reader);
#endif  // SLAMIO_HAVE_PDAL_STREAMS

  if (reader)
  {
    options.add("filename", file_name);
    reader->setOptions(options);
  }

#if SLAMIO_HAVE_PDAL_STREAMS
  return streamable_reader;
#else   // SLAMIO_HAVE_PDAL_STREAMS
  return reader;
#endif  // SLAMIO_HAVE_PDAL_STREAMS
}

#if SLAMIO_HAVE_PDAL_STREAMS
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
#else   // SLAMIO_HAVE_PDAL_STREAMS
pdal::Dimension::Id findField(pdal::PointTable &point_table, const std::vector<pdal::Dimension::Id> ids,
                              const std::vector<std::string> &names, pdal::Dimension::Type *dim_type = nullptr)
{
  pdal::Dimension::Id field_id = pdal::Dimension::Id::Unknown;
  for (const auto &id : ids)
  {
    if (point_table.layout()->hasDim(id))
    {
      field_id = id;
      break;
    }
  }

  for (const auto &name : names)
  {
    const auto id = point_table.layout()->findDim(name);
    if (id != pdal::Dimension::Id::Unknown)
    {
      field_id = id;
      break;
    }
  }

  if (dim_type && field_id != pdal::Dimension::Id::Unknown)
  {
    *dim_type = point_table.layout()->dimType(field_id);
  }

  return field_id;
}

float colourScaleForType(pdal::Dimension::Type dim_type)
{
  switch (dim_type)
  {
  case pdal::Dimension::Type::Unsigned8:
    return 1.0f / 255.0f;
  case pdal::Dimension::Type::Unsigned16:
    return 1.0f / float(std::numeric_limits<uint16_t>::max());
  case pdal::Dimension::Type::Unsigned32:
    return 1.0f / float(std::numeric_limits<uint32_t>::max());
  default:
    break;
  }

  return 1.0f;
}
#endif  // SLAMIO_HAVE_PDAL_STREAMS
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

#if SLAMIO_HAVE_PDAL_STREAMS
  point_stream_ = std::make_unique<PointStream>(kCloudStreamBufferSize, required);
  cloud_reader_->prepare(*point_stream_);

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
  available_channels_ |= (point_stream_->hasNormals()) ? DataChannel::Normal : DataChannel::None;
  available_channels_ |= (point_stream_->hasColourRgb()) ? DataChannel::ColourRgb : DataChannel::None;
  available_channels_ |= (point_stream_->hasColourAlpha()) ? DataChannel::ColourAlpha : DataChannel::None;
  available_channels_ |= (point_stream_->hasIntensity()) ? DataChannel::Intensity : DataChannel::None;

  point_count_ = pdalPointCount(cloud_reader_);
#else   // SLAMIO_HAVE_PDAL_STREAMS
  point_table_ = std::make_unique<pdal::PointTable>();
  cloud_reader_->prepare(*point_table_);

  available_channels_ = DataChannel::None;

  std::vector<std::string> time_fields;
  size_t field_name_count = 0;
  const auto *time_field_names = timeFieldNames(field_name_count);
  time_fields.resize(field_name_count);
  std::copy(time_field_names, time_field_names + field_name_count, time_fields.begin());

  fields_.time = findField(
    *point_table_, { pdal::Dimension::Id::GpsTime, pdal::Dimension::Id::InternalTime, pdal::Dimension::Id::OffsetTime },
    time_fields);

  if (fields_.time != pdal::Dimension::Id::Unknown)
  {
    available_channels_ |= DataChannel::Time;
  }

  if (findField(*point_table_, { pdal::Dimension::Id::X }, {}) != pdal::Dimension::Id::Unknown &&
      findField(*point_table_, { pdal::Dimension::Id::Y }, {}) != pdal::Dimension::Id::Unknown &&
      findField(*point_table_, { pdal::Dimension::Id::Z }, {}) != pdal::Dimension::Id::Unknown)
  {
    available_channels_ |= DataChannel::Position;
  }

  if (findField(*point_table_, { pdal::Dimension::Id::NormalX }, {}) != pdal::Dimension::Id::Unknown &&
      findField(*point_table_, { pdal::Dimension::Id::NormalY }, {}) != pdal::Dimension::Id::Unknown &&
      findField(*point_table_, { pdal::Dimension::Id::NormalZ }, {}) != pdal::Dimension::Id::Unknown)
  {
    available_channels_ |= DataChannel::Normal;
  }

  pdal::Dimension::Type rgba_types[4] = { pdal::Dimension::Type::None };
  if (findField(*point_table_, { pdal::Dimension::Id::Red }, {}, &rgba_types[0]) != pdal::Dimension::Id::Unknown &&
      findField(*point_table_, { pdal::Dimension::Id::Green }, {}, &rgba_types[1]) != pdal::Dimension::Id::Unknown &&
      findField(*point_table_, { pdal::Dimension::Id::Blue }, {}, &rgba_types[2]) != pdal::Dimension::Id::Unknown)
  {
    if (rgba_types[0] == rgba_types[1] && rgba_types[0] == rgba_types[1])
    {
      available_channels_ |= DataChannel::ColourRgb;
      fields_.rgb_scale = colourScaleForType(rgba_types[0]);
    }
  }

  if (findField(*point_table_, { pdal::Dimension::Id::Alpha }, {}, &rgba_types[3]) != pdal::Dimension::Id::Unknown)
  {
    available_channels_ |= DataChannel::ColourAlpha;
    fields_.alpha_scale = colourScaleForType(rgba_types[3]);
  }

  if (findField(*point_table_, { pdal::Dimension::Id::Intensity }, {}) != pdal::Dimension::Id::Unknown)
  {
    available_channels_ |= DataChannel::Intensity;
  }

  std::vector<std::string> return_number_fields;
  field_name_count = 0;
  const auto *return_number_field_names = returnNumberFieldNames(field_name_count);
  return_number_fields.resize(field_name_count);
  std::copy(return_number_field_names, return_number_field_names + field_name_count, return_number_fields.begin());

  if (findField(*point_table_, { pdal::Dimension::Id::ReturnNumber }, return_number_fields) !=
      pdal::Dimension::Id::Unknown)
  {
    available_channels_ |= DataChannel::Intensity;
  }

  pdal::PointViewSet point_sets = cloud_reader_->execute(*point_table_);
  samples_view_ = *point_sets.begin();

  point_count_ = samples_view_->size();
#endif  // SLAMIO_HAVE_PDAL_STREAMS

  // Set the desired channels if not set yet.
  if (desired_channels_ == DataChannel::None)
  {
    desired_channels_ = available_channels_;
  }

#if SLAMIO_HAVE_PDAL_STREAMS
  sample_thread_ = std::thread([this]() {  //
    cloud_reader_->execute(*point_stream_);
    point_stream_->markLoadComplete();
  });
#endif  // SLAMIO_HAVE_PDAL_STREAMS

  return true;
}

void PointCloudReaderPdal::close()
{
#if SLAMIO_HAVE_PDAL_STREAMS
  if (point_stream_)
  {
    point_stream_->abort();
  }
#else   // SLAMIO_HAVE_PDAL_STREAMS
  samples_view_ = nullptr;
  point_table_ = nullptr;
  samples_view_index_ = 0;
  fields_ = PointFields{};
#endif  // SLAMIO_HAVE_PDAL_STREAMS
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
#if SLAMIO_HAVE_PDAL_STREAMS
  return true;
#else   // SLAMIO_HAVE_PDAL_STREAMS
  return false;
#endif  // SLAMIO_HAVE_PDAL_STREAMS
}


uint64_t PointCloudReaderPdal::pointCount() const
{
  return point_count_;
}


bool PointCloudReaderPdal::readNext(CloudPoint &point)
{
#if SLAMIO_HAVE_PDAL_STREAMS
  // FIXME: Should really use a condition variable in nextPoint() rather than busy wait.
  bool have_read = point_stream_->nextPoint(point);
  while (!point_stream_->done() && !have_read)
  {
    std::this_thread::yield();
    have_read = point_stream_->nextPoint(point);
  }

  return have_read;
#else   // SLAMIO_HAVE_PDAL_STREAMS
  if (samples_view_index_ < point_count_)
  {
    if ((available_channels_ & DataChannel::Time) != DataChannel::None)
    {
      point.timestamp = samples_view_->getFieldAs<double>(fields_.time, samples_view_index_);
    }
    else
    {
      point.timestamp = 0;
    }
    point.position.x = samples_view_->getFieldAs<double>(pdal::Dimension::Id::X, samples_view_index_);
    point.position.y = samples_view_->getFieldAs<double>(pdal::Dimension::Id::Y, samples_view_index_);
    point.position.z = samples_view_->getFieldAs<double>(pdal::Dimension::Id::Z, samples_view_index_);

    if ((available_channels_ & DataChannel::Normal) != DataChannel::None)
    {
      point.normal.x = samples_view_->getFieldAs<double>(pdal::Dimension::Id::NormalX, samples_view_index_);
      point.normal.y = samples_view_->getFieldAs<double>(pdal::Dimension::Id::NormalY, samples_view_index_);
      point.normal.z = samples_view_->getFieldAs<double>(pdal::Dimension::Id::NormalZ, samples_view_index_);
    }

    if ((available_channels_ & DataChannel::ColourRgb) != DataChannel::None)
    {
      point.colour[0] =
        samples_view_->getFieldAs<float>(pdal::Dimension::Id::Red, samples_view_index_) * fields_.rgb_scale;
      point.colour[1] =
        samples_view_->getFieldAs<float>(pdal::Dimension::Id::Green, samples_view_index_) * fields_.rgb_scale;
      point.colour[2] =
        samples_view_->getFieldAs<float>(pdal::Dimension::Id::Blue, samples_view_index_) * fields_.rgb_scale;
    }
    if ((available_channels_ & DataChannel::ColourAlpha) != DataChannel::None)
    {
      point.colour[3] =
        samples_view_->getFieldAs<float>(pdal::Dimension::Id::Alpha, samples_view_index_) * fields_.alpha_scale;
    }

    if ((available_channels_ & DataChannel::Intensity) != DataChannel::None)
    {
      point.intensity = samples_view_->getFieldAs<float>(pdal::Dimension::Id::Intensity, samples_view_index_);
    }

    ++samples_view_index_;
    return true;
  }
  return false;
#endif  // SLAMIO_HAVE_PDAL_STREAMS
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

#if SLAMIO_HAVE_PDAL_STREAMS
#ifdef _MSC_VER
#pragma warning(pop)
#endif  // _MSC_VER
#endif  // SLAMIO_HAVE_PDAL_STREAMS
