// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloudReaderPly.h"

#include "rply/rply.h"
#include "rply/rplyfile.h"

#include <algorithm>
#include <iostream>
#include <memory>

namespace
{
void onPlyError(p_ply ply, const char *message)
{
  (void)ply;
  std::cerr << message << std::endl;
}
}  // namespace

namespace slamio
{
struct RPlyHandle
{
  std::unique_ptr<FILE, int (*)(FILE *)> file;
  std::unique_ptr<t_ply_, int (*)(p_ply)> ply;
  p_ply_element vertex_element = nullptr;
  p_ply_argument ply_argument = nullptr;

  RPlyHandle()
    : file(nullptr, &fclose)
    , ply(nullptr, &ply_close)
  {}

  ~RPlyHandle() { close(); }

  bool open(const char *filename)
  {
    close();
    file = std::unique_ptr<FILE, int (*)(FILE *)>(fopen(filename, "rb"), &fclose);
    if (!file)
    {
      close();
      return false;
    }
    ply = std::unique_ptr<t_ply_, int (*)(p_ply)>(ply_open_from_file(file.get(), &onPlyError, 0, nullptr), &ply_close);
    if (!ply)
    {
      close();
      return false;
    }
    return true;
  }

  void close()
  {
    ply.release();
    file.release();
    vertex_element = nullptr;
  }
};
}  // namespace slamio

namespace
{
/// General vertex property callback.
int vertexProperty(p_ply_argument argument)
{
  long datum_id = 0;
  slamio::PointCloudReaderPly::ReadSampleData *read_data = nullptr;
  ply_get_argument_user_data(argument, (void **)&read_data, &datum_id);
  read_data->properties[datum_id] = ply_get_argument_value(argument) * read_data->scale_factor[datum_id];
  return 1;
}

/// First vertex property callback.
int vertexPropertyFirst(p_ply_argument argument)
{
  long datum_id = 0;
  slamio::PointCloudReaderPly::ReadSampleData *read_data = nullptr;
  ply_get_argument_user_data(argument, (void **)&read_data, &datum_id);
  std::fill(read_data->properties.begin(), read_data->properties.end(), 0.0);
  read_data->properties[datum_id] = ply_get_argument_value(argument) * read_data->scale_factor[datum_id];
  return 1;
}

/// Last vertex property callback.
int vertexPropertyFinalise(p_ply_argument argument)
{
  // Read laste property
  int result = vertexProperty(argument);
  long datum_index = 0;
  slamio::PointCloudReaderPly::ReadSampleData *read_data = nullptr;
  ply_get_argument_user_data(argument, (void **)&read_data, &datum_index);

  // Vertex done.
  const bool have_alpha =
    (read_data->have_property_flags & (1 << unsigned(slamio::PointCloudReaderPly::PlyProperty::kA))) != 0;

  read_data->sample = {};
  read_data->sample.timestamp = read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kTimestamp)];
  read_data->sample.position.x = read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kX)];
  read_data->sample.position.y = read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kY)];
  read_data->sample.position.z = read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kZ)];
  read_data->sample.normal.x = read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kNX)];
  read_data->sample.normal.y = read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kNY)];
  read_data->sample.normal.z = read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kNZ)];
  read_data->sample.colour.r = float(read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kR)]);
  read_data->sample.colour.g = float(read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kG)]);
  read_data->sample.colour.b = float(read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kB)]);
  read_data->sample.colour.a =
    have_alpha ? float(read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kA)]) : 1.0f;
  read_data->sample.intensity =
    float(read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kIntensity)]);
  read_data->sample.return_number = uint8_t(std::round(
    std::min<double>(read_data->properties[unsigned(slamio::PointCloudReaderPly::PlyProperty::kReturnNumber)] *
                       std::numeric_limits<uint8_t>::max(),
                     std::numeric_limits<uint8_t>::max())));

  return result;
}


bool isOneOf(const std::string &value, const std::vector<std::string> &value_set)
{
  return std::find(value_set.begin(), value_set.end(), value) != value_set.end();
}


bool isOneOf(const std::string &value, const std::initializer_list<std::string> &value_set)
{
  for (const auto &test_value : value_set)
  {
    if (value == test_value)
    {
      return true;
    }
  }
  return false;
}

bool isOneOf(const std::string &value, const std::string &test_value)
{
  return value == test_value;
}

bool haveProperty(unsigned flags, slamio::PointCloudReaderPly::PlyProperty property)
{
  return (flags & (1u << unsigned(property))) != 0u;
}

double scaleFactorForType(e_ply_type type)
{
  switch (type)
  {
  case PLY_INT8:
    return 1.0 / double(std::numeric_limits<int8_t>::max());
  case PLY_UINT8:
    return 1.0 / double(std::numeric_limits<uint8_t>::max());
  case PLY_INT16:
    return 1.0 / double(std::numeric_limits<int16_t>::max());
  case PLY_UINT16:
    return 1.0 / double(std::numeric_limits<uint16_t>::max());
  case PLY_INT32:
    return 1.0 / double(std::numeric_limits<int32_t>::max());
  case PLY_UIN32:
    return 1.0 / double(std::numeric_limits<uint32_t>::max());
  case PLY_CHAR:
    return 1.0 / double(std::numeric_limits<int8_t>::max());
  case PLY_UCHAR:
    return 1.0 / double(std::numeric_limits<uint8_t>::max());
  case PLY_SHORT:
    return 1.0 / double(std::numeric_limits<int16_t>::max());
  case PLY_USHORT:
    return 1.0 / double(std::numeric_limits<uint16_t>::max());
  case PLY_INT:
    return 1.0 / double(std::numeric_limits<int32_t>::max());
  case PLY_UINT:
    return 1.0 / double(std::numeric_limits<uint32_t>::max());
  default:
    break;
  }
  return 1.0;
}
}  // namespace

namespace slamio
{
PointCloudReaderPly::PointCloudReaderPly()
  : ply_handle_(std::make_unique<RPlyHandle>())
{}

PointCloudReaderPly::~PointCloudReaderPly()
{
  close();
}

DataChannel PointCloudReaderPly::availableChannels() const
{
  return available_channels_;
}

DataChannel PointCloudReaderPly::desiredChannels() const
{
  return desired_channels_;
}

void PointCloudReaderPly::setDesiredChannels(DataChannel channels)
{
  desired_channels_ = channels;
}

bool PointCloudReaderPly::isOpen()
{
  return ply_handle_->ply.get() != nullptr;
}

bool PointCloudReaderPly::open(const char *filename)
{
  close();

  if (!ply_handle_->open(filename))
  {
    close();
    return false;
  }

  if (!ply_read_header(ply_handle_->ply.get()))
  {
    std::cerr << "Failed to read header for " << filename << std::endl;
    return false;
  }

  readHeader();

  return true;
}

void PointCloudReaderPly::close()
{
  ply_handle_->close();
  read_sample_.sample = {};
  point_count_ = 0;
  available_channels_ = DataChannel::None;
}

bool PointCloudReaderPly::streaming() const
{
  return true;
}

uint64_t PointCloudReaderPly::pointCount() const
{
  return point_count_;
}

bool PointCloudReaderPly::readNext(CloudPoint &point)
{
  if (next_point_index_ < long(point_count_))
  {
    if (ply_read_next_instance(ply_handle_->ply.get(), ply_handle_->vertex_element, ply_handle_->ply_argument,
                               &next_point_index_))
    {
      point = read_sample_.sample;
      return true;
    }
  }

  return false;
}

uint64_t PointCloudReaderPly::readChunk(CloudPoint *point, uint64_t count)
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

bool PointCloudReaderPly::readHeader()
{
  if (!ply_handle_->ply)
  {
    return false;
  }

  std::fill(read_sample_.properties.begin(), read_sample_.properties.end(), 0.0);
  std::fill(read_sample_.scale_factor.begin(), read_sample_.scale_factor.end(), 1.0);

  p_ply ply = ply_handle_->ply.get();
  p_ply_element element = nullptr;
  std::string last_vertex_property_name;
  PlyProperty last_vertex_property_id{};

  std::vector<std::string> time_fields;
  size_t time_field_name_count = 0;
  auto &&time_field_names = timeFieldNames(time_field_name_count);
  time_fields.resize(time_field_name_count);
  for (size_t i = 0; i < time_field_name_count; ++i)
  {
    time_fields[i] = time_field_names[i];
  }

  // iterate over all elements in input file
  unsigned element_count = 0;
  unsigned vertex_element_index = 0;
  while ((element = ply_get_next_element(ply, element)))
  {
    p_ply_property property = nullptr;
    long instances = 0;  // This will unfortunately have a different width on Windows vs Linux.
    const char *element_name;
    ply_get_element_info(element, &element_name, &instances);
    if (strcmp(element_name, "vertex") == 0)
    {
      p_ply_read_cb property_callback = &vertexPropertyFirst;
      vertex_element_index = element_count;
      ply_handle_->vertex_element = element;
      point_count_ = uint64_t(instances);
      // Resolve point data and types.
      const char *property_name{};
      e_ply_type type{};
      e_ply_type length_type{};
      e_ply_type value_type{};
      while ((property = ply_get_next_property(element, property)))
      {
        ply_get_property_info(property, &property_name, &type, &length_type, &value_type);
        if (isOneOf(property_name, time_fields) && (desired_channels_ & DataChannel::Time) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_,
                          long(PlyProperty::kTimestamp));
          last_vertex_property_id = PlyProperty::kTimestamp;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = 1.0;
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, "x") && (desired_channels_ & DataChannel::Position) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kX));
          last_vertex_property_id = PlyProperty::kX;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = 1.0;
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, "y") && (desired_channels_ & DataChannel::Position) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kY));
          last_vertex_property_id = PlyProperty::kY;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = 1.0;
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, "z") && (desired_channels_ & DataChannel::Position) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kZ));
          last_vertex_property_id = PlyProperty::kZ;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = 1.0;
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "nx", "normal_x" }) &&
                 (desired_channels_ & DataChannel::Normal) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kNX));
          last_vertex_property_id = PlyProperty::kNX;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = 1.0;
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "ny", "normal_y" }) &&
                 (desired_channels_ & DataChannel::Normal) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kNY));
          last_vertex_property_id = PlyProperty::kNY;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = 1.0;
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "nz", "normal_z" }) &&
                 (desired_channels_ & DataChannel::Normal) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kNZ));
          last_vertex_property_id = PlyProperty::kNZ;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = 1.0;
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "red", "r" }) &&
                 (desired_channels_ & DataChannel::ColourRgb) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kR));
          last_vertex_property_id = PlyProperty::kR;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = scaleFactorForType(type);
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "green", "g" }) &&
                 (desired_channels_ & DataChannel::ColourRgb) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kG));
          last_vertex_property_id = PlyProperty::kG;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = scaleFactorForType(type);
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "blue", "b" }) &&
                 (desired_channels_ & DataChannel::ColourRgb) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kB));
          last_vertex_property_id = PlyProperty::kB;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = scaleFactorForType(type);
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "alpha", "a" }) &&
                 (desired_channels_ & DataChannel::ColourAlpha) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_, long(PlyProperty::kA));
          last_vertex_property_id = PlyProperty::kA;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = scaleFactorForType(type);
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, "intensity") &&
                 (desired_channels_ & DataChannel::Intensity) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_,
                          long(PlyProperty::kIntensity));
          last_vertex_property_id = PlyProperty::kIntensity;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = scaleFactorForType(type);
          property_callback = &vertexProperty;
        }
        else if (isOneOf(property_name, { "return_number", "returnnumber" }) &&
                 (desired_channels_ & DataChannel::ReturnNumber) != DataChannel::None)
        {
          ply_set_read_cb(ply, element_name, property_name, property_callback, &read_sample_,
                          long(PlyProperty::kReturnNumber));
          last_vertex_property_id = PlyProperty::kReturnNumber;
          last_vertex_property_name = property_name;
          read_sample_.scale_factor[unsigned(last_vertex_property_id)] = scaleFactorForType(type);
          property_callback = &vertexProperty;
        }

        read_sample_.have_property_flags |= (1u << unsigned(last_vertex_property_id));
      }
    }
    // else // not interested for point data
    ++element_count;
  }

  // Change the callback for the last read vertex property to finalise the point.
  ply_set_read_cb(ply, "vertex", last_vertex_property_name.c_str(), vertexPropertyFinalise, &read_sample_,
                  long(last_vertex_property_id));

  // Prime for reading vertex data.
  ply_handle_->ply_argument = ply_get_argument(ply);
  // Skip to the vertex element
  for (unsigned i = 0; i < vertex_element_index; ++i)
  {
    p_ply_element element = ply_get_element(ply, long(i));
    if (!ply_set_argument_element(ply, ply_handle_->ply_argument, element))
    {
      return false;
    }
    if (!ply_read_element(ply, element, ply_handle_->ply_argument))
    {
      return false;
    }
  }

  if (!ply_set_argument_element(ply, ply_handle_->ply_argument, ply_handle_->vertex_element))
  {
    return false;
  }

  // Confirm which data values are available
  const unsigned property_flags = read_sample_.have_property_flags;
  if (haveProperty(property_flags, PlyProperty::kTimestamp))
  {
    available_channels_ |= DataChannel::Time;
  }
  if (haveProperty(property_flags, PlyProperty::kX) && haveProperty(property_flags, PlyProperty::kY) &&
      haveProperty(property_flags, PlyProperty::kZ))
  {
    available_channels_ |= DataChannel::Position;
  }
  if (haveProperty(property_flags, PlyProperty::kNX) && haveProperty(property_flags, PlyProperty::kNY) &&
      haveProperty(property_flags, PlyProperty::kNZ))
  {
    available_channels_ |= DataChannel::Normal;
  }
  if (haveProperty(property_flags, PlyProperty::kR) && haveProperty(property_flags, PlyProperty::kG) &&
      haveProperty(property_flags, PlyProperty::kB))
  {
    available_channels_ |= DataChannel::ColourRgb;
  }
  if (haveProperty(property_flags, PlyProperty::kA))
  {
    available_channels_ |= DataChannel::ColourAlpha;
  }
  if (haveProperty(property_flags, PlyProperty::kIntensity))
  {
    available_channels_ |= DataChannel::Intensity;
  }
  if (haveProperty(property_flags, PlyProperty::kReturnNumber))
  {
    available_channels_ |= DataChannel::ReturnNumber;
  }

  return true;
}
}  // namespace slamio
