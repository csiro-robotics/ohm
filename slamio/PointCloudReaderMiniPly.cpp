// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloudReaderMiniPly.h"

#include "miniply/miniply.h"

namespace
{
const std::vector<std::string> time_fields = { "time", "timestamp", "gpstime", "offsettime", "internaltime" };

void checkAvailable(bool &found_any, bool &found_all, const miniply::PLYElement &element,
                    const std::vector<std::string> &fields, std::string *first_field = nullptr)
{
  found_any = false;
  found_all = true;
  bool have_found = false;
  for (const auto &field : fields)
  {
    const bool found = element.find_property(field.c_str()) != miniply::kInvalidIndex;
    found_any = found_any || found;
    found_all = found_all && found;
    if (!have_found && first_field)
    {
      *first_field = field;
    }
    have_found = have_found || found;
  }
}

bool checkAllAvailable(const miniply::PLYElement &element, const std::vector<std::string> &fields)
{
  bool found_any = false;
  bool found_all = true;
  checkAvailable(found_any, found_all, element, fields);
  return found_any || found_all;
}

bool checkAnyAvailable(const miniply::PLYElement &element, const std::vector<std::string> &fields,
                       std::string *selected)
{
  bool found_any = false;
  bool found_all = true;
  checkAvailable(found_any, found_all, element, fields, selected);
  return found_any;
}

void fixColourRange(std::vector<glm::vec4> &colours, miniply::PLYReader &reader, std::string &alpha_field_name)
{
  auto *element = reader.element();

  if (!element)
  {
    return;
  }

  uint32_t offsets[4];
  const bool have_colours = reader.find_color(offsets);
  offsets[3] = reader.find_property(alpha_field_name.c_str());
  const bool have_alpha = offsets[3] != miniply::kInvalidIndex;

  if (have_colours || have_alpha)
  {
    for (auto &c : colours)
    {
      for (int i = 0; i < 4; ++i)
      {
        if (offsets[i] != miniply::kInvalidIndex)
        {
          const auto &property = element->properties[offsets[i]];
          switch (property.type)
          {
          case miniply::PLYPropertyType::Char:
            c[i] /= float(std::numeric_limits<char>::max());
            break;
          case miniply::PLYPropertyType::UChar:
            c[i] /= float(std::numeric_limits<unsigned char>::max());
            break;
          case miniply::PLYPropertyType::Short:
            c[i] /= float(std::numeric_limits<short>::max());
            break;
          case miniply::PLYPropertyType::UShort:
            c[i] /= float(std::numeric_limits<unsigned short>::max());
            break;
          case miniply::PLYPropertyType::Int:
            c[i] /= float(std::numeric_limits<int>::max());
            break;
          case miniply::PLYPropertyType::UInt:
            c[i] /= float(std::numeric_limits<unsigned int>::max());
            break;
          }
        }
      }

      if (!have_alpha)
      {
        // Fix alpha to 1
        c[3] = 1.0f;
      }
    }
  }
}
}  // namespace

namespace slamio
{
PointCloudReaderMiniPly::PointCloudReaderMiniPly() = default;
PointCloudReaderMiniPly::~PointCloudReaderMiniPly() = default;

DataChannel PointCloudReaderMiniPly::availableChannels() const
{
  return available_channels_;
}

DataChannel PointCloudReaderMiniPly::desiredChannels() const
{
  return desired_channels_;
}

void PointCloudReaderMiniPly::setDesiredChannels(DataChannel channels)
{
  desired_channels_ = channels;
}

bool PointCloudReaderMiniPly::isOpen()
{
  return reader_ != nullptr;
}

bool PointCloudReaderMiniPly::open(const char *filename)
{
  close();
  reader_ = std::make_unique<miniply::PLYReader>(filename);

  if (!reader_->valid())
  {
    close();
    return false;
  }

  // Resolve the point count.
  const miniply::PLYElement *vertex_element = nullptr;
  for (uint32_t i = 0, end_i = reader_->num_elements(); i < end_i; ++i)
  {
    const miniply::PLYElement *elem = reader_->get_element(i);
    if (elem->name == miniply::kPLYVertexElement)
    {
      vertex_element = elem;
      point_count_ = elem->count;
      break;
    }
  }

  // Missing vertices.
  if (!vertex_element)
  {
    close();
    return false;
  }

  // Resolve available fields.
  if (checkAnyAvailable(*vertex_element, time_fields, &time_field_name_))
  {
    available_channels_ |= DataChannel::Time;
  }

  if (checkAllAvailable(*vertex_element, { "x", "y", "z" }))
  {
    available_channels_ |= DataChannel::Position;
  }
  if (checkAllAvailable(*vertex_element, { "nx", "ny", "nz" }))
  {
    available_channels_ |= DataChannel::Normals;
  }
  if (checkAllAvailable(*vertex_element, { "r", "g", "b" }) ||
      checkAllAvailable(*vertex_element, { "red", "green", "blue" }))
  {
    available_channels_ |= DataChannel::ColourRgb;
  }

  if (checkAnyAvailable(*vertex_element, { "a", "alpha" }, &alpha_field_name_))
  {
    available_channels_ |= DataChannel::ColourAlpha;
  }

  if (checkAnyAvailable(*vertex_element, { "intensity" }, nullptr))
  {
    available_channels_ |= DataChannel::Intensity;
  }

  if (desired_channels_ == DataChannel::None)
  {
    desired_channels_ = available_channels_;
  }

  readSamples();

  return true;
}

void PointCloudReaderMiniPly::close()
{
  reader_ = nullptr;
  samples_.release();
  point_count_ = 0;
  available_channels_ = DataChannel::None;
  desired_channels_ = DataChannel::None;
}

bool PointCloudReaderMiniPly::streaming() const
{
  return false;
}

uint64_t PointCloudReaderMiniPly::pointCount() const
{
  return point_count_;
}

bool PointCloudReaderMiniPly::readNext(CloudPoint &point)
{
  if (read_count_ < point_count_)
  {
    point.position = (!samples_.positions.empty()) ? samples_.positions[samples_cursor_] : glm::dvec3(0);
    point.normal = (!samples_.normals.empty()) ? samples_.normals[samples_cursor_] : glm::dvec3(0);
    point.colour = (!samples_.colours.empty()) ? samples_.colours[samples_cursor_] : glm::vec4(0);
    point.timestamp = (!samples_.timestamps.empty()) ? samples_.timestamps[samples_cursor_] : 0.0;
    point.intensity = (!samples_.intensities.empty()) ? samples_.intensities[samples_cursor_] : 0.0f;
    ++samples_cursor_;
    ++read_count_;

    // Release memory very 1M points. This is to try reduce the overhead as we read all data on start.
    // While miniply is very computationally efficient, the read constraint is memory inefficient with respect to the
    // API being created here.
    // TODO(KS): Investigate modifying miniply to support selective load (progressive probably won't work).
    const uint64_t consume_at = 1000000;
    if (samples_cursor_ >= consume_at)
    {
      samples_.consume(consume_at);
      samples_cursor_ -= consume_at;
    }
    else if (samples_cursor_ == point_count_)
    {
      // Reading done. Release remaining memory.
      samples_.release();
    }

    return true;
  }

  return false;
}

uint64_t PointCloudReaderMiniPly::readChunk(CloudPoint *point, uint64_t count)
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

void PointCloudReaderMiniPly::readSamples()
{
  samples_.release();

  if (point_count_)
  {
    bool have_vertices = false;
    while (reader_->has_element() && !have_vertices)
    {
      if (reader_->element_is(miniply::kPLYVertexElement) && reader_->load_element())
      {
        uint32_t offsets[4] = {};
        if ((DataChannel::Time | (available_channels_ & desired_channels_)) != DataChannel::None)
        {
          // Read positions.
          offsets[0] = reader_->find_property(time_field_name_.c_str());
          if (offsets[0] != miniply::kInvalidIndex)
          {
            samples_.timestamps.resize(point_count_);
            reader_->extract_properties_with_stride(offsets, 1, miniply::PLYPropertyType::Double,
                                                    samples_.timestamps.data(), sizeof(*samples_.timestamps.data()));
          }
        }
        if ((DataChannel::Position | (available_channels_ & desired_channels_)) != DataChannel::None)
        {
          // Read positions.
          if (reader_->find_pos(offsets))
          {
            samples_.positions.resize(point_count_);
            reader_->extract_properties_with_stride(offsets, 3, miniply::PLYPropertyType::Double,
                                                    samples_.positions.data(), sizeof(*samples_.positions.data()));
          }
        }
        if ((DataChannel::Normals | (available_channels_ & desired_channels_)) != DataChannel::None)
        {
          // Read positions.
          if (reader_->find_normal(offsets))
          {
            samples_.normals.resize(point_count_);
            reader_->extract_properties_with_stride(offsets, 3, miniply::PLYPropertyType::Double,
                                                    samples_.normals.data(), sizeof(*samples_.normals.data()));
          }
        }
        if ((DataChannel::Intensity | (available_channels_ & desired_channels_)) != DataChannel::None)
        {
          // Read positions.
          offsets[0] = reader_->find_property("intensity");
          if (offsets[0] != miniply::kInvalidIndex)
          {
            samples_.intensities.resize(point_count_);
            reader_->extract_properties_with_stride(offsets, 1, miniply::PLYPropertyType::Float,
                                                    samples_.intensities.data(), sizeof(*samples_.intensities.data()));
          }
        }
        if ((DataChannel::Colour | (available_channels_ & desired_channels_)) != DataChannel::None)
        {
          // Read positions.
          int offset_count = 0;
          size_t write_offset = 0;

          if ((DataChannel::ColourRgb | (available_channels_ & desired_channels_)) != DataChannel::None)
          {
            // Will read colour.
            if (reader_->find_color(offsets))
            {
              offset_count = 3;
            }
          }

          if ((DataChannel::ColourAlpha | (available_channels_ & desired_channels_)) != DataChannel::None)
          {
            // Will read alpha
            if (offset_count)
            {
              // Only reading alpha.
              write_offset = sizeof(glm::vec4) - sizeof(float);
            }
            offsets[offset_count] = reader_->find_property(alpha_field_name_.c_str());
            ++offset_count;
          }
          if (offsets[0] != miniply::kInvalidIndex)
          {
            samples_.colours.resize(point_count_);
            reader_->extract_properties_with_stride(offsets, offset_count, miniply::PLYPropertyType::Float,
                                                    samples_.colours.data(), sizeof(*samples_.colours.data()));
            fixColourRange(samples_.colours, *reader_, alpha_field_name_);
          }
        }
        have_vertices = true;
      }
    }
  }
}

template <typename T>
void consumeData(std::vector<T> &vector, uint64_t count)
{
  if (count < vector.size())
  {
    const size_t new_size = vector.size() - count;
    memmove(vector.data(), vector.data() + count, sizeof(*vector.data()) * new_size);
    vector.resize(new_size);
  }
  else
  {
    vector.resize(0);
  }
  vector.shrink_to_fit();
}

void PointCloudReaderMiniPly::Samples::consume(uint64_t count)
{
  consumeData(positions, count);
  consumeData(normals, count);
  consumeData(colours, count);
  consumeData(timestamps, count);
  consumeData(intensities, count);
}
}  // namespace slamio
