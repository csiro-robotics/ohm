// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PlyPointStream.h"

#include <glm/vec3.hpp>

#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace ohm
{
namespace
{
template <typename T>
struct PlyTypeOf
{
};

#define PLYTYPEOF(type_decl, enum_value)                 \
  template <>                                            \
  struct PlyTypeOf<type_decl>                            \
  {                                                      \
    static const PlyPointStream::Type type = enum_value; \
  }

PLYTYPEOF(int8_t, PlyPointStream::Type::kInt8);
PLYTYPEOF(uint8_t, PlyPointStream::Type::kUInt8);
PLYTYPEOF(int16_t, PlyPointStream::Type::kInt16);
PLYTYPEOF(uint16_t, PlyPointStream::Type::kUInt16);
PLYTYPEOF(int32_t, PlyPointStream::Type::kInt32);
PLYTYPEOF(uint32_t, PlyPointStream::Type::kUInt32);
PLYTYPEOF(float, PlyPointStream::Type::kFloat32);
PLYTYPEOF(double, PlyPointStream::Type::kFloat64);


bool isBigEndian()
{
  union
  {
    uint32_t i;
    char c[sizeof(uint32_t)];  // NOLINT(modernize-avoid-c-arrays)
  } bint = { 0x01020304 };     // NOLINT(readability-magic-numbers)

  return bint.c[0] == 1;
}


/// Helper for use with writing a PLY element count to an @c ostream .
///
/// Usage:
/// ```cpp
/// std::cout << "element vertex " << ElementCount(point_count) << '\n';
/// ```
///
/// This will prefix the output value with '0' to maximum width of the @c ElementCount::value . This can be used
/// to write a place holder value at the start of the stream, then to return and write the correct value.
class ElementCount
{
public:
  /// Constructor
  /// @param count The value for the element count.
  explicit ElementCount(uint64_t count = 0)
    : value(count)
  {}

  uint64_t value = 0;  ///< The element count value to write.
};


/// Streaming operator for an @c ElementCount .
///
/// This writes the @c ElementCount::value padded with leading zeros and can be used to support writing a placeholder
/// value, then later returning the stream to write the corrected value.
/// @param out The output stream to write.
/// @param count The @c ElementCount to write.
/// @return @c out
std::ostream &operator<<(std::ostream &out, const ElementCount &count)
{
  auto restore_fill = out.fill();
  auto restore_width = out.width();
  out << std::setfill('0') << std::setw(std::numeric_limits<decltype(count.value)>::digits10) << count.value;
  out.fill(restore_fill);
  out.width(restore_width);
  return out;
}
}  // namespace

PlyPointStream::PlyPointStream(const std::vector<Property> &properties)
{
  properties_ = properties;
  values_.resize(properties_.size());
}


PlyPointStream::PlyPointStream(const std::vector<Property> &properties, std::ostream &out)
{
  properties_ = properties;
  values_.resize(properties_.size());
  open(out);
}


PlyPointStream::~PlyPointStream()
{
  close();
}


void PlyPointStream::open(std::ostream &out)
{
  close();
  out_ = &out;
  point_count_pos_ = -1;
  point_count_pos_ = 0;
  writeHeader();
}


bool PlyPointStream::close()
{
  bool ok = false;
  if (isOpen())
  {
    ok = finalisePointCount();
    out_ = nullptr;
  }
  return ok;
}


bool PlyPointStream::isOpen() const
{
  return out_ != nullptr;
}


bool PlyPointStream::setPointPosition(const glm::dvec3 &pos)
{
  return setProperty("x", pos.x) && setProperty("y", pos.y) && setProperty("z", pos.z);
}


bool PlyPointStream::setPointTimestamp(double time)
{
  return setProperty("time", time);
}


bool PlyPointStream::setPointNormal(const glm::dvec3 &normal)
{
  return setProperty("nx", normal.x) && setProperty("ny", normal.y) && setProperty("nz", normal.z);
}


bool PlyPointStream::setProperty(const std::string &name, int8_t value)
{
  return setPropertyT(name, value);
}


bool PlyPointStream::setProperty(const std::string &name, uint8_t value)
{
  return setPropertyT(name, value);
}


bool PlyPointStream::setProperty(const std::string &name, int16_t value)
{
  return setPropertyT(name, value);
}


bool PlyPointStream::setProperty(const std::string &name, uint16_t value)
{
  return setPropertyT(name, value);
}


bool PlyPointStream::setProperty(const std::string &name, int32_t value)
{
  return setPropertyT(name, value);
}


bool PlyPointStream::setProperty(const std::string &name, uint32_t value)
{
  return setPropertyT(name, value);
}


bool PlyPointStream::setProperty(const std::string &name, float value)
{
  return setPropertyT(name, value);
}


bool PlyPointStream::setProperty(const std::string &name, double value)
{
  return setPropertyT(name, value);
}


void PlyPointStream::writePoint()
{
  for (size_t i = 0; i < properties_.size(); ++i)
  {
    const Value &value = values_[i];
    switch (properties_[i].type)
    {
    case Type::kInt8:
      out_->write(reinterpret_cast<const char *>(&value.i8), sizeof(value.i8));
      break;
    case Type::kUInt8:
      out_->write(reinterpret_cast<const char *>(&value.u8), sizeof(value.u8));
      break;
    case Type::kInt16:
      out_->write(reinterpret_cast<const char *>(&value.i16), sizeof(value.i16));
      break;
    case Type::kUInt16:
      out_->write(reinterpret_cast<const char *>(&value.u16), sizeof(value.u16));
      break;
    case Type::kInt32:
      out_->write(reinterpret_cast<const char *>(&value.i32), sizeof(value.i32));
      break;
    case Type::kUInt32:
      out_->write(reinterpret_cast<const char *>(&value.u32), sizeof(value.u32));
      break;
    case Type::kFloat32:
      out_->write(reinterpret_cast<const char *>(&value.f32), sizeof(value.f32));
      break;
    case Type::kFloat64:
      out_->write(reinterpret_cast<const char *>(&value.f64), sizeof(value.f64));
      break;
    default:
      throw std::runtime_error("Unexpected data type");
    }
  }
  ++point_count_;
}


std::string PlyPointStream::typeName(Type type)
{
  // Lint(KS): tried using an enum value for the array size, but that didn't work.
  // NOLINTNEXTLINE(readability-magic-numbers)
  static std::array<const std::string, 9> names =  //
    {
      "null",    //
      "char",    //
      "uchar",   //
      "short",   //
      "ushort",  //
      "int",     //
      "uint",    //
      "float",   //
      "double"   //
    };

  if (unsigned(type) < names.size())
  {
    return names[unsigned(type)];
  }

  return std::string();
}


template <typename T>
bool PlyPointStream::setPropertyT(const std::string &name, T value)
{
  for (size_t i = 0; i < properties_.size(); ++i)
  {
    if (name == properties_[i].name)
    {
      if (PlyTypeOf<T>::type == properties_[i].type)
      {
        setValue(&values_[i], value);
        return true;
      }
      return false;
    }
  }

  return false;
}


void PlyPointStream::setValue(Value *dst, int8_t value)
{
  dst->i8 = value;
}


void PlyPointStream::setValue(Value *dst, uint8_t value)
{
  dst->u8 = value;
}


void PlyPointStream::setValue(Value *dst, int16_t value)
{
  dst->i16 = value;
}


void PlyPointStream::setValue(Value *dst, uint16_t value)
{
  dst->u16 = value;
}


void PlyPointStream::setValue(Value *dst, int32_t value)
{
  dst->i32 = value;
}


void PlyPointStream::setValue(Value *dst, uint32_t value)
{
  dst->u32 = value;
}


void PlyPointStream::setValue(Value *dst, float value)
{
  dst->f32 = value;
}


void PlyPointStream::setValue(Value *dst, double value)
{
  dst->f64 = value;
}


void PlyPointStream::writeHeader()
{
  std::ostream &out = *out_;
  /// Write the ply header fields.
  out << "ply\n";
  if (isBigEndian())
  {
    out << "format binary_big_endian 1.0\n";
  }
  else
  {
    out << "format binary_little_endian 1.0\n";
  }
  out << "comment Generated by ohmutil\n";
  // Write the initial point count as 0, but we record where to patch later. We add general padding plus enough for
  // the padding comment.
  out << "element vertex " << std::flush;
  point_count_pos_ = out.tellp();
  out << ElementCount() << '\n';
  // Write property details.
  for (const auto &property : properties_)
  {
    out << "property " << typeName(property.type) << " " << property.name << "\n";
  }

  out << "end_header" << std::endl;
}


bool PlyPointStream::finalisePointCount()
{
  if (out_ && point_count_pos_ >= 0)
  {
    auto cur_pos = out_->tellp();
    out_->flush();
    out_->seekp(point_count_pos_);
    if (out_->tellp() != point_count_pos_)
    {
      return false;
    }
    *out_ << ElementCount(point_count_) << '\n';
    // Seek back to where we were before finalising the point count.
    out_->seekp(cur_pos);
    return true;
  }
  return false;
}
}  // namespace ohm
