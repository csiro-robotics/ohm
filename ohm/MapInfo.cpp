//
// @author Kazys Stepanas
//
// Copyright (c) 2015 CSIRO
//
#include "MapInfo.h"

#include <cstring>
#include <sstream>
#include <string>
#include <unordered_set>

#ifndef _MSC_VER
#include <strings.h>
#define _stricmp strcasecmp
#endif // _MSC_VER

using namespace ohm;

namespace std
{
  template <>
  struct hash<ohm::MapValue>
  {
    size_t operator()(const ohm::MapValue &value) const
    {
      return std::hash<std::string>()(*static_cast<const std::string *>(value.namePtr()));
    }
  };
}

namespace ohm
{
  struct TreeSearchPredicate
  {
    bool operator()(const MapValue &left, const MapValue &right) const
    {
      return left.namesEqual(right);
    }
  };


  struct MapInfoDetail
  {
    std::unordered_set<MapValue, std::hash<MapValue>, TreeSearchPredicate> dictionary;
  };
}

namespace
{
  template <typename T>
  class StringConvert
  {
  public:
    static T convert(const char *str)
    {
      if (!str)
      {
        return T(0);
      }

      std::istringstream s(str);
      T val = T(0);
      s >> val;
      return val;
    }
  };


  template <>
  class StringConvert < const char * >
  {
  public:
    static const char *convert(const char *str) { return str; }
  };


  template <>
  class StringConvert < bool >
  {
  public:
    static bool convert(const char *str)
    {
      if (!str)
      {
        return false;
      }

      if (_stricmp("true", str) == 0 || _stricmp("yes", str) == 0 || _stricmp("on", str) == 0)
      {
        return true;
      }
      if (_stricmp("false", str) == 0 || _stricmp("no", str) == 0 || _stricmp("off", str) == 0)
      {
        return true;
      }

      return StringConvert<float>::convert(str) != 0;
    }
  };
}

MapValue::MapValue()
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
}


MapValue::MapValue(const MapValue &other)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  *this = other;
}


MapValue::MapValue(MapValue &&other)
  : name_(other.name_)
  , type_(other.type_)
{
  memcpy(&value_, &other.value_, sizeof(value_));
  other.type_ = kTypeNone;
  other.name_ = nullptr;
  memset(&other.value_, 0, sizeof(other.value_));
}


MapValue::MapValue(const char *name, int8_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, uint8_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, int16_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, uint16_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, int32_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, uint32_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, int64_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, uint64_t val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, float val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, double val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, bool val)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = val;
}


MapValue::MapValue(const char *name, const char *string)
  : name_(nullptr)
  , type_(kTypeNone)
{
  memset(&value_, 0, sizeof(value_));
  setName(name);
  *this = string;
}


MapValue::~MapValue()
{
  clear();
  delete static_cast<std::string *>(name_);
}


void MapValue::clear()
{
  clearValue();
  if (name_)
  {
    *static_cast<std::string *>(name_) = std::string();
  }
}


void MapValue::clearValue()
{
  if (type_ == kString)
  {
    delete[] value_.str;
  }
  type_ = kTypeNone;
  value_.u64 = 0ull;
}


const char *MapValue::name() const
{
  if (name_)
  {
    return static_cast<std::string *>(name_)->c_str();
  }

  return "";
}


void MapValue::setName(const char *name)
{
  if (!name_)
  {
    name_ = new std::string();
  }
  *static_cast<std::string *>(name_) = name;
}


bool MapValue::namesEqual(const MapValue &other) const
{
  const std::string *a = static_cast<const std::string *>(name_);
  const std::string *b = static_cast<const std::string *>(other.name_);

  if (!a && (!b || b->empty()) || !b && (!a || a->empty()))
  {
    // Both names empty/null.
    return true;
  }

  if (a && b)
  {
    return *a == *b;
  }

  return false;
}


MapValue::operator int8_t() const
{
  return value<int8_t>();
}


MapValue::operator uint8_t() const
{
  return value<uint8_t>();
}


MapValue::operator int16_t() const
{
  return value<int16_t>();
}


MapValue::operator uint16_t() const
{
  return value<uint16_t>();
}


MapValue::operator int32_t() const
{
  return value<int32_t>();
}


MapValue::operator uint32_t() const
{
  return value<uint32_t>();
}


MapValue::operator int64_t() const
{
  return value<int64_t>();
}


MapValue::operator uint64_t() const
{
  return value<uint64_t>();
}


MapValue::operator float() const
{
  return value<float>();
}


MapValue::operator double() const
{
  return value<double>();
}


MapValue::operator bool() const
{
  return value<bool>();
}


MapValue::operator const char *() const
{
  if (type_ == kString)
  {
    return value_.str;
  }
  return "";
}


MapValue MapValue::toStringValue() const
{
  if (type_ == kString)
  {
    return *this;
  }

  std::stringstream str;

  switch (type_)
  {
  case kInt8:
    str << int(value_.i8);
    break;
  case kUInt8:
    str << int(value_.u8);
    break;
  case kInt16:
    str << value_.i16;
    break;
  case kUInt16:
    str << value_.u16;
    break;
  case kInt32:
    str << value_.i32;
    break;
  case kUInt32:
    str << value_.u32;
    break;
  case kInt64:
    str << value_.i64;
    break;
  case kUInt64:
    str << value_.u64;
    break;
  case kFloat32:
    str << value_.f32;
    break;
  case kFloat64:
    str << value_.f64;
    break;
  case kBoolean:
    str << (value_.b ? "true" : "false");
    break;
  default:
    break;
  }

  return MapValue(name(), str.str().c_str());
}


MapValue &MapValue::operator = (const MapValue &other)
{
  clearValue();
  type_ = other.type_;
  if (type_ != kString)
  {
    value_.u64 = other.value_.u64;
  }
  else
  {
    value_.str = nullptr;
    size_t len = other.value_.str ? strlen(other.value_.str) : 0;
    value_.str = new char[len + 1];
    value_.str[0] = '\0';
    if (other.value_.str)
    {
#ifdef _MSC_VER
      strncpy_s(value_.str, len + 1, other.value_.str, len);
#else  // !_MSC_VER
      strncpy(value_.str, other.value_.str, len);
#endif // _MSC_VER
      value_.str[len] = '\0';
    }
  }
  setName(other.name());

  return *this;
}


MapValue &MapValue::operator = (int8_t val)
{
  clearValue();
  type_ = kInt8;
  value_.i8 = val;
  return *this;
}


MapValue &MapValue::operator = (uint8_t val)
{
  clearValue();
  type_ = kUInt8;
  value_.u8 = val;
  return *this;
}


MapValue &MapValue::operator = (int16_t val)
{
  clearValue();
  type_ = kInt16;
  value_.i16 = val;
  return *this;
}


MapValue &MapValue::operator = (uint16_t val)
{
  clearValue();
  type_ = kUInt16;
  value_.u16 = val;
  return *this;
}


MapValue &MapValue::operator = (int32_t val)
{
  clearValue();
  type_ = kInt32;
  value_.i32 = val;
  return *this;
}


MapValue &MapValue::operator = (uint32_t val)
{
  clearValue();
  type_ = kUInt32;
  value_.u32 = val;
  return *this;
}


MapValue &MapValue::operator = (int64_t val)
{
  clearValue();
  type_ = kInt64;
  value_.i64 = val;
  return *this;
}


MapValue &MapValue::operator = (uint64_t val)
{
  clearValue();
  type_ = kUInt64;
  value_.u64 = val;
  return *this;
}


MapValue &MapValue::operator = (float val)
{
  clearValue();
  type_ = kFloat32;
  value_.f32 = val;
  return *this;
}


MapValue &MapValue::operator = (double val)
{
  clearValue();
  type_ = kFloat64;
  value_.f64 = val;
  return *this;
}


MapValue &MapValue::operator = (bool val)
{
  clearValue();
  type_ = kBoolean;
  value_.b = val;
  return *this;
}


MapValue &MapValue::operator = (const char *string)
{
  clearValue();
  type_ = kString;
  value_.str = nullptr;
  size_t len = string ? strlen(string) : 0;
  value_.str = new char[len + 1];
  value_.str[0] = '\0';
  if (string)
  {
#ifdef _MSC_VER
    strncpy_s(value_.str, len + 1, string, len);
#else  // _MSC_VER
    strncpy(value_.str, string, len);
#endif // _MSC_VER
    value_.str[len] = '\0';
  }
  return *this;
}


bool MapValue::operator == (const MapValue &other) const
{
  if (type_ != other.type_)
  {
    return false;
  }

  const std::string *n1 = static_cast<const std::string *>(name_);
  const std::string *n2 = static_cast<const std::string *>(other.name_);

  if (!(
        n1 == nullptr && n2 == nullptr ||
        n1 == nullptr && n2->empty() ||
        n1->empty() && n2 == nullptr ||
        n1 == n2
      ))
  {
    return false;
  }

  switch (type_)
  {
  case kInt8:
    return value_.i8 == other.value_.i8;
  case kUInt8:
    return value_.u8 == other.value_.u8;
  case kInt16:
    return value_.i16 == other.value_.i16;
  case kUInt16:
    return value_.u16 == other.value_.u16;
  case kInt32:
    return value_.i32 == other.value_.i32;
  case kUInt32:
    return value_.u32 == other.value_.u32;
  case kInt64:
    return value_.i64 == other.value_.i64;
  case kUInt64:
    return value_.u64 == other.value_.u64;
  case kFloat32:
    return value_.f32 == other.value_.f32;
  case kFloat64:
    return value_.f64 == other.value_.f64;
  case kBoolean:
    return value_.b == other.value_.b;
  case kString:
    return value_.str == nullptr && other.value_.str == nullptr ||
           value_.str && other.value_.str && strcmp(value_.str, other.value_.str) == 0;

  default:
    break;
  }

  return false;
}


bool MapValue::operator != (const MapValue &other) const
{
  return !(*this == other);
}


#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4800)
#endif // _MSC_VER
template <typename T>
T MapValue::value() const
{
  switch (type_)
  {
  case kInt8:
    return T(value_.i8);
  case kUInt8:
    return T(value_.u8);
  case kInt16:
    return T(value_.i16);
  case kUInt16:
    return T(value_.u16);
  case kInt32:
    return T(value_.i32);
  case kUInt32:
    return T(value_.u32);
  case kInt64:
    return T(value_.i64);
  case kUInt64:
    return T(value_.u64);
  case kFloat32:
    return T(value_.f32);
  case kFloat64:
    return T(value_.f64);
  case kBoolean:
    return value_.b ? T(1) : T(0);
  case kString:
    return StringConvert<T>::convert(value_.str);
  default:
    break;
  }

  return T(0);
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER


bool MapValue::operator < (const MapValue &other) const
{
  return *static_cast<const std::string *>(name_) < *static_cast<const std::string *>(other.name_);
}


bool MapValue::sortCompare(const MapValue &a, const MapValue &b)
{
  return *static_cast<std::string *>(a.name_) < *static_cast<std::string *>(b.name_);
}


MapInfo::MapInfo()
  : imp_(new MapInfoDetail)
{
}


MapInfo::MapInfo(const MapInfo &other)
  : imp_(new MapInfoDetail)
{
  for (auto iter = other.imp_->dictionary.begin(); iter != other.imp_->dictionary.end(); ++iter)
  {
    imp_->dictionary.insert(*iter);
  }
}


MapInfo::MapInfo(MapInfo &&other)
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


MapInfo::~MapInfo()
{
  delete imp_;
}


void MapInfo::set(const MapValue &value)
{
  auto iter = imp_->dictionary.find(value);
  if (iter != imp_->dictionary.end())
  {
    imp_->dictionary.erase(iter);
  }
  imp_->dictionary.insert(value);
}


MapValue MapInfo::get(const char *name) const
{
  auto iter = imp_->dictionary.find(MapValue(name, int32_t(0)));
  if (iter != imp_->dictionary.end())
  {
    return *iter;
  }

  return MapValue();
}


bool MapInfo::remove(const char *name)
{
  auto iter = imp_->dictionary.find(MapValue(name, int32_t(0)));
  if (iter != imp_->dictionary.end())
  {
    imp_->dictionary.erase(iter);
    return true;
  }
  return false;
}


void MapInfo::clear()
{
  if (imp_)
  {
    imp_->dictionary.clear();
  }
}


unsigned MapInfo::extract(MapValue *values, unsigned maxCount, unsigned offset) const
{
  if (!values || !maxCount)
  {
    return unsigned(imp_->dictionary.size());
  }

  unsigned extracted = 0;
  for (auto iter = imp_->dictionary.begin(); iter != imp_->dictionary.end() && extracted < maxCount; ++iter)
  {
    if (!offset)
    {
      values[extracted++] = *iter;
    }
    else
    {
      --offset;
    }
  }

  return extracted;
}
