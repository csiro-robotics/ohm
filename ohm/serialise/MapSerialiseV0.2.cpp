// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapSerialiseV0.2.h"

#include "private/OccupancyMapDetail.h"
#include "private/SerialiseUtil.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapSerialise.h"
#include "Stream.h"

namespace ohm
{
namespace v0_2
{
int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
         size_t region_count)
{
  return load(stream, detail, progress, version, region_count, &v0_1::loadChunk);
}

int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
         size_t region_count, const ChunkFunc &load_chunk)
{
  // From version 0.2 we have MapInfo.
  int err = loadMapInfo(stream, detail.info);
  if (err)
  {
    return err;
  }

  return v0_1::load(stream, detail, progress, version, region_count, load_chunk);
}


int loadMapInfo(InputStream &in, MapInfo &info)  //, const bool endianSwap)
{
  uint32_t item_count = 0;
  info.clear();

  if (!readRaw<uint32_t>(in, item_count))
  {
    return kSeInfoError;
  }

  if (!item_count)
  {
    return kSeOk;
  }

  int err = 0;
  for (unsigned i = 0; i < item_count; ++i)
  {
    MapValue value;
    err = loadItem(in, value);  //, endianSwap);
    if (err != kSeOk)
    {
      return err;
    }
    info.set(value);
  }

  return kSeOk;
}


int loadItem(InputStream &in, MapValue &value)  //, const bool endianSwap)
{
  uint16_t len16;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  in.read(reinterpret_cast<char *>(&len16), sizeof(len16));
  // if (endianSwap)
  // {
  //   endian::endianSwap(&len16);
  // }

  char *str = new char[len16 + 1];
  if (len16)
  {
    in.read(str, len16);
  }
  str[len16] = '\0';
  value.setName(str);
  delete[] str;

  uint8_t type;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  in.read(reinterpret_cast<char *>(&type), 1);

  switch (type)
  {
  case MapValue::kInt8: {
    int8_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), 1);
    value = val;
    break;
  }
  case MapValue::kUInt8: {
    uint8_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), 1);
    value = val;
    break;
  }
  case MapValue::kInt16: {
    int16_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kUInt16: {
    uint16_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kInt32: {
    int32_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kUInt32: {
    uint32_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kInt64: {
    int64_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kUInt64: {
    uint64_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kFloat32: {
    float val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kFloat64: {
    double val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), sizeof(val));
    // if (endianSwap) { endian::endianSwap(&val); }
    value = val;
    break;
  }
  case MapValue::kBoolean: {
    uint8_t val;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&val), 1);
    if (val)
    {
      value = true;
    }
    else
    {
      value = false;
    }
    break;
  }
  case MapValue::kString: {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    in.read(reinterpret_cast<char *>(&len16), sizeof(len16));
    // if (endianSwap)
    // {
    //   endian::endianSwap(&len16);
    // }

    char *str = new char[len16 + 1];
    if (len16)
    {
      in.read(str, len16);
    }
    str[len16] = '\0';
    value = str;
    delete[] str;
    break;
  }

  default:
    return kSeUnknownDataType;
  }

  //{
  //  MapValue strValue = value.toStringValue();
  //  fprintf(stderr, "r: %s = %s\n", value.name(), static_cast<const char *>(strValue));
  //}
  return kSeOk;
}
}  // namespace v0_2
}  // namespace ohm