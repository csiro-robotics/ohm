// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapSerialise.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapFlag.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Stream.h"
#include "VoxelBlock.h"
#include "VoxelBuffer.h"
#include "VoxelLayout.h"

#include "private/OccupancyMapDetail.h"
#include "private/SerialiseUtil.h"

#include "serialise/MapSerialiseV0.1.h"
#include "serialise/MapSerialiseV0.2.h"
#include "serialise/MapSerialiseV0.4.h"
#include "serialise/MapSerialiseV0.5.h"
#include "serialise/MapSerialiseV0.h"

#include <glm/glm.hpp>

#include <array>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include <zlib.h>

namespace
{
inline std::pair<int, std::string> makeErrorCode(ohm::SerialisationError code, const std::string &str)
{
  return std::make_pair(int(code), str);
}

std::mutex s_error_code_lock;
std::map<int, std::string> s_error_codes = { makeErrorCode(ohm::kSeOk, "ok"),
                                             makeErrorCode(ohm::kSeFileCreateFailure, "file create failure"),
                                             makeErrorCode(ohm::kSeFileOpenFailure, "file open failure"),
                                             makeErrorCode(ohm::kSeFileWriteFailure, "write failure"),
                                             makeErrorCode(ohm::kSeFileReadFailure, "read failure"),
                                             makeErrorCode(ohm::kSeValueOverflow, "value overflow"),
                                             makeErrorCode(ohm::kSeMemberOffsetError, "member offset error"),
                                             makeErrorCode(ohm::kSeInfoError, "info error"),
                                             makeErrorCode(ohm::kSeDataItemTooLarge, "data item too large"),
                                             makeErrorCode(ohm::kSeUnknownDataType, "unknown data type"),
                                             makeErrorCode(ohm::kSeUnsupportedVersion, "unsupported version"),
                                             makeErrorCode(ohm::kSeDeprecatedVersion, "deprecated version"),
                                             makeErrorCode(ohm::kSeExtensionCode, "unknown extension error") };
}  // namespace

namespace ohm
{
/// Contains the header marker and version information for an occupancy map.
/// The header should equal @c MapHeaderMarker, while the version may vary.
/// Version zero did not store any header marker or version number.
///
/// The version number is arranged as follows (in base 10) @c vvvMMMPPP where:
/// - @c vvv is the major version number (any number)
/// - @c MMM is a three digit specification of the current minor version.
/// - @c PPP is a three digit specification of the current patch version.
struct HeaderVersion
{
  /// Marker equal to @c MapHeaderMarker if valid.
  uint32_t marker = 0;
  /// Map format version number.
  MapVersion version = { 0, 0, 0 };
};

const uint32_t kMapHeaderMarker = 0x44330011u;

// Digits are arranged as follows:
//    vvvMMMPPP
// where:
// - vvv is the major version number (any number)
// - MMM is a three digit specification of the current minor version.
// - PPP is a three digit specification of the current patch version.
const MapVersion kSupportedVersionMin = { 0, 0, 0 };
const MapVersion kSupportedVersionMax = { 0, 5, 0 };
const MapVersion kCurrentVersion = { 0, 5, 0 };

// Note: version 0.3.x is not supported.

int saveItem(OutputStream &stream, const MapValue &value)
{
  //{
  //  MapValue strValue = value.toStringValue();
  //  fprintf(stderr, "w: %s = %s\n", value.name(), static_cast<const char *>(strValue));
  //}
  const char *name = value.name();
  size_t len = strlen(name);
  if (len > std::numeric_limits<uint16_t>::max())
  {
    return kSeDataItemTooLarge;
  }

  auto len16 = uint16_t(len);
  // if (endianSwap)
  // {
  //   endian::endianSwap(&len16);
  // }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.write(reinterpret_cast<char *>(&len16), sizeof(len16));

  if (len)
  {
    stream.write(name, unsigned(len));
  }

  uint8_t type = value.type();
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.write(reinterpret_cast<char *>(&type), 1);

  switch (value.type())
  {
  case MapValue::kInt8:
  {
    auto val = static_cast<int8_t>(value);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), 1);
    break;
  }
  case MapValue::kUInt8:
  {
    auto val = static_cast<uint8_t>(value);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), 1);
    break;
  }
  case MapValue::kInt16:
  {
    auto val = static_cast<int16_t>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kUInt16:
  {
    auto val = static_cast<uint16_t>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kInt32:
  {
    auto val = static_cast<int32_t>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kUInt32:
  {
    auto val = static_cast<uint32_t>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kInt64:
  {
    auto val = static_cast<int64_t>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kUInt64:
  {
    auto val = static_cast<uint64_t>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kFloat32:
  {
    auto val = static_cast<float>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kFloat64:
  {
    auto val = static_cast<double>(value);
    // if (endianSwap) { endian::endianSwap(&val); }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), sizeof(val));
    break;
  }
  case MapValue::kBoolean:
  {
    bool bval = static_cast<bool>(value);
    uint8_t val = (bval) ? 1 : 0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&val), 1);
    break;
  }
  case MapValue::kString:
  {
    const char *str = static_cast<const char *>(value);
    len = strlen(str);
    if (len > std::numeric_limits<uint16_t>::max())
    {
      return kSeDataItemTooLarge;
    }

    len16 = uint16_t(len);
    // if (endianSwap)
    // {
    //   endian::endianSwap(&len16);
    // }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.write(reinterpret_cast<char *>(&len16), sizeof(len16));

    if (unsigned(len))
    {
      stream.write(str, unsigned(len));
    }
    break;
  }

  default:
    return kSeDataItemTooLarge;
  }

  return kSeOk;
}


int saveMapInfo(OutputStream &stream, const MapInfo &map_info)
{
  uint32_t item_count = map_info.extract(nullptr, 0);

  bool ok = true;
  ok = writeUncompressed<uint32_t>(stream, item_count) && ok;

  if (!ok)
  {
    return kSeInfoError;
  }

  if (item_count == 0)
  {
    return kSeOk;
  }

  std::vector<MapValue> values(item_count);
  values.resize(item_count);
  unsigned extracted = map_info.extract(values.data(), item_count);

  if (extracted != item_count)
  {
    return kSeInfoError;
  }

  int err = 0;
  for (unsigned i = 0; i < extracted; ++i)
  {
    err = saveItem(stream, values[i]);
    if (err != kSeOk)
    {
      return err;
    }
  }

  return kSeOk;
}


int saveHeader(OutputStream &stream, const OccupancyMapDetail &map)
{
  bool ok = true;
  // Header marker + version
  HeaderVersion version;
  version.marker = kMapHeaderMarker;
  version.version = kCurrentVersion;

  ok = writeUncompressed<uint32_t>(stream, version.marker) && ok;
  ok = writeUncompressed<uint32_t>(stream, version.version.major) && ok;
  ok = writeUncompressed<uint16_t>(stream, version.version.minor) && ok;
  ok = writeUncompressed<uint16_t>(stream, version.version.patch) && ok;

  ok = writeUncompressed<double>(stream, map.origin.x) && ok;
  ok = writeUncompressed<double>(stream, map.origin.y) && ok;
  ok = writeUncompressed<double>(stream, map.origin.z) && ok;
  ok = writeUncompressed<double>(stream, map.region_spatial_dimensions.x) && ok;
  ok = writeUncompressed<double>(stream, map.region_spatial_dimensions.y) && ok;
  ok = writeUncompressed<double>(stream, map.region_spatial_dimensions.z) && ok;
  ok = writeUncompressed<int32_t>(stream, map.region_voxel_dimensions.x) && ok;
  ok = writeUncompressed<int32_t>(stream, map.region_voxel_dimensions.y) && ok;
  ok = writeUncompressed<int32_t>(stream, map.region_voxel_dimensions.z) && ok;
  ok = writeUncompressed<double>(stream, map.resolution) && ok;
  ok = writeUncompressed<double>(stream, map.occupancy_threshold_value) && ok;
  ok = writeUncompressed<double>(stream, map.hit_value) && ok;
  ok = writeUncompressed<double>(stream, map.miss_value) && ok;
  ok = writeUncompressed<uint32_t>(stream, map.chunks.size()) && ok;

  // Added v0.5.0
  ok = writeUncompressed<double>(stream, map.first_ray_time) && ok;

  // Added v0.3.0
  // Saving the map stamp has become important to ensure MapChunk::touched_stamps are correctly maintained.
  ok = writeUncompressed<uint64_t>(stream, map.stamp) && ok;

  // Add v0.3.2
  ok = writeUncompressed<uint32_t>(stream, std::underlying_type_t<MapFlag>(map.flags)) && ok;

  return (ok) ? 0 : kSeFileWriteFailure;
}


int saveLayout(OutputStream &stream, const OccupancyMapDetail &map)
{
  // Save details about the map layers.
  const MapLayout &layout = map.layout;
  const auto layer_count = uint32_t(layout.layerCount());
  bool ok = true;

  ok = write<int32_t>(stream, layer_count) && ok;

  for (unsigned i = 0; i < layer_count; ++i)
  {
    const MapLayer &layer = layout.layer(i);
    // Write the layer name.
    auto val32 = uint32_t(strlen(layer.name()));
    ok = write<uint32_t>(stream, val32) && ok;
    ok = stream.write(layer.name(), val32) == val32 && ok;

    // Write flags.
    val32 = layer.flags();
    ok = write<uint32_t>(stream, val32) && ok;

    // Write the subsampling
    ok = write<uint16_t>(stream, uint16_t(layer.subsampling())) && ok;

    // Write voxel layout.
    VoxelLayoutConst voxel_layout = layer.voxelLayout();
    // Write voxel size.
    val32 = uint32_t(voxel_layout.voxelByteSize());
    ok = write<uint32_t>(stream, val32) && ok;
    // Write member count.
    val32 = uint32_t(voxel_layout.memberCount());
    ok = write<uint32_t>(stream, val32) && ok;
    for (size_t j = 0; j < voxel_layout.memberCount(); ++j)
    {
      // Write member name.
      val32 = uint32_t(strlen(voxel_layout.memberName(j)));
      ok = write<uint32_t>(stream, val32) && ok;
      ok = stream.write(voxel_layout.memberName(j), val32) == val32 && ok;
      // Member type.

      // Write member type and offset.
      uint16_t val16;
      val16 = uint16_t(voxel_layout.memberType(j));
      ok = write<uint16_t>(stream, val16) && ok;
      val16 = uint16_t(voxel_layout.memberOffset(j));
      ok = write<uint16_t>(stream, val16) && ok;

      // Write clear value.
      uint64_t clear_value = voxel_layout.memberClearValue(j);
      ok = write<uint64_t>(stream, clear_value) && ok;
    }
  }

  return (ok) ? 0 : kSeFileWriteFailure;
}


int saveChunk(OutputStream &stream, const MapChunk &chunk, const OccupancyMapDetail &detail)
{
  bool ok = true;

  // Write region details, then nodes. MapChunk members are derived.
  ok = write<int32_t>(stream, chunk.region.coord.x) && ok;
  ok = write<int32_t>(stream, chunk.region.coord.y) && ok;
  ok = write<int32_t>(stream, chunk.region.coord.z) && ok;
  ok = write<double>(stream, chunk.region.centre.x) && ok;
  ok = write<double>(stream, chunk.region.centre.y) && ok;
  ok = write<double>(stream, chunk.region.centre.z) && ok;
  ok = write<double>(stream, chunk.touched_time) && ok;

  // Save each map layer.
  const MapLayout &layout = chunk.layout();
  for (size_t i = 0; i < layout.layerCount(); ++i)
  {
    const MapLayer &layer = layout.layer(i);

    if (layer.flags() & MapLayer::kSkipSerialise)
    {
      // Not to be serialised.
      continue;
    }

    uint64_t layer_touched_stamp = chunk.touched_stamps[i];
    ok = write<uint64_t>(stream, layer_touched_stamp) && ok;

    // Get the layer memory.
    VoxelBuffer<const VoxelBlock> voxel_buffer(chunk.voxel_blocks[layer.layerIndex()]);
    const uint8_t *layer_mem = voxel_buffer.voxelMemory();
    const size_t node_count = layer.volume(detail.region_voxel_dimensions);
    const size_t node_byte_count = layer.voxelByteSize() * node_count;
    if (node_byte_count != unsigned(node_byte_count))
    {
      return kSeValueOverflow;
    }

    ok = stream.write(layer_mem, unsigned(node_byte_count)) == node_byte_count && ok;
  }

  return (ok) ? 0 : kSeFileWriteFailure;
}


int loadHeader(InputStream &stream, HeaderVersion &version, OccupancyMapDetail &map, size_t &region_count)
{
  bool ok = true;

  // Try for marker and version number. Was not present in the original code.
  static_assert(sizeof(version.marker) + sizeof(version.version.major) == sizeof(map.origin.x),
                "Assuming we can migrate to double if no marker present");
  ok = readRaw<uint32_t>(stream, version.marker) && ok;
  ok = readRaw<uint32_t>(stream, version.version.major) && ok;

  if (version.marker == kMapHeaderMarker)
  {
    ok = readRaw<uint16_t>(stream, version.version.minor) && ok;
    ok = readRaw<uint16_t>(stream, version.version.patch) && ok;

    if (!ok)
    {
      return kSeFileReadFailure;
    }

    // Have a header marker and version number. Check version.
    if (version.version < kSupportedVersionMin || version.version > kSupportedVersionMax)
    {
      return kSeUnsupportedVersion;
    }

    if (version.version.major == 0 && version.version.minor == 3)
    {
      // Version 0.3.x not supported. That introduced voxel mean positioning using a progressive weighting.
      // Support in 0.4.0 changed to separate VoxelMean layer using a progressive point count yielding much better
      // coordinates.
      return kSeDeprecatedVersion;
    }

    ok = readRaw<double>(stream, map.origin.x) && ok;
  }
  else
  {
    // No marker. Assume version zero and migrate to map.origin.x
    std::array<uint8_t, sizeof(double)> buffer;
    static_assert(sizeof(buffer) >= sizeof(version.marker) + sizeof(version.version.major),
                  "Read ahead buffer too small.");
    memcpy(buffer.data(), &version.marker, sizeof(version.marker));
    memcpy(buffer.data() + sizeof(version.marker), &version.version.major, sizeof(version.version.major));
    memcpy(&map.origin.x, buffer.data(), sizeof(map.origin.x));

    version.marker = 0;
    version.version = { 0, 0, 0 };
  }

  ok = readRaw<double>(stream, map.origin.y) && ok;
  ok = readRaw<double>(stream, map.origin.z) && ok;
  ok = readRaw<double>(stream, map.region_spatial_dimensions.x) && ok;
  ok = readRaw<double>(stream, map.region_spatial_dimensions.y) && ok;
  ok = readRaw<double>(stream, map.region_spatial_dimensions.z) && ok;
  ok = readRaw<int32_t>(stream, map.region_voxel_dimensions.x) && ok;
  ok = readRaw<int32_t>(stream, map.region_voxel_dimensions.y) && ok;
  ok = readRaw<int32_t>(stream, map.region_voxel_dimensions.z) && ok;
  ok = readRaw<double>(stream, map.resolution) && ok;
  ok = readRaw<double>(stream, map.occupancy_threshold_value) && ok;
  ok = readRaw<double>(stream, map.hit_value) && ok;
  ok = readRaw<double>(stream, map.miss_value) && ok;
  region_count = 0;
  ok = readRaw<uint32_t>(stream, region_count) && ok;
  map.loaded_region_count = region_count;

  if (version.version.major > 0 || version.version.major == 0 && version.version.minor >= 5)
  {
    // Read the map first ray time stamp.
    ok = readRaw<double>(stream, map.first_ray_time) && ok;
  }

  if (version.version.major > 0 || version.version.major == 0 && version.version.minor >= 3)
  {
    // Read the map stamp.
    ok = readRaw<uint64_t>(stream, map.stamp) && ok;
  }

  // v0.3.2 added serialisation of map flags
  if (version.version.major > 0 || version.version.minor > 3 ||
      version.version.minor == 3 && version.version.patch >= 2)
  {
    uint32_t flags = 0;
    ok = readRaw<std::underlying_type_t<MapFlag>>(stream, map.flags) && ok;
    map.flags = static_cast<MapFlag>(flags);
  }
  else
  {
    map.flags = MapFlag::kNone;
  }

  if (!ok)
  {
    return kSeFileReadFailure;
  }

  return kSeOk;
}


// Current version of chunk loading.
int loadChunk(InputStream &stream, MapChunk &chunk, const OccupancyMapDetail &detail)
{
  bool ok = true;

  // Write region details, then nodes. MapChunk members are derived.
  ok = read<int32_t>(stream, chunk.region.coord.x) && ok;
  ok = read<int32_t>(stream, chunk.region.coord.y) && ok;
  ok = read<int32_t>(stream, chunk.region.coord.z) && ok;
  ok = read<double>(stream, chunk.region.centre.x) && ok;
  ok = read<double>(stream, chunk.region.centre.y) && ok;
  ok = read<double>(stream, chunk.region.centre.z) && ok;
  ok = read<double>(stream, chunk.touched_time) && ok;

  if (ok)
  {
    const MapLayout &layout = detail.layout;
    for (size_t i = 0; i < layout.layerCount(); ++i)
    {
      const MapLayer &layer = layout.layer(i);
      VoxelBuffer<VoxelBlock> voxel_buffer(chunk.voxel_blocks[i]);
      uint8_t *layer_mem = voxel_buffer.voxelMemory();

      if (layer.flags() & MapLayer::kSkipSerialise)
      {
        // Not to be serialised. Clear instead.
        layer.clear(layer_mem, detail.region_voxel_dimensions);
        continue;
      }

      // Get the layer memory.
      const size_t node_count = layer.volume(detail.region_voxel_dimensions);
      const size_t node_byte_count = layer.voxelByteSize() * node_count;
      if (node_byte_count != unsigned(node_byte_count))
      {
        return kSeValueOverflow;
      }

      ok = stream.read(layer_mem, unsigned(node_byte_count)) == node_byte_count && ok;
    }
  }

  return (ok) ? 0 : kSeFileReadFailure;
}


const char *serialiseErrorCodeString(int err)
{
  std::unique_lock<std::mutex> guard(s_error_code_lock);
  const auto lookup = s_error_codes.find(err);
  if (lookup != s_error_codes.end())
  {
    return lookup->second.c_str();
  }
  return "<unknown>";
}


void registerSerialiseExtensionErrorCodeString(int err, const char *error_string)
{
  std::unique_lock<std::mutex> guard(s_error_code_lock);
  s_error_codes.insert(std::make_pair(err, std::string(error_string)));
}


int save(const std::string &filename, const OccupancyMap &map, SerialiseProgress *progress)
{
  OutputStream stream(filename, kSfCompress);
  const OccupancyMapDetail &detail = *map.detail();

  if (!stream.isOpen())
  {
    return kSeFileCreateFailure;
  }

  if (progress)
  {
    progress->setTargetProgress(unsigned(detail.chunks.size()));
  }

  // Header is written uncompressed.
  int err = saveHeader(stream, detail);

  if (err)
  {
    return err;
  }

  // Save the MapInfo
  err = saveMapInfo(stream, detail.info);

  if (err)
  {
    return err;
  }

  err = saveLayout(stream, detail);

  if (err)
  {
    return err;
  }

  for (auto region_iter = detail.chunks.begin(); region_iter != detail.chunks.end() && (!progress || !progress->quit());
       ++region_iter)
  {
    err = saveChunk(stream, *region_iter->second, detail);
    if (err)
    {
      return err;
    }

    if (progress)
    {
      progress->incrementProgress();
    }
  }

  return kSeOk;
}


int load(const std::string &filename, OccupancyMap &map, SerialiseProgress *progress, MapVersion *version_out)
{
  InputStream stream(filename, kSfCompress);
  OccupancyMapDetail &detail = *map.detail();

  if (!stream.isOpen())
  {
    return kSeFileOpenFailure;
  }

  map.clear();

  // Header is read uncompressed.
  size_t region_count = 0;
  HeaderVersion version;
  int err = loadHeader(stream, version, detail, region_count);
  if (version_out)
  {
    *version_out = version.version;
  }

  if (err)
  {
    return err;
  }

  err = kSeUnsupportedVersion;
  if (version.marker == 0 || version.version.major == 0 && version.version.minor == 0)
  {
    err = v0::load(stream, detail, progress, version.version, region_count);
  }
  else if (kSupportedVersionMin <= version.version && version.version <= kSupportedVersionMax)
  {
    if (version.version.major == 0 && version.version.minor == 1)
    {
      err = v0_1::load(stream, detail, progress, version.version, region_count);
    }
    else if (version.version.major == 0 && version.version.minor == 2)
    {
      err = v0_2::load(stream, detail, progress, version.version, region_count);
    }
    else if (version.version.major == 0 && version.version.minor == 4)
    {
      err = v0_4::load(stream, detail, progress, version.version, region_count);
    }
    else if (version.version.major == 0 && version.version.minor == 5)
    {
      err = v0_5::load(stream, detail, progress, version.version, region_count);
    }
  }

  return err;
}


int loadHeader(const std::string &filename, OccupancyMap &map, MapVersion *version_out, size_t *region_count)
{
  InputStream stream(filename, kSfCompress);
  OccupancyMapDetail &detail = *map.detail();

  if (!stream.isOpen())
  {
    return kSeFileOpenFailure;
  }

  map.clear();

  // Header is read uncompressed.
  size_t region_count_local = 0;
  HeaderVersion version;
  int err = loadHeader(stream, version, detail, region_count_local);
  if (version_out)
  {
    *version_out = version.version;
  }

  // From version 0.2 we have MapInfo.
  detail.info.clear();
  if (version.version.major > 0 || version.version.minor > 1)
  {
    if (!err)
    {
      err = v0_2::loadMapInfo(stream, detail.info);
    }
  }

  if (region_count)
  {
    *region_count = region_count_local;
  }

  if (!err)
  {
    if (version.version.major == 0 && version.version.minor == 0 && version.version.patch == 0)
    {
      // Version 0.0.0 had no layout.
      detail.setDefaultLayout(MapFlag::kNone);
    }
    else
    {
      err = v0_1::loadLayout(stream, detail);
    }
  }

  // Correct the voxel mean flags. The flags may not match the reality of the map layout, such as when we load an older
  // version.
  if (!err)
  {
    if (detail.layout.meanLayer() >= 0)
    {
      detail.flags |= MapFlag::kVoxelMean;
    }
    else
    {
      detail.flags &= ~MapFlag::kVoxelMean;
    }
  }

  return err;
}

}  // namespace ohm
