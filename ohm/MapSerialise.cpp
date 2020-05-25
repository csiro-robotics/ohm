//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "MapSerialise.h"

#include "DefaultLayer.h"
#include "Heightmap.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Stream.h"
#include "VoxelLayout.h"

#include "private/HeightmapDetail.h"
#include "private/OccupancyMapDetail.h"
#include "private/SerialiseUtil.h"

#include "serialise/MapSerialiseV0.1.h"
#include "serialise/MapSerialiseV0.2.h"
#include "serialise/MapSerialiseV0.4.h"
#include "serialise/MapSerialiseV0.h"

#include <glm/glm.hpp>

#include <cstdio>
#include <cstring>
#include <functional>
#include <type_traits>
#include <vector>

#define OM_ZIP 1

#if OM_ZIP
#include <zlib.h>
#endif  // OM_ZIP

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
  const MapVersion kSupportedVersionMax = { 0, 4, 0 };
  const MapVersion kCurrentVersion = { 0, 4, 0 };

  // Note: version 0.3.x is not supported.

  int saveItem(OutputStream &stream, const MapValue &value)  // NOLINT(google-runtime-references)
  {
    //{
    //  MapValue strValue = value.toStringValue();
    //  fprintf(stderr, "w: %s = %s\n", value.name(), static_cast<const char *>(strValue));
    //}
    const char *name = value.name();
    size_t len = strlen(name);
    if (len > 0xffffu)
    {
      return kSeDataItemTooLarge;
    }

    uint16_t len16 = uint16_t(len);
    // if (endianSwap)
    // {
    //   endian::endianSwap(&len16);
    // }
    stream.write(reinterpret_cast<char *>(&len16), sizeof(len16));

    if (len)
    {
      stream.write(name, unsigned(len));
    }

    uint8_t type = value.type();
    stream.write(reinterpret_cast<char *>(&type), 1);

    switch (value.type())
    {
    case MapValue::kInt8:
    {
      int8_t val = value;
      stream.write(reinterpret_cast<char *>(&val), 1);
      break;
    }
    case MapValue::kUInt8:
    {
      uint8_t val = value;
      stream.write(reinterpret_cast<char *>(&val), 1);
      break;
    }
    case MapValue::kInt16:
    {
      int16_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kUInt16:
    {
      uint16_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kInt32:
    {
      int32_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kUInt32:
    {
      uint32_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kInt64:
    {
      int64_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kUInt64:
    {
      uint64_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kFloat32:
    {
      float val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kFloat64:
    {
      double val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write(reinterpret_cast<char *>(&val), sizeof(val));
      break;
    }
    case MapValue::kBoolean:
    {
      bool bval = value;
      uint8_t val = (bval) ? 1 : 0;
      stream.write(reinterpret_cast<char *>(&val), 1);
      break;
    }
    case MapValue::kString:
    {
      const char *str = value;
      len = strlen(str);
      if (len > 0xffffu)
      {
        return kSeDataItemTooLarge;
      }

      len16 = uint16_t(len);
      // if (endianSwap)
      // {
      //   endian::endianSwap(&len16);
      // }
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


  int saveMapInfo(OutputStream &stream, const MapInfo &map_info)  // NOLINT(google-runtime-references)
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


  int saveHeader(OutputStream &stream, const OccupancyMapDetail &map)  // NOLINT(google-runtime-references)
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

    // Added v0.3.0
    // Saving the map stamp has become important to ensure MapChunk::touched_stamps are correctly maintained.
    ok = writeUncompressed<uint64_t>(stream, map.stamp) && ok;

    // Add v0.3.2
    ok = writeUncompressed<uint32_t>(stream, std::underlying_type_t<ohm::MapFlag>(map.flags)) && ok;

    return (ok) ? 0 : kSeFileWriteFailure;
  }


  int saveLayout(OutputStream &stream, const OccupancyMapDetail &map)  // NOLINT(google-runtime-references)
  {
    // Save details about the map layers.
    const MapLayout &layout = map.layout;
    const uint32_t layer_count = uint32_t(layout.layerCount());
    bool ok = true;

    ok = write<int32_t>(stream, layer_count) && ok;

    for (unsigned i = 0; i < layer_count; ++i)
    {
      const MapLayer &layer = layout.layer(i);
      // Write the layer name.
      uint32_t val32 = uint32_t(strlen(layer.name()));
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


  int saveChunk(OutputStream &stream,  // NOLINT(google-runtime-references)
                const MapChunk &chunk, const OccupancyMapDetail &detail)
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
    const MapLayout &layout = *chunk.layout;
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
      const uint8_t *layer_mem = layer.voxels(chunk);
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


  int loadHeader(InputStream &stream, HeaderVersion &version,    // NOLINT(google-runtime-references)
                 OccupancyMapDetail &map, size_t &region_count)  // NOLINT(google-runtime-references)
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
        return kSeUnsupportedVersion;
      }

      ok = readRaw<double>(stream, map.origin.x) && ok;
    }
    else
    {
      // No marker. Assume version zero and migrate to map.origin.x
      uint8_t buffer[sizeof(double)];
      memcpy(buffer, &version.marker, sizeof(version.marker));
      memcpy(buffer + sizeof(version.marker), &version.version.major, sizeof(version.version.major));
      map.origin.x = *reinterpret_cast<double *>(buffer);

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

    if (version.version.major > 0 || version.version.major == 0 && version.version.minor > 2)
    {
      // Read the map stamp.
      ok = readRaw<uint64_t>(stream, map.stamp) && ok;
    }

    // v0.3.2 added serialisation of map flags
    if (version.version.major > 0 || version.version.minor > 3 || version.version.patch > 1)
    {
      uint32_t flags = 0;
      ok = readRaw<std::underlying_type_t<ohm::MapFlag>>(stream, map.flags) && ok;
      map.flags = static_cast<ohm::MapFlag>(flags);
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
  int loadChunk(InputStream &stream, MapChunk &chunk,  // NOLINT(google-runtime-references)
                const OccupancyMapDetail &detail)
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

        if (layer.flags() & MapLayer::kSkipSerialise)
        {
          // Not to be serialised. Clear instead.
          layer.clear(layer.voxels(chunk), detail.region_voxel_dimensions);
          continue;
        }

        // Get the layer memory.
        uint8_t *layer_mem = layer.voxels(chunk);
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
}  // namespace ohm


const char *ohm::errorCodeString(int err)
{
  static const char *names[] =  //
    {                           //
      "ok",
      "file create failure",
      "file open failure",
      "write failure",
      "read failure",
      "value overflow",
      "member offset error",
      "info error",
      "heightmap info mismatch",
      "data item too large",
      "unknown data type",
      "unsupported version"
    };

  if (err < 0 || unsigned(err) > sizeof(names) / sizeof(names[0]))
  {
    return "<unknown>";
  }

  return names[err];
}


int ohm::save(const char *filename, const OccupancyMap &map, SerialiseProgress *progress)
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


int ohm::load(const char *filename, OccupancyMap &map, SerialiseProgress *progress, MapVersion *version_out)
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
    err = ohm::v0::load(stream, detail, progress, version.version, region_count);
  }
  else if (kSupportedVersionMin <= version.version && version.version <= kSupportedVersionMax)
  {
    if (version.version.major == 0 && version.version.minor == 1)
    {
      err = ohm::v0_1::load(stream, detail, progress, version.version, region_count);
    }
    else if (version.version.major == 0 && version.version.minor == 2)
    {
      err = ohm::v0_2::load(stream, detail, progress, version.version, region_count);
    }
    else if (version.version.major == 0 && version.version.minor == 4)
    {
      err = ohm::v0_4::load(stream, detail, progress, version.version, region_count);
    }
  }

  return err;
}


int ohm::load(const char *filename, Heightmap &heightmap, SerialiseProgress *progress, MapVersion *version_out)
{
  HeightmapDetail &detail = *heightmap.detail();

  int err = load(filename, *detail.heightmap, progress, version_out);
  if (err)
  {
    return err;
  }

  // TODO(KS): Set axis from map info.
  const MapInfo &info = detail.heightmap->mapInfo();
  if (!bool(info.get("heightmap")))
  {
    return kSeHeightmapInfoMismatch;
  }

  detail.fromMapInfo(info);

  return err;
}


int ohm::loadHeader(const char *filename, OccupancyMap &map, MapVersion *version_out, size_t *region_count)
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
      detail.setDefaultLayout(false);
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
