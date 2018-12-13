//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "MapSerialise.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "Heightmap.h"
#include "OccupancyMap.h"
#include "Stream.h"
#include "VoxelLayout.h"

#include "private/HeightmapDetail.h"
#include "private/OccupancyMapDetail.h"

#include <glm/glm.hpp>

#include <cstdio>
#include <functional>
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
  const MapVersion kSupportedVersionMin = { 0, 0, 0 };
  const MapVersion kSupportedVersionMax = { 0, 2, 0 };

  // Digits are arranged as follows:
  //    vvvMMMPPP
  // where:
  // - vvv is the major version number (any number)
  // - MMM is a three digit specification of the current minor version.
  // - PPP is a three digit specification of the current patch version.
  const MapVersion kCurrentVersion = { 0, 2, 0 };

  /// Explicitly typed stream writing, uncompressed.
  template <typename T, typename S>
  inline bool writeUncompressed(OutputStream &stream, const S &val)
  {
    const T val2 = static_cast<T>(val);
    return stream.writeUncompressed(&val2, unsigned(sizeof(val2))) == sizeof(val2);
  }


  /// Explicitly typed stream writing.
  template <typename T, typename S>
  inline bool write(OutputStream &stream, const S &val)
  {
    const T val2 = static_cast<T>(val);
    return stream.write(&val2, unsigned(sizeof(val2))) == sizeof(val2);
  }


  /// Explicitly typed stream reading, uncompressed.
  template <typename T, typename S>
  inline bool readRaw(InputStream &stream, S &val)
  {
    T val2{ 0 };
    if (stream.readRaw(&val2, unsigned(sizeof(val2))) != sizeof(val2))
    {
      return false;
    }
    val = static_cast<S>(val2);
    return true;
  }


  /// Explicitly typed stream reading.
  template <typename T, typename S>
  inline bool read(InputStream &stream, S &val)
  {
    T val2{ 0 };
    if (stream.read(&val2, unsigned(sizeof(val2))) != sizeof(val2))
    {
      return false;
    }
    val = static_cast<S>(val2);
    return true;
  }


  int saveItem(OutputStream &stream, const MapValue &value)
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
    stream.write((char *)&len16, sizeof(len16));

    if (len)
    {
      stream.write(name, unsigned(len));
    }

    uint8_t type = value.type();
    stream.write((char *)&type, 1);

    switch (value.type())
    {
    case MapValue::kInt8:
    {
      int8_t val = value;
      stream.write((char *)&val, 1);
      break;
    }
    case MapValue::kUInt8:
    {
      uint8_t val = value;
      stream.write((char *)&val, 1);
      break;
    }
    case MapValue::kInt16:
    {
      int16_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kUInt16:
    {
      uint16_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kInt32:
    {
      int32_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kUInt32:
    {
      uint32_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kInt64:
    {
      int64_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kUInt64:
    {
      uint64_t val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kFloat32:
    {
      float val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kFloat64:
    {
      double val = value;
      // if (endianSwap) { endian::endianSwap(&val); }
      stream.write((char *)&val, sizeof(val));
      break;
    }
    case MapValue::kBoolean:
    {
      bool bval = value;
      uint8_t val = (bval) ? 1 : 0;
      stream.write((char *)&val, 1);
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
      stream.write((char *)&len16, sizeof(len16));

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


  int saveMapInfo(OutputStream &stream, const MapInfo &mapInfo)
  {
    uint32_t item_count = mapInfo.extract(nullptr, 0);

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
    unsigned extracted = mapInfo.extract(values.data(), item_count);

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


  int loadItem(InputStream &in, MapValue &value)//, const bool endianSwap)
  {
    uint16_t len16;
    in.read((char *)&len16, sizeof(len16));
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
    in.read((char *)&type, 1);

    switch (type)
    {
    case MapValue::kInt8:
    {
      int8_t val;
      in.read((char *)&val, 1);
      value = val;
      break;
    }
    case MapValue::kUInt8:
    {
      uint8_t val;
      in.read((char *)&val, 1);
      value = val;
      break;
    }
    case MapValue::kInt16:
    {
      int16_t val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kUInt16:
    {
      uint16_t val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kInt32:
    {
      int32_t val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kUInt32:
    {
      uint32_t val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kInt64:
    {
      int64_t val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kUInt64:
    {
      uint64_t val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kFloat32:
    {
      float val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kFloat64:
    {
      double val;
      in.read((char *)&val, sizeof(val));
      // if (endianSwap) { endian::endianSwap(&val); }
      value = val;
      break;
    }
    case MapValue::kBoolean:
    {
      uint8_t val;
      in.read((char *)&val, 1);
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
    case MapValue::kString:
    {
      in.read((char *)&len16, sizeof(len16));
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


  int loadMapInfo(InputStream &in, MapInfo &info)//, const bool endianSwap)
  {
    uint32_t item_count = 0;
    info.clear();

    if (!readRaw<uint32_t>(in, item_count))
    {
      return kSeHeightmapInfoMismatch;
    }

    if (!item_count)
    {
      return kSeOk;
    }

    int err = 0;
    for (unsigned i = 0; i < item_count; ++i)
    {
      MapValue value;
      err = loadItem(in, value);//, endianSwap);
      if (err != kSeOk)
      {
        return err;
      }
      info.set(value);
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

    return (ok) ? 0 : kSeFileWriteFailure;
  }


  int saveLayout(OutputStream &stream, const OccupancyMapDetail &map)
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
    const MapLayout &layout = *chunk.layout;
    for (size_t i = 0; i < layout.layerCount(); ++i)
    {
      const MapLayer &layer = layout.layer(i);
      if (layer.flags() & MapLayer::kSkipSerialise)
      {
        // Not to be serialised.
        continue;
      }

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

    if (!ok)
    {
      return kSeFileReadFailure;
    }

    return kSeOk;
  }


  int loadLayout(InputStream &stream, OccupancyMapDetail &map)
  {
    // Save details about the map layers.
    MapLayout &layout = map.layout;
    bool ok = true;
    uint32_t layer_count = 0;
    std::vector<char> layer_name, member_name;
    uint32_t len;
    uint32_t layer_flags;
    uint16_t subsampling;

    layout.clear();

    ok = read<int32_t>(stream, layer_count) && ok;

    for (unsigned i = 0; ok && i < layer_count; ++i)
    {
      // Read the layer name.
      len = 0;
      ok = read<uint32_t>(stream, len) && ok;

      layer_name.resize(len + 1);
      ok = stream.read(layer_name.data(), len) == len && ok;
      layer_name[len] = '\0';

      // Read flags.
      layer_flags = 0;
      ok = read<uint32_t>(stream, layer_flags) && ok;

      // Read the subsampling
      subsampling = 0;
      ok = read<uint16_t>(stream, subsampling) && ok;

      MapLayer *layer = layout.addLayer(layer_name.data(), subsampling);

      // Read voxel layout.
      VoxelLayout voxel_layout = layer->voxelLayout();
      // Read voxel size.
      uint32_t voxel_size = 0;
      ok = read<uint32_t>(stream, voxel_size) && ok;
      // Read member count.
      uint32_t voxel_member_count = 0;
      ok = read<uint32_t>(stream, voxel_member_count) && ok;
      for (size_t j = 0; j < voxel_member_count; ++j)
      {
        // Read member name.
        len = 0;
        ok = read<uint32_t>(stream, len) && ok;

        member_name.resize(len + 1);
        ok = stream.read(member_name.data(), len) == len && ok;
        member_name[len] = '\0';

        // Read member type, offset and clear value.
        uint16_t type = 0, offset;
        uint64_t clear_value;
        ok = read<uint16_t>(stream, type) && ok;
        ok = read<uint16_t>(stream, offset) && ok;
        ok = read<uint64_t>(stream, clear_value) && ok;

        // Add the data member.
        if (ok && type)
        {
          voxel_layout.addMember(member_name.data(), DataType::Type(type), clear_value);
          if (voxel_layout.memberOffset(j) != offset)
          {
            ok = false;
            return kSeMemberOffsetError;
          }
        }
      }
    }

    return (ok) ? 0 : kSeFileReadFailure;
  }


  // Version zero chunk loading
  int loadChunkV0(InputStream &stream, MapChunk &chunk, const OccupancyMapDetail &detail)
  {
    bool ok = true;

    const MapLayer *occupancy_layer = chunk.layout->layerPtr(kDlOccupancy);
    const MapLayer *clearance_layer = chunk.layout->layerPtr(kDlClearance);
    const MapLayer *coarse_clearance_layer = chunk.layout->layerPtr(kDlCoarseClearance);

    if (coarse_clearance_layer)
    {
      memset(coarse_clearance_layer->voxels(chunk), 0,
             coarse_clearance_layer->layerByteSize(detail.region_voxel_dimensions));
    }

    // Write region details, then nodes. MapChunk members are derived.
    ok = read<int32_t>(stream, chunk.region.coord.x) && ok;
    ok = read<int32_t>(stream, chunk.region.coord.y) && ok;
    ok = read<int32_t>(stream, chunk.region.coord.z) && ok;
    ok = read<double>(stream, chunk.region.centre.x) && ok;
    ok = read<double>(stream, chunk.region.centre.y) && ok;
    ok = read<double>(stream, chunk.region.centre.z) && ok;
    ok = read<double>(stream, chunk.touched_time) && ok;

    const unsigned node_count =
      detail.region_voxel_dimensions.x * detail.region_voxel_dimensions.y * detail.region_voxel_dimensions.z;
    const size_t node_byte_count = 2 * sizeof(float) * node_count;
    if (node_byte_count != unsigned(node_byte_count))
    {
      return kSeValueOverflow;
    }

    if (ok)
    {
      // Initial version used MapNode which contained two floats.
      // This interleaves occupancy/clearance, so we need to pull them out.
      std::vector<float> node_data(node_count * 2);
      ok = stream.read(node_data.data(), unsigned(node_byte_count)) == node_byte_count && ok;

      float *occupancy = occupancy_layer->voxelsAs<float>(chunk);
      float *clearance = clearance_layer->voxelsAs<float>(chunk);

      for (size_t i = 0; i < node_data.size() / 2; ++i)
      {
        occupancy[i] = node_data[(i << 1) + 0];
        clearance[i] = node_data[(i << 1) + 1];
      }

      chunk.region.hash = MapRegion::Hash::calculate(chunk.region);
    }

    return (ok) ? 0 : kSeFileReadFailure;
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

      chunk.region.hash = MapRegion::Hash::calculate(chunk.region);
    }

    return (ok) ? 0 : kSeFileReadFailure;
  }
}  // namespace ohm


const char *ohm::errorCodeString(int err)
{
  static const char *names[] =
  {
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
  OutputStream stream(filename, SfCompress);
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
  InputStream stream(filename, SfCompress);
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

  std::function<int(InputStream &, MapChunk &, const OccupancyMapDetail &)> load_chunk_func;

  if (version.marker == 0)
  {
    load_chunk_func = loadChunkV0;
    detail.setDefaultLayout();
  }
  else
  {
    if (version.version.major == 0 && version.version.minor == 0)
    {
      load_chunk_func = loadChunkV0;
    }
    else if (kSupportedVersionMin <= version.version && version.version <= kSupportedVersionMax)
    {
      load_chunk_func = loadChunk;
    }
    else
    {
      return kSeUnsupportedVersion;
    }

    // From 0.2.0 we have map info.
    if (version.version.major > 0 || version.version.minor > 1)
    {
      err = loadMapInfo(stream, detail.info);

      if (err)
      {
        return err;
      }
    }

    err = loadLayout(stream, detail);
    if (err)
    {
      return err;
    }
  }

  if (progress)
  {
    if (region_count)
    {
      progress->setTargetProgress(unsigned(region_count));
    }
    else
    {
      progress->setTargetProgress(unsigned(1));
      progress->incrementProgress();
    }
  }

  MapChunk *chunk = nullptr;
  for (unsigned i = 0; i < region_count && (!progress || !progress->quit()); ++i)
  {
    chunk = new MapChunk(detail.layout, detail.region_voxel_dimensions);
    err = load_chunk_func(stream, *chunk, detail);
    if (err)
    {
      delete chunk;
      return err;
    }

    // Resolve map chunk details.
    chunk->searchAndUpdateFirstValid(detail.region_voxel_dimensions);
    detail.chunks.insert(std::make_pair(chunk->region.hash, chunk));

    if (progress)
    {
      progress->incrementProgress();
    }
  }

  return kSeOk;
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
  InputStream stream(filename, SfCompress);
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
      err = loadMapInfo(stream, detail.info);
    }
  }

  if (region_count)
  {
    *region_count = region_count_local;
  }

  if (!err)
  {
    err = loadLayout(stream, detail);
  }

  return err;
}
