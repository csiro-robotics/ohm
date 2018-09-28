//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "occupancymapserialise.h"

#include "mapchunk.h"
#include "maplayer.h"
#include "maplayout.h"
#include "occupancymap.h"
#include "ohmdefaultlayers.h"
#include "ohmstream.h"
#include "ohmvoxellayout.h"
#include "private/occupancymapdetail.h"

#include <cstdio>
#include <functional>
#include <vector>

#define OM_ZIP 1

#if OM_ZIP
#include <zlib.h>
#endif // OM_ZIP

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

  const uint32_t MapHeaderMarker = 0x44330011u;
  const MapVersion SupportedVersionMin = { 0, 0, 0 };
  const MapVersion SupportedVersionMax = { 0, 1, 0 };

  // Digits are arranged as follows:
  //    vvvMMMPPP
  // where:
  // - vvv is the major version number (any number)
  // - MMM is a three digit specification of the current minor version.
  // - PPP is a three digit specification of the current patch version.
  const MapVersion CurrentVersion = { 0, 1, 0 };

  /// Explicitly typed stream writing, uncompressed.
  template <typename T, typename S>
  inline bool writeUncompressed(OutputStream &stream, const S &val)
  {
    const T tval = static_cast<T>(val);
    return stream.writeUncompressed(&tval, unsigned(sizeof(tval))) == sizeof(tval);
  }


  /// Explicitly typed stream writing.
  template <typename T, typename S>
  inline bool write(OutputStream &stream, const S &val)
  {
    const T tval = static_cast<T>(val);
    return stream.write(&tval, unsigned(sizeof(tval))) == sizeof(tval);
  }


  /// Explicitly typed stream reading, uncompressed.
  template <typename T, typename S>
  inline bool readRaw(InputStream &stream, S &val)
  {
    T tval{0};
    if (stream.readRaw(&tval, unsigned(sizeof(tval))) != sizeof(tval))
    {
      return false;
    }
    val = static_cast<S>(tval);
    return true;
  }


  /// Explicitly typed stream reading.
  template <typename T, typename S>
  inline bool read(InputStream &stream, S &val)
  {
    T tval{ 0 };
    if (stream.read(&tval, unsigned(sizeof(tval))) != sizeof(tval))
    {
      return false;
    }
    val = static_cast<S>(tval);
    return true;
  }


  int saveHeader(OutputStream &stream, const OccupancyMapDetail &map)
  {
    bool ok = true;
    // Header marker + version
    HeaderVersion version;
    version.marker = MapHeaderMarker;
    version.version = CurrentVersion;

    ok = writeUncompressed<uint32_t>(stream, version.marker) && ok;
    ok = writeUncompressed<uint32_t>(stream, version.version.major) && ok;
    ok = writeUncompressed<uint16_t>(stream, version.version.minor) && ok;
    ok = writeUncompressed<uint16_t>(stream, version.version.patch) && ok;

    ok = writeUncompressed<double>(stream, map.origin.x) && ok;
    ok = writeUncompressed<double>(stream, map.origin.y) && ok;
    ok = writeUncompressed<double>(stream, map.origin.z) && ok;
    ok = writeUncompressed<double>(stream, map.regionSpatialDimensions.x) && ok;
    ok = writeUncompressed<double>(stream, map.regionSpatialDimensions.y) && ok;
    ok = writeUncompressed<double>(stream, map.regionSpatialDimensions.z) && ok;
    ok = writeUncompressed<int32_t>(stream, map.regionVoxelDimensions.x) && ok;
    ok = writeUncompressed<int32_t>(stream, map.regionVoxelDimensions.y) && ok;
    ok = writeUncompressed<int32_t>(stream, map.regionVoxelDimensions.z) && ok;
    ok = writeUncompressed<double>(stream, map.resolution) && ok;
    ok = writeUncompressed<double>(stream, map.occupancyThresholdValue) && ok;
    ok = writeUncompressed<double>(stream, map.hitValue) && ok;
    ok = writeUncompressed<double>(stream, map.missValue) && ok;
    ok = writeUncompressed<uint32_t>(stream, map.chunks.size()) && ok;

    return (ok) ? 0 : SE_FileWriteFailure;
  }


  int saveLayout(OutputStream &stream, const OccupancyMapDetail &map)
  {
    // Save details about the map layers.
    const MapLayout &layout = map.layout;
    uint32_t layerCount = uint32_t(layout.layerCount());
    bool ok = true;

    ok = write<int32_t>(stream, layerCount) && ok;

    for (unsigned i = 0; i < layerCount; ++i)
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
      VoxelLayoutConst voxelLayout = layer.voxelLayout();
      // Write voxel size.
      val32 = uint32_t(voxelLayout.voxelByteSize());
      ok = write<uint32_t>(stream, val32) && ok;
      // Write member count.
      val32 = uint32_t(voxelLayout.memberCount());
      ok = write<uint32_t>(stream, val32) && ok;
      for (size_t i = 0; i < voxelLayout.memberCount(); ++ i)
      {
        // Write member name.
        val32 = uint32_t(strlen(voxelLayout.memberName(i)));
        ok = write<uint32_t>(stream, val32) && ok;
        ok = stream.write(voxelLayout.memberName(i), val32) == val32 && ok;
        // Member type.

        // Write member type and offset.
        uint16_t val16;
        val16 = (uint16_t)voxelLayout.memberType(i);
        ok = write<uint16_t>(stream, val16) && ok;
        val16 = (uint16_t)voxelLayout.memberOffset(i);
        ok = write<uint16_t>(stream, val16) && ok;

        // Write clear value.
        uint64_t clearValue = voxelLayout.memberClearValue(i);
        ok = write<uint64_t>(stream, clearValue) && ok;
      }
    }

    return (ok) ? 0 : SE_FileWriteFailure;
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
    ok = write<double>(stream, chunk.touchedTime) && ok;

    // Save each map layer.
    const MapLayout &layout = *chunk.layout;
    for (size_t i = 0; i < layout.layerCount(); ++i)
    {
      const MapLayer &layer = layout.layer(i);
      if (layer.flags() & MapLayer::SkipSerialise)
      {
        // Not to be serialised.
        continue;
      }

      // Get the layer memory.
      const uint8_t *layerMem = layer.voxels(chunk);
      size_t nodeCount = layer.volume(detail.regionVoxelDimensions);
      size_t nodeByteCount = layer.voxelByteSize() * nodeCount;
      if (nodeByteCount != unsigned(nodeByteCount))
      {
        return SE_ValueOverflow;
      }

      ok = stream.write(layerMem, unsigned(nodeByteCount)) == nodeByteCount && ok;
    }

    return (ok) ? 0 : SE_FileWriteFailure;
  }


  int loadHeader(InputStream &stream, HeaderVersion &version, OccupancyMapDetail &map, size_t &regionCount)
  {
    bool ok = true;

    // Try for marker and version number. Was not present in the original code.
    static_assert(sizeof(version.marker) + sizeof(version.version.major) == sizeof(map.origin.x), "Assuming we can migrate to double if no marker present");
    ok = readRaw<uint32_t>(stream, version.marker) && ok;
    ok = readRaw<uint32_t>(stream, version.version.major) && ok;

    if (version.marker == MapHeaderMarker)
    {
      ok = readRaw<uint16_t>(stream, version.version.minor) && ok;
      ok = readRaw<uint16_t>(stream, version.version.patch) && ok;

      if (!ok)
      {
        return SE_FileReadFailure;
      }

      // Have a header marker and version number. Check version.
      if (version.version < SupportedVersionMin || version.version > SupportedVersionMax)
      {
        return SE_UnsupportedVersion;
      }

      ok = readRaw<double>(stream, map.origin.x) && ok;
    }
    else
    {
      // No marker. Assume version zero and migrate to map.origin.x
      uint8_t buffer[sizeof(double)];
      memcpy(buffer, &version.marker, sizeof(version.marker));
      memcpy(buffer + sizeof(version.marker), &version.version.major, sizeof(version.version.major));
      map.origin.x = *(double *)buffer;

      version.marker = 0;
      version.version = { 0, 0, 0 };
    }

    ok = readRaw<double>(stream, map.origin.y) && ok;
    ok = readRaw<double>(stream, map.origin.z) && ok;
    ok = readRaw<double>(stream, map.regionSpatialDimensions.x) && ok;
    ok = readRaw<double>(stream, map.regionSpatialDimensions.y) && ok;
    ok = readRaw<double>(stream, map.regionSpatialDimensions.z) && ok;
    ok = readRaw<int32_t>(stream, map.regionVoxelDimensions.x) && ok;
    ok = readRaw<int32_t>(stream, map.regionVoxelDimensions.y) && ok;
    ok = readRaw<int32_t>(stream, map.regionVoxelDimensions.z) && ok;
    ok = readRaw<double>(stream, map.resolution) && ok;
    ok = readRaw<double>(stream, map.occupancyThresholdValue) && ok;
    ok = readRaw<double>(stream, map.hitValue) && ok;
    ok = readRaw<double>(stream, map.missValue) && ok;
    regionCount = 0;
    ok = readRaw<uint32_t>(stream, regionCount) && ok;

    return (ok) ? 0 : SE_FileReadFailure;
  }


  int loadLayout(InputStream &stream, OccupancyMapDetail &map)
  {
    // Save details about the map layers.
    MapLayout &layout = map.layout;
    bool ok = true;
    uint32_t layerCount = 0;
    std::vector<char> layerName, memberName;
    uint32_t len;
    uint32_t layerFlags;
    uint16_t subsampling;

    layout.clear();

    ok = read<int32_t>(stream, layerCount) && ok;

    for (unsigned i = 0; ok && i < layerCount; ++i)
    {
      // Read the layer name.
      len = 0;
      ok = read<uint32_t>(stream, len) && ok;

      layerName.resize(len + 1);
      ok = stream.read(layerName.data(), len) == len && ok;
      layerName[len] = '\0';

      // Read flags.
      layerFlags = 0;
      ok = read<uint32_t>(stream, layerFlags) && ok;

      // Read the subsampling
      subsampling = 0;
      ok = read<uint16_t>(stream, subsampling) && ok;

      MapLayer *layer = layout.addLayer(layerName.data(), subsampling);

      // Read voxel layout.
      VoxelLayout voxelLayout = layer->voxelLayout();
      // Read voxel size.
      uint32_t voxelSize = 0;
      ok = read<uint32_t>(stream, voxelSize) && ok;
      // Read member count.
      uint32_t voxelMemberCount = 0;
      ok = read<uint32_t>(stream, voxelMemberCount) && ok;
      for (size_t i = 0; i < voxelMemberCount; ++ i)
      {
        // Read member name.
        len = 0;
        ok = read<uint32_t>(stream, len) && ok;

        memberName.resize(len + 1);
        ok = stream.read(memberName.data(), len) == len && ok;
        memberName[len] = '\0';

        // Read member type, offset and clear value.
        uint16_t type, offset;
        uint64_t clearValue;
        ok = read<uint16_t>(stream, type) && ok;
        ok = read<uint16_t>(stream, offset) && ok;
        ok = read<uint64_t>(stream, clearValue) && ok;

        // Add the data member.
        if (ok)
        {
          voxelLayout.addMember(memberName.data(), DataType::Type(type), clearValue);
          if (voxelLayout.memberOffset(i) != offset)
          {
            ok = false;
            return SE_MemberOffsetError;
          }
        }
      }
    }

    return (ok) ? 0 : SE_FileReadFailure;
  }


  // Version zero chunk loading
  int loadChunkV0(InputStream &stream, MapChunk &chunk, const OccupancyMapDetail &detail)
  {
    bool ok = true;

    const MapLayer *occupancyLayer = chunk.layout->layerPtr(DL_Occupancy);
    const MapLayer *clearanceLayer = chunk.layout->layerPtr(DL_Clearance);
    const MapLayer *coarseClearanceLayer = chunk.layout->layerPtr(DL_CoarseClearance);

    if (coarseClearanceLayer)
    {
      memset(coarseClearanceLayer->voxels(chunk), 0, coarseClearanceLayer->layerByteSize(detail.regionVoxelDimensions));
    }

    // Write region details, then nodes. MapChunk members are derived.
    ok = read<int32_t>(stream, chunk.region.coord.x) && ok;
    ok = read<int32_t>(stream, chunk.region.coord.y) && ok;
    ok = read<int32_t>(stream, chunk.region.coord.z) && ok;
    ok = read<double>(stream, chunk.region.centre.x) && ok;
    ok = read<double>(stream, chunk.region.centre.y) && ok;
    ok = read<double>(stream, chunk.region.centre.z) && ok;
    ok = read<double>(stream, chunk.touchedTime) && ok;

    const unsigned nodeCount = detail.regionVoxelDimensions.x * detail.regionVoxelDimensions.y * detail.regionVoxelDimensions.z;
    size_t nodeByteCount = 2 * sizeof(float) * nodeCount;
    if (nodeByteCount != unsigned(nodeByteCount))
    {
      return SE_ValueOverflow;
    }

    if (ok)
    {
      // Initial version used MapNode which contained two floats.
      // This interleaves occupancy/clearance, so we need to pull them out.
      std::vector<float> nodeData(nodeCount * 2);
      ok = stream.read(nodeData.data(), unsigned(nodeByteCount)) == nodeByteCount && ok;

      float *occupancy = occupancyLayer->voxelsAs<float>(chunk);
      float *clearance = clearanceLayer->voxelsAs<float>(chunk);

      for (size_t i = 0; i < nodeData.size() / 2; ++i)
      {
        occupancy[i] = nodeData[(i << 1) + 0];
        clearance[i] = nodeData[(i << 1) + 1];
      }

      chunk.region.hash = MapRegion::Hash::calculate(chunk.region);
    }

    return (ok) ? 0 : SE_FileReadFailure;
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
    ok = read<double>(stream, chunk.touchedTime) && ok;

    if (ok)
    {
      const MapLayout &layout = detail.layout;
      for (size_t i = 0; i < layout.layerCount(); ++i)
      {
        const MapLayer &layer = layout.layer(i);

        if (layer.flags() & MapLayer::SkipSerialise)
        {
          // Not to be serialised. Clear instead.
          layer.clear(layer.voxels(chunk), detail.regionVoxelDimensions);
          continue;
        }

        // Get the layer memory.
        uint8_t *layerMem = layer.voxels(chunk);
        size_t nodeCount = layer.volume(detail.regionVoxelDimensions);
        size_t nodeByteCount = layer.voxelByteSize() * nodeCount;
        if (nodeByteCount != unsigned(nodeByteCount))
        {
          return SE_ValueOverflow;
        }

        ok = stream.read(layerMem, unsigned(nodeByteCount)) == nodeByteCount && ok;
      }

      chunk.region.hash = MapRegion::Hash::calculate(chunk.region);
    }

    return (ok) ? 0 : SE_FileReadFailure;
  }
}


int ohm::save(const char *filename, const OccupancyMap &map, SerialiseProgress *progress)
{
  OutputStream stream(filename, SF_Compress);
  const OccupancyMapDetail &detail = *map.detail();
  int err = 0;

  if (!stream.isOpen())
  {
    return SE_FileCreateFailure;
  }

  if (progress)
  {
    progress->setTargetProgress(unsigned(detail.chunks.size()));
  }

  // Header is written uncompressed.
  err = saveHeader(stream, detail);

  if (err)
  {
    return err;
  }

  err = saveLayout(stream, detail);

  if (err)
  {
    return err;
  }

  for (auto regionIter = detail.chunks.begin(); regionIter != detail.chunks.end() && (!progress || !progress->quit()); ++regionIter)
  {
    err = saveChunk(stream, *regionIter->second, detail);
    if (err)
    {
      return err;
    }

    if (progress)
    {
      progress->incrementProgress();
    }
  }

  return SE_OK;
}


int ohm::load(const char *filename, OccupancyMap &map, SerialiseProgress *progress, MapVersion *versionOut)
{
  InputStream stream(filename, SF_Compress);
  OccupancyMapDetail &detail = *map.detail();
  int err = 0;

  if (!stream.isOpen())
  {
    return SE_FileOpenFailure;
  }

  map.clear();

  // Header is read uncompressed.
  size_t regionCount = 0;
  HeaderVersion version;
  err = loadHeader(stream, version, detail, regionCount);
  if (versionOut)
  {
    *versionOut = version.version;
  }

  if (err)
  {
    return err;
  }

  std::function<int (InputStream &, MapChunk &, const OccupancyMapDetail &)> loadChunkFunc;

  if (version.marker == 0)
  {
    loadChunkFunc = loadChunkV0;
    detail.setDefaultLayout();
  }
  else
  {
    if (version.version.major == 0 && version.version.minor == 0)
    {
      loadChunkFunc = loadChunkV0;
    }
    else if (version.version == CurrentVersion)
    {
      loadChunkFunc = loadChunk;
    }
    else
    {
      return SE_UnsupportedVersion;
    }

    loadLayout(stream, detail);
  }

  if (progress)
  {
    if (regionCount)
    {
      progress->setTargetProgress(unsigned(regionCount));
    }
    else
    {
      progress->setTargetProgress(unsigned(1));
      progress->incrementProgress();
    }
  }

  MapChunk *chunk = nullptr;
  for (unsigned i = 0; i < regionCount && (!progress || !progress->quit()); ++i)
  {
    chunk = new MapChunk(detail.layout, detail.regionVoxelDimensions);
    err = loadChunkFunc(stream, *chunk, detail);
    if (err)
    {
      delete chunk;
      return err;
    }

    // Resolve map chunk details.
    chunk->searchAndUpdateFirstValid(detail.regionVoxelDimensions);
    detail.chunks.insert(std::make_pair(chunk->region.hash, chunk));

    if (progress)
    {
      progress->incrementProgress();
    }
  }

  return SE_OK;
}


int ohm::loadHeader(const char *filename, OccupancyMap &map, MapVersion *versionOut)
{
  InputStream stream(filename, SF_Compress);
  OccupancyMapDetail &detail = *map.detail();
  int err = 0;

  if (!stream.isOpen())
  {
    return SE_FileOpenFailure;
  }

  map.clear();

  // Header is read uncompressed.
  size_t regionCount = 0;
  HeaderVersion version;
  err = loadHeader(stream, version, detail, regionCount);
  if (versionOut)
  {
    *versionOut = version.version;
  }

  return err;
}
