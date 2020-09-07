// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapSerialiseV0.1.h"

#include "private/OccupancyMapDetail.h"
#include "private/SerialiseUtil.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapSerialise.h"
#include "Stream.h"
#include "VoxelBlock.h"
#include "VoxelBuffer.h"

namespace ohm
{
  namespace v0_1
  {
    int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
             size_t region_count)
    {
      return load(stream, detail, progress, version, region_count, v0_1::loadChunk);
    }

    int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress,
             const MapVersion & /*version*/, size_t region_count, const ChunkFunc &load_chunk)
    {
      int err = loadLayout(stream, detail);
      if (err)
      {
        return err;
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
        chunk = new MapChunk(detail);
        err = load_chunk(stream, *chunk, detail);
        if (err)
        {
          delete chunk;
          return err;
        }

        // Resolve map chunk details.
        chunk->searchAndUpdateFirstValid(detail.region_voxel_dimensions);
        detail.chunks.insert(std::make_pair(chunk->region.coord, chunk));

        if (progress)
        {
          progress->incrementProgress();
        }
      }

      return err;
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

        layer_name.resize(len + 1u);  // NOLINT
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

          member_name.resize(len + 1u);  // NOLINT
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
              // ok = false;
              return kSeMemberOffsetError;
            }
          }
        }
      }

      return (ok) ? 0 : kSeFileReadFailure;
    }


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
  }  // namespace v0_1
}  // namespace ohm