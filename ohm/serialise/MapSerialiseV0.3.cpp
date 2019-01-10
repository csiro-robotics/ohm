// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapSerialiseV0.3.h"

#include "MapSerialiseV0.2.h"

#include "private/OccupancyMapDetail.h"
#include "private/SerialiseUtil.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapSerialise.h"
#include "Stream.h"

namespace ohm
{
  namespace v0_3
  {
    int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
             size_t region_count)
    {
      return load(stream, detail, progress, version, region_count, &v0_3::loadChunk);
    }

    int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
             size_t region_count, const ChunkFunc &load_chunk)
    {
      return v0_2::load(stream, detail, progress, version, region_count, load_chunk);
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

          if (layer.flags() & MapLayer::kSkipSerialise)
          {
            // Not to be serialised. Clear instead.
            layer.clear(layer.voxels(chunk), detail.region_voxel_dimensions);
            continue;
          }

          uint64_t layer_touched_stamp = 0;
          ok = read<uint64_t>(stream, layer_touched_stamp) && ok;

          chunk.touched_stamps[i] = layer_touched_stamp;

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

      return (ok) ? 0 : kSeFileReadFailure;    }
  }  // namespace v0
}  // namespace ohm