// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapSerialiseV0.h"

#include "private/OccupancyMapDetail.h"
#include "private/SerialiseUtil.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapSerialise.h"
#include "Stream.h"

namespace ohm
{
  namespace v0
  {
    int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &/*version*/,
             size_t region_count)
    {
      int err = kSeOk;

      detail.setDefaultLayout();

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
        err = v0::loadChunk(stream, *chunk, detail);
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

      return err;
    }


    // Version zero chunk loading
    int loadChunk(InputStream &stream, MapChunk &chunk, const OccupancyMapDetail &detail)
    {
      bool ok = true;

      const MapLayer *occupancy_layer = chunk.layout->layerPtr(chunk.layout->occupancyLayer());
      const MapLayer *clearance_layer = chunk.layout->layerPtr(chunk.layout->clearanceLayer());
      // Use a hard coded reference to the legacy layer "coarseClearance". The layer was never used anywhere.
      const MapLayer *coarse_clearance_layer = chunk.layout->layer("coarseClearance");

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
  }  // namespace v0
}  // namespace ohm