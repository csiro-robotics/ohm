// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "CopyUtil.h"

#include "private/OccupancyMapDetail.h"

#include "DefaultLayer.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapRegionCache.h"
#include "OccupancyMap.h"
#include "VoxelBlock.h"
#include "VoxelBuffer.h"

#include <glm/glm.hpp>

#include <cstring>

namespace
{
void copyChunkLayerUnsafe(ohm::MapChunk &dst_chunk, unsigned dst_layer, const ohm::MapChunk &src_chunk,
                          unsigned src_layer)
{
  ohm::VoxelBuffer<ohm::VoxelBlock> dst_buffer(dst_chunk.voxel_blocks[dst_layer]);
  ohm::VoxelBuffer<const ohm::VoxelBlock> src_buffer(src_chunk.voxel_blocks[src_layer]);

  memcpy(dst_buffer.voxelMemory(), src_buffer.voxelMemory(), src_buffer.voxelMemorySize());
}
}  // namespace


namespace ohm
{
CopyChunkFilter copyFilterExtents(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext)
{
  return [min_ext, max_ext](const MapChunk &chunk) {
    const glm::dvec3 region_half_ext = 0.5 * chunk.map->region_spatial_dimensions;
    const glm::dvec3 region_min = chunk.region.centre - region_half_ext;
    const glm::dvec3 region_max = chunk.region.centre + region_half_ext;
    return !glm::any(glm::lessThan(region_max, min_ext)) && !glm::any(glm::greaterThan(region_min, max_ext));
  };
}

CopyChunkFilter copyFilterStamp(uint64_t after_stamp)
{
  return [after_stamp](const MapChunk &chunk) { return chunk.dirty_stamp > after_stamp; };
}

bool canCopy(const OccupancyMap &dst, const OccupancyMap &src)
{
  return &src != &dst && src.resolution() == dst.resolution() &&
         src.regionVoxelDimensions() == dst.regionVoxelDimensions() && src.origin() == dst.origin();
}

bool copyMap(OccupancyMap &dst, const OccupancyMap &src, const CopyChunkFilter &copy_chunk_filter,
             const CopyLayerFilter &copy_layer_filter)
{
  if (!canCopy(dst, src))
  {
    return false;
  }

  OccupancyMapDetail &dst_detail = *dst.detail();
  const MapLayout &dst_layout = dst_detail.layout;
  const OccupancyMapDetail &src_detail = *src.detail();

  // Lock required mutexes.
  // We only lock the source map and assume we are the only writers to the dst map. The @c region() call will lock dst
  // map mutex.
  std::unique_lock<decltype(src_detail.mutex)> src_guard(src_detail.mutex);

  // First resolve the overlapping layer set. Holds src, dst map layer index pairs.
  std::vector<std::pair<unsigned, unsigned>> layer_overlap;
  std::vector<MapRegionCache *> layer_caches;  // Cache pointers matching layer_overlap[n].first .
  if (src_detail.layout.calculateOverlappingLayerSet(layer_overlap, dst_detail.layout) == 0)
  {
    // No common layers to copy.
    return false;
  }

  // We have layer_overlap which identifies source to destination layer indices. Now we remove any source layer entries
  // which fail copy_layer_filter.
  if (copy_layer_filter)
  {
    for (auto iter = layer_overlap.begin(); iter != layer_overlap.end();)
    {
      const std::string source_layer_name = src_detail.layout.layer(iter->first).name();
      if (!copy_layer_filter(source_layer_name))
      {
        iter = layer_overlap.erase(iter);
      }
      else
      {
        ++iter;
      }
    }
  }

  // Find layer caches for each layer_overlap entry.
  if (src_detail.gpu_cache)
  {
    for (const auto &overlap : layer_overlap)
    {
      layer_caches.emplace_back(src_detail.gpu_cache->findLayerCache(overlap.first));
    }
  }
  else if (!layer_overlap.empty())
  {
    layer_caches.resize(layer_overlap.size());
    std::fill(layer_caches.begin(), layer_caches.end(), nullptr);
  }

  const int tsdf_layer_index = dst_layout.layerIndex(default_layer::tsdfLayerName());
  for (const auto &src_iter : src_detail.chunks)
  {
    if (!src_iter.second || (copy_chunk_filter && !copy_chunk_filter(*src_iter.second)))
    {
      // Excluded chunk.
      continue;
    }

    const MapChunk &src_chunk = *src_iter.second;
    MapChunk &dst_chunk = *dst.region(src_iter.first, true);
    assert(&dst_chunk);

    // Included chunk.
    // First try copy via the GPU cache.
    for (size_t i = 0; i < layer_overlap.size(); ++i)
    {
      const auto &layer_pair = layer_overlap[i];
      auto *layer_cache = (i < layer_caches.size()) ? layer_caches[i] : nullptr;
      const bool update_first_valid =
        (dst_layout.occupancyLayer() >= 0 && layer_pair.second == unsigned(dst_layout.occupancyLayer())) ||
        (tsdf_layer_index >= 0 && layer_pair.second == unsigned(tsdf_layer_index));
      // First try letting the layer cache handle the copy/sync.
      if (layer_cache && layer_cache->syncLayerTo(dst_chunk, layer_pair.second, src_chunk, layer_pair.first))
      {
        // Special case: if we are dealing with the occupancy or TSDF layer, then we need to update
        // MapRegion::first_valid_index in the target map for correct map iteration. However, when the layer cache
        // handles the copy we can't guarantee the source map value is up to date, so we can't just copy from the source
        // chunk value.
        if (update_first_valid)
        {
          dst_chunk.searchAndUpdateFirstValid(dst_detail.region_voxel_dimensions);
        }
      }
      else
      {
        // Layer cache not present or it didn't handle the copy. Use the fallback function.
        copyChunkLayerUnsafe(dst_chunk, layer_pair.second, src_chunk, layer_pair.first);
        // Special case: as in the branch above, but this time we can just copy the first_valid_index from the source
        // chunk as there's no layer cache and we can assume the MapChunk is fully up to date.
        if (update_first_valid)
        {
          dst_chunk.updateFirstValid(src_chunk.first_valid_index);
        }
      }
    }
  }

  return true;
}  // namespace ohm

}  // namespace ohm
