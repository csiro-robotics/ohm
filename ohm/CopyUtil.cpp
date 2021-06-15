// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "CopyUtil.h"

#include "private/OccupancyMapDetail.h"

#include "MapChunk.h"
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
  using namespace ohm;

  ohm::VoxelBuffer<ohm::VoxelBlock> dst_buffer(dst_chunk.voxel_blocks[dst_layer]);
  ohm::VoxelBuffer<const ohm::VoxelBlock> src_buffer(src_chunk.voxel_blocks[src_layer]);

  memcpy(dst_buffer.voxelMemory(), src_buffer.voxelMemory(), src_buffer.voxelMemorySize());
}
}  // namespace


namespace ohm
{
CopyFilter copyFilterExtents(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext)
{
  return [min_ext, max_ext](const MapChunk &chunk) {
    const glm::dvec3 region_half_ext = 0.5 * chunk.map->region_spatial_dimensions;
    const glm::dvec3 region_min = chunk.region.centre - region_half_ext;
    const glm::dvec3 region_max = chunk.region.centre + region_half_ext;
    return !glm::any(glm::lessThan(region_max, min_ext)) && !glm::any(glm::greaterThan(region_min, max_ext));
  };
}

CopyFilter copyFilterStamp(uint64_t at_or_after_stamp)
{
  return [at_or_after_stamp](const MapChunk &chunk) { return chunk.dirty_stamp >= at_or_after_stamp; };
}

bool canCopy(const OccupancyMap &dst, const OccupancyMap &src)
{
  return &src != &dst && src.resolution() == dst.resolution() &&
         src.regionVoxelDimensions() == dst.regionVoxelDimensions() && src.origin() == dst.origin();
}

bool copyMap(OccupancyMap &dst, const OccupancyMap &src, CopyFilter copy_filter)
{
  if (!canCopy(dst, src))
  {
    return false;
  }

  OccupancyMapDetail &dst_detail = *dst.detail();
  const OccupancyMapDetail &src_detail = *src.detail();

  // Lock required mutexes. TODO: convert to C++17 std::scoped_lock for beter deadlock avoidance.
  std::unique_lock<decltype(dst_detail.mutex)> dst_guard(dst_detail.mutex);
  std::unique_lock<decltype(src_detail.mutex)> src_guard(src_detail.mutex);

  // First resolve the overlapping layer set.
  std::vector<std::pair<unsigned, unsigned>> layer_overlap;
  if (src_detail.layout.calculateOverlappingLayerSet(layer_overlap, dst_detail.layout) == 0)
  {
    // No common layers to copy.
    return false;
  }

  for (const auto &src_iter : src_detail.chunks)
  {
    if (!src_iter.second || (copy_filter && !copy_filter(*src_iter.second)))
    {
      // Excluded chunk.
      continue;
    }

    const MapChunk &src_chunk = *src_iter.second;
    MapChunk &dst_chunk = *dst.region(src_iter.first, true);
    assert(&dst_chunk);

    // Included chunk.
    // First try copy via the GPU cache.
    for (auto &&layer_pair : layer_overlap)
    {
      if (!src_detail.gpu_cache ||
          !src_detail.gpu_cache->syncLayerTo(dst_chunk, layer_pair.second, src_chunk, layer_pair.first))
      {
        copyChunkLayerUnsafe(dst_chunk, layer_pair.second, src_chunk, layer_pair.first);
      }
    }
  }

  return true;
}  // namespace ohm

}  // namespace ohm
