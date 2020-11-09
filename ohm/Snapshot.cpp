// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Jason Williams
#include "Snapshot.h"

#include "Aabb.h"
#include "Key.h"
#include "MapCache.h"
#include "MapChunk.h"
#include "MapCoord.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "OccupancyType.h"
#include "Voxel.h"

#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#include "CovarianceVoxel.h"

#include <algorithm>
#include <cstring>
#include <iostream>

#define PROFILING 0
#include <ohmutil/Profile.h>

using namespace ohm;

namespace
{

}  // namespace

namespace ohm
{
  struct SnapshotDetail
  {
    OccupancyMap *occupancy_map;
  };
}  // namespace ohmmapping


Snapshot::SnapshotChunk::SnapshotChunk()
  : p_occ_(0)
  , min_p_occ_(255)
  , max_p_occ_(0)
{}


Snapshot::SnapshotChunk::SnapshotChunk(const MapChunk &ch, const OccupancyMap &map, MapCache &map_cache)
  : SnapshotChunk(
      ch.region.coord, map, map_cache,
      glm::u8vec3(std::min(ch.first_valid_index.x, uint8_t(map.regionVoxelDimensions().x - 1)),
                  std::min(ch.first_valid_index.y, uint8_t(map.regionVoxelDimensions().y - 1)),
                  std::min(ch.first_valid_index.z, uint8_t(map.regionVoxelDimensions().z - 1))),
      glm::u8vec3(map.regionVoxelDimensions().x, map.regionVoxelDimensions().y, map.regionVoxelDimensions().z))
{}


Snapshot::SnapshotChunk::SnapshotChunk(const glm::i16vec3 &region_key, const OccupancyMap &map, MapCache &map_cache,
                                       glm::u8vec3 min_ext, glm::u8vec3 max_ext)
  : p_occ_(0)
  , min_p_occ_(255)
  , max_p_occ_(0)
{
  // Can end up with empty blocks due to non-power of two or non-uniform chunk sizes
  if (min_ext.x >= max_ext.x || min_ext.y >= max_ext.y || min_ext.z >= max_ext.z)
  {
    // accept default values, representing invalid
  }
  else
  {
    if (min_ext.x + 1 == max_ext.x && min_ext.y + 1 == max_ext.y && min_ext.z + 1 == max_ext.z)
    {
      // Leaf node
      const Key key(region_key, min_ext);
      const VoxelConst voxel = map.voxel(key, &map_cache);
      const bool is_valid = voxel.isValid();
      const double l_occ = is_valid ? voxel.occupancy() : 0.0;
      const double exp_l_occ = std::isnan(l_occ) ? 1.0 : std::exp(l_occ);
      const double p_occ = exp_l_occ / (1.0 + exp_l_occ);
      p_occ_ = min_p_occ_ = max_p_occ_ = std::isnan(p_occ) ? uint8_t(0) : uint8_t(255*p_occ);
    }
    else
    {
      // Inner node
      children_ = std::make_unique<std::array<SnapshotChunk, 8>>();
      glm::u8vec3 new_min_ext, new_max_ext;
      unsigned ch_index = 0, num_valid_ch = 0;
      unsigned p_occ = 0;
      for (uint8_t x = 0; x < 2; ++x)
      {
        new_min_ext.x = x == 0 ? min_ext.x : (min_ext.x + max_ext.x) >> 1;
        new_max_ext.x = x == 0 ? (min_ext.x + max_ext.x) >> 1 : max_ext.x;
        for (uint8_t y = 0; y < 2; ++y)
        {
          new_min_ext.y = y == 0 ? min_ext.y : (min_ext.y + max_ext.y) >> 1;
          new_max_ext.y = y == 0 ? (min_ext.y + max_ext.y) >> 1 : max_ext.y;
          for (uint8_t z = 0; z < 2; ++z)
          {
            new_min_ext.z = z == 0 ? min_ext.z : (min_ext.z + max_ext.z) >> 1;
            new_max_ext.z = z == 0 ? (min_ext.z + max_ext.z) >> 1 : max_ext.z;

            auto &ch = (*children_)[ch_index++] = SnapshotChunk(region_key, map, map_cache, new_min_ext, new_max_ext);
            if (ch.max_p_occ_ >= ch.min_p_occ_)
            {
              p_occ += ch.p_occ_;
              min_p_occ_ = std::min(min_p_occ_, ch.min_p_occ_);
              max_p_occ_ = std::max(max_p_occ_, ch.max_p_occ_);
              ++num_valid_ch;
            }
          }
        }
      }
      p_occ_ = uint8_t(p_occ / num_valid_ch); // true mean only for power of two dimensions
    }
  }
}


Snapshot::SnapshotChunk::~SnapshotChunk() = default;


void Snapshot::SnapshotChunk::simplify(float voxel_size)
{
  if (!children_ || min_p_occ_ > max_p_occ_)
  {
    return;
  }
  if ((max_p_occ_ < 128 && voxel_size < 2.0) ||  //
      (max_p_occ_ < 64 && voxel_size < 4.0) ||   //
      (max_p_occ_ < 32) ||                       // any size
      (max_p_occ_ - min_p_occ_ < 16) ||          // any size
      (min_p_occ_ >= 192 && voxel_size < 0.25) || //
      (min_p_occ_ >= 224 && voxel_size < 1))
  {
    delete children_.release();
  }
  else
  {
    for (auto &ch : *children_)
    {
      ch.simplify(voxel_size / 2.0f);
    }
  }
}


void Snapshot::SnapshotChunk::getVoxels(size_t layer, float voxel_size, glm::f32vec3 centre,
                                        std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const
{
  if (layer == 0 && min_p_occ_ <= max_p_occ_)
  {
    // we're outputting this layer
    voxels.emplace_back(p_occ_, centre);
  }
  else if (children_)
  {
    unsigned ch_index = 0;
    float new_voxel_size = 0.5*voxel_size;
    for (float dx = -0.5; dx < 1; dx += 1)
    {
      for (float dy = -0.5; dy < 1; dy += 1)
      {
        for (float dz = -0.5; dz < 1; dz += 1)
        {
          (*children_)[ch_index++].getVoxels(
            layer - 1, new_voxel_size,
            glm::f32vec3(centre.x + dx * new_voxel_size, centre.y + dy * new_voxel_size,
                         centre.z + dz * new_voxel_size), voxels);
        }
      }
    }
  }
}


size_t Snapshot::SnapshotChunk::treeHeight() const
{
  size_t tree_height = 0;
  if (children_)
  {
    for (const auto &ch : *children_)
    {
      tree_height = std::max(tree_height, ch.treeHeight());
    }
  }
  return tree_height + 1;
}


size_t Snapshot::SnapshotChunk::memoryUse() const
{
  size_t mem_use = sizeof(SnapshotChunk);
  if (children_)
  {
    for (const auto &ch : *children_)
    {
      mem_use += ch.memoryUse();
    }
  }
  return mem_use;
}


Snapshot::SnapshotChunk &Snapshot::SnapshotChunk::operator=(SnapshotChunk &&s) = default;



Snapshot::Snapshot()
  : imp_(new SnapshotDetail{nullptr})
{
}


Snapshot::~Snapshot() = default;


std::vector<std::pair<float, std::vector<std::pair<uint8_t, glm::f32vec3>>>> Snapshot::buildSnapshot(const ohm::Aabb &cull_to)
{
  if (!imp_->occupancy_map)
  {
    return std::vector<std::pair<float, std::vector<std::pair<uint8_t, glm::f32vec3>>>>();
  }

  PROFILE(buildSnapshot);

  const OccupancyMap &src_map = *imp_->occupancy_map;
  ohm::Aabb src_region;
  src_map.calculateExtents(&src_region.minExtentsMutable(), &src_region.maxExtentsMutable());

  // Clip to the cull box.
  for (int i = 0; i < 3; ++i)
  {
    if (cull_to.diagonal()[i] > 0)
    {
      src_region.minExtentsMutable()[i] = cull_to.minExtents()[i];
      src_region.maxExtentsMutable()[i] = cull_to.maxExtents()[i];
    }
  }

  std::vector<const MapChunk *> chunks;
  src_map.enumerateRegions(chunks);
  MapCache map_cache;

  // Construct mean occupancy of each chunk
  std::vector<std::pair<float, std::vector<std::pair<uint8_t, glm::f32vec3>>>> voxel_layers;
  const float chunk_size = float(src_map.regionSpatialResolution().x), voxel_size = float(src_map.resolution());
  float layer_size = chunk_size;
  while (layer_size >= voxel_size)
  {
    voxel_layers.emplace_back(layer_size, std::vector<std::pair<uint8_t, glm::f32vec3>>());
    layer_size *= 0.5f;
  }

  size_t mem_use = 0;
  for (const auto ch_ptr : chunks)
  {
    const MapChunk &ch = *ch_ptr;
    if (!ch.overlapsExtents(src_region.minExtents(), src_region.maxExtents(), src_map.regionSpatialResolution()))
    {
      continue;
    }

    /*// Calculate mean occupancy within region
    double mean_occ = 0.0f;
    size_t num_voxel = 0;

    // firstKeyForChunk
    const auto region_voxel_dimensions = src_map.regionVoxelDimensions();
    auto key = Key(ch.region.coord, std::min(ch.first_valid_index.x, uint8_t(region_voxel_dimensions.x - 1)),
                   std::min(ch.first_valid_index.y, uint8_t(region_voxel_dimensions.y - 1)),
                   std::min(ch.first_valid_index.z, uint8_t(region_voxel_dimensions.z - 1)));
    do
    {
      const VoxelConst voxel = src_map.voxel(key, &map_cache);
      const bool is_valid = voxel.isValid();
      ++num_voxel;
      const double exp_val = is_valid ? std::exp(voxel.occupancy()) : 1.0;
      const double occ_prob = exp_val / (1.0 + exp_val);
      mean_occ += std::isnan(occ_prob) ? 0.5 : occ_prob;
    } while (nextLocalKey(key, src_map.regionVoxelDimensions()));
    mean_occupancies.emplace_back(ch.region.centre, num_voxel > 0 ? double(mean_occ / double(num_voxel)) : 0.5f); */

    SnapshotChunk ss_ch(ch, src_map, map_cache);
    ss_ch.simplify(src_map.regionSpatialResolution().x);
    mem_use += ss_ch.memoryUse();
    glm::f32vec3 ch_centre(ch.region.centre);
    for (size_t i = 0; i < voxel_layers.size(); ++i)
    {
      ss_ch.getVoxels(i, chunk_size, ch_centre, voxel_layers[i].second);
    }
  }
  std::cout << "snapshot memory: " << mem_use/1024 << " kb" << std::endl;


#if PROFILING
  ohm::Profile::instance().report();
#endif  // PROFILING

  return voxel_layers;
}


void Snapshot::setOccupancyMap(OccupancyMap *map)
{
  imp_->occupancy_map = map;
}

/// Access the current source occupancy map.
OccupancyMap *Snapshot::occupancyMap() const
{
  return imp_->occupancy_map;
}
