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
  constexpr double probToLikelihood(double d) { return std::log(d / (1.0 - d)); }
  static constexpr double kFreeThreshold = probToLikelihood(0.45);
  static constexpr double kOccupiedThreshold = probToLikelihood(0.55);
}  // namespace

namespace ohm
{
  struct SnapshotDetail
  {
    OccupancyMap *occupancy_map;
  };
}  // namespace ohmmapping


Snapshot::SnapshotNode::SnapshotNode()
  : state_(Snapshot::State::kUnknown)
{}


Snapshot::SnapshotNode::SnapshotNode(const MapChunk &ch, const OccupancyMap &map, MapCache &map_cache)
  : SnapshotNode(
      ch.region.coord, map, map_cache,
      glm::u8vec3(0, 0, 0),
      glm::u8vec3(map.regionVoxelDimensions().x, map.regionVoxelDimensions().y, map.regionVoxelDimensions().z))
{}


Snapshot::SnapshotNode::SnapshotNode(const glm::i16vec3 &region_key, const OccupancyMap &map, MapCache &map_cache,
                                     glm::u8vec3 min_ext, glm::u8vec3 max_ext)
  : state_(Snapshot::State::kUnknown)
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
      const VoxelConst voxel = map.voxel(Key(region_key, min_ext), &map_cache);
      const double l_occ = voxel.isValid() ? voxel.occupancy() : 0.0;
      state_ = l_occ > kOccupiedThreshold && l_occ != ohm::voxel::invalidMarkerValue() ?
                 Snapshot::State::kOccupied :
                 (l_occ < kFreeThreshold ? Snapshot::State::kFree : Snapshot::State::kUnknown);
    }
    else
    {
      // Inner node
      children_ = std::make_unique<std::array<SnapshotNode, 8>>();
      glm::u8vec3 new_min_ext, new_max_ext;
      unsigned ch_index = 0;
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

            (*children_)[ch_index++] = SnapshotNode(region_key, map, map_cache, new_min_ext, new_max_ext);
            state_ = Snapshot::State::kNonLeaf;
          }
        }
      }
    }
  }
}


Snapshot::SnapshotNode::SnapshotNode(const SnapshotRegion &region, size_t &byte_address,
                                       uint8_t &bit_address)
{
  state_ = static_cast<State>((region.data[byte_address] >> bit_address) & kStateMask);
  bit_address += kStateBits;
  if (bit_address == 8)
  {
    bit_address = 0;
    ++byte_address;
  }
  if (state_ == kNonLeaf)
  {
    children_ = std::make_unique<std::array<SnapshotNode, 8>>();
    for (auto &ch : *children_)
    {
      ch = SnapshotNode(region, byte_address, bit_address);
    }
  }
}

Snapshot::SnapshotNode::SnapshotNode(const SnapshotNode &ss)
: state_(ss.state_)
{}


Snapshot::SnapshotNode::~SnapshotNode() = default;


std::tuple<uint16_t, uint16_t, uint16_t> Snapshot::SnapshotNode::simplify(float voxel_size)
{
  if (!children_)
  {
    // Base case--no children
    switch (state_)
    {
    case Snapshot::State::kFree:
      return std::make_tuple(uint16_t(1), uint16_t(0), uint16_t(0));
    case Snapshot::State::kOccupied:
      return std::make_tuple(uint16_t(0), uint16_t(1), uint16_t(0));
    case Snapshot::State::kUnknown:
      return std::make_tuple(uint16_t(0), uint16_t(0), uint16_t(1));
    default:
      return std::make_tuple(uint16_t(0), uint16_t(0), uint16_t(0));
    }
  }

  // Recurse to children
  auto res = std::make_tuple(uint16_t(0), uint16_t(0), uint16_t(0));
  for (auto &ch : *children_)
  {
    const auto ch_res = ch.simplify(voxel_size * 0.5f);
    std::get<0>(res) += std::get<0>(ch_res);
    std::get<1>(res) += std::get<1>(ch_res);
    std::get<2>(res) += std::get<2>(ch_res);
  }

  // Make this a leaf node if there is only one type, or if it isn't too big and is a mix of free and unknown
  const auto sum = std::get<0>(res) + std::get<1>(res) + std::get<2>(res);
  if (std::get<0>(res) == sum)
  {
    state_ = Snapshot::State::kFree;
    delete children_.release();
  }
  else if (std::get<1>(res) == sum)
  {
    state_ = Snapshot::State::kOccupied;
    delete children_.release();
  }
  else if (std::get<2>(res) == sum)
  {
    state_ = Snapshot::State::kUnknown;
    delete children_.release();
  }
  else if (voxel_size < 1.0 && std::get<0>(res) > std::get<2>(res) && std::get<1>(res) == 0)
  {
    state_ = Snapshot::State::kFree;
    delete children_.release();
  }
  else if (voxel_size < 0.25 && std::get<1>(res) > 0)
  {
    state_ = Snapshot::State::kOccupied;
    delete children_.release();
  }
  else if (voxel_size < 0.25 && std::get<0>(res) > 0)
  {
    state_ = Snapshot::State::kFree;
    delete children_.release();
  }

  return res;
}


void Snapshot::SnapshotNode::getVoxels(size_t layer, float voxel_size, glm::f32vec3 centre,
                                        std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const
{
  if (layer == 0 && state_ != Snapshot::State::kNonLeaf)
  {
    // we're outputting this layer
    voxels.emplace_back(uint8_t(state_), centre);
  }
  else if (children_)
  {
    unsigned ch_index = 0;
    float new_voxel_size = 0.5 * voxel_size;
    for (float dx = -0.5; dx < 1; dx += 1)
    {
      for (float dy = -0.5; dy < 1; dy += 1)
      {
        for (float dz = -0.5; dz < 1; dz += 1)
        {
          (*children_)[ch_index++].getVoxels(
            layer - 1, new_voxel_size,
            glm::f32vec3(centre.x + dx * new_voxel_size, centre.y + dy * new_voxel_size,
                         centre.z + dz * new_voxel_size),
            voxels);
        }
      }
    }
  }
}


size_t Snapshot::SnapshotNode::treeHeight() const
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


size_t Snapshot::SnapshotNode::memoryUse() const
{
  size_t mem_use = sizeof(SnapshotNode);
  if (children_)
  {
    for (const auto &ch : *children_)
    {
      mem_use += ch.memoryUse();
    }
  }
  return mem_use;
}


size_t Snapshot::SnapshotNode::numNode() const
{
  size_t num_node = 1;
  if (children_)
  {
    for (const auto &ch : *children_)
    {
      num_node += ch.numNode();
    }
  }
  return num_node;
}


SnapshotRegion Snapshot::SnapshotNode::getRegion(double time, const glm::i16vec3 &region_key) const
{
  SnapshotRegion region;
  region.time = time;
  region.region_key = region_key;
  region.data.reserve((numNode() >> kStateBits) + 1);
  uint8_t bits_used_in_final_byte = 0;
  getRegion(region, bits_used_in_final_byte);
  return region;
}


void Snapshot::SnapshotNode::getRegion(SnapshotRegion &region, uint8_t &bits_used_in_final_byte) const
{
  if (bits_used_in_final_byte == 0 || bits_used_in_final_byte == 8)
  {
    region.data.push_back(0);
    bits_used_in_final_byte = 0;
  }

  uint8_t &byte = region.data.back();
  byte = byte | ((state_ & kStateMask) << bits_used_in_final_byte);
  bits_used_in_final_byte += kStateBits;

  if (state_ == kNonLeaf)
  {
    if (!children_)
    {
      throw std::runtime_error("Non-leaf node had no children");
    }
    for (const auto &ch : *children_)
    {
      ch.getRegion(region, bits_used_in_final_byte);
    }
  }
}


Snapshot::SnapshotNode &Snapshot::SnapshotNode::operator=(SnapshotNode &&s) = default;


Snapshot::Snapshot()
  : imp_(new SnapshotDetail{ nullptr })
{}


Snapshot::~Snapshot() = default;


std::vector<SnapshotRegion> Snapshot::buildSnapshot(const ohm::Aabb &cull_to)
{
  std::vector<SnapshotRegion> regions;

  if (!imp_->occupancy_map)
  {
    return regions;
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

  for (const auto ch_ptr : chunks)
  {
    const MapChunk &ch = *ch_ptr;
    if (!ch.overlapsExtents(src_region.minExtents(), src_region.maxExtents(), src_map.regionSpatialResolution()))
    {
      continue;
    }

    SnapshotNode ss_node(ch, src_map, map_cache);
    ss_node.simplify(src_map.regionSpatialResolution().x);
    regions.push_back(ss_node.getRegion(0.0, ch.region.coord));
  }

#if PROFILING
  ohm::Profile::instance().report();
#endif  // PROFILING

  return regions;
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
