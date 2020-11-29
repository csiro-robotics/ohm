// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Jason Williams
#include "Snapshot.h"

#include "Aabb.h"
#include "Key.h"
#include "MapChunk.h"
#include "MapCoord.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "OccupancyType.h"
#include "Voxel.h"
#include "VoxelBuffer.h"
#include "VoxelOccupancy.h"

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

struct SnapshotSrcVoxel
{
  Voxel<const float> occupancy;  ///< Occupancy value (required)
  float occupancy_threshold;     ///< Occupancy threshold cached from the source map.

  SnapshotSrcVoxel(const OccupancyMap &map)
    : occupancy(&map, map.layout().occupancyLayer())
    , occupancy_threshold(map.occupancyThresholdValue())
  {}

  /// Set the key, but only for the occupancy layer.
  inline void setKey(const Key &key) { occupancy.setKey(key); }

  /// Query the target map.
  inline const OccupancyMap &map() const { return *occupancy.map(); }

  /// Query the occupancy classification of the current voxel.
  inline Snapshot::State state() const
  {
    float value = unobservedOccupancyValue();
#ifdef __clang_analyzer__
    if (occupancy.voxelMemory())
#else   // __clang_analyzer__
    if (occupancy.isValid())
#endif  // __clang_analyzer__
    {
      occupancy.read(&value);
    }
    Snapshot::State state =
      (value >= Snapshot::kOccupiedThreshold) ?
        Snapshot::State::kOccupied :
        ((value <= Snapshot::kFreeThreshold) ? Snapshot::State::kFree : Snapshot::State::kUnobserved);
    state = value != unobservedOccupancyValue() ? state : Snapshot::State::kUnobserved;
    return occupancy.chunk() ? state : Snapshot::State::kUnobserved;
  }
};

struct SnapshotVoxelBlock
{
  int occupancy_layer;
  glm::u8vec3 occupancy_dim;
  VoxelBuffer<VoxelBlock> occupancy_buffer;

  SnapshotVoxelBlock(OccupancyMap &map, MapChunk *chunk)
    : occupancy_layer(map.layout().occupancyLayer())
    , occupancy_dim(map.layout().layerPtr(occupancy_layer)->dimensions(map.regionVoxelDimensions()))
    , occupancy_buffer(chunk->voxel_blocks[occupancy_layer])
  {}

  /// Query the occupancy classification of the current voxel.
  inline Snapshot::State state(const Key &key)
  {
    const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
    float value;
    occupancy_buffer.readVoxel(voxel_index, &value);

    Snapshot::State state =
      (value >= Snapshot::kOccupiedThreshold) ?
        Snapshot::State::kOccupied :
        ((value <= Snapshot::kFreeThreshold) ? Snapshot::State::kFree : Snapshot::State::kUnobserved);
    return value != unobservedOccupancyValue() ? state : Snapshot::State::kUnobserved;
  }

  /// Update the occupancy classification of the current voxel.
  inline Snapshot::State update(const Key &key, Snapshot::State state)
  {
    const unsigned voxel_index = ohm::voxelIndex(key, occupancy_dim);
    float value;
    occupancy_buffer.readVoxel(voxel_index, &value);

    Snapshot::State old_state =
      (value >= Snapshot::kOccupiedThreshold) ?
        Snapshot::State::kOccupied :
        ((value <= Snapshot::kFreeThreshold) ? Snapshot::State::kFree : Snapshot::State::kUnobserved);
    old_state = value != unobservedOccupancyValue() ? state : Snapshot::State::kUnobserved;

    value = (state == Snapshot::State::kOccupied) ?
              Snapshot::kOccupiedThreshold :
              ((state == Snapshot::State::kFree) ? Snapshot::kFreeThreshold : value);
    occupancy_buffer.writeVoxel(voxel_index, value);
    return (state == Snapshot::State::kOccupied) ?
             Snapshot::State::kOccupied :
             ((state == Snapshot::State::kFree) ? Snapshot::State::kFree : old_state);
  }
};


struct SnapshotDetail
{
  OccupancyMap *occupancy_map;
};
}  // namespace ohm


Snapshot::SnapshotNode::SnapshotNode()
  : state_(Snapshot::State::kUnobserved)
{}


Snapshot::SnapshotNode::SnapshotNode(const glm::i16vec3 &region_key, SnapshotSrcVoxel &voxel, glm::u8vec3 min_ext,
                                     glm::u8vec3 max_ext)
  : state_(Snapshot::State::kUnobserved)
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
      voxel.setKey(Key(region_key, min_ext));
      state_ = voxel.state();
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

            (*children_)[ch_index++] = SnapshotNode(region_key, voxel, new_min_ext, new_max_ext);
            state_ = Snapshot::State::kNonLeaf;
          }
        }
      }
    }
  }
}


Snapshot::SnapshotNode::SnapshotNode(const EncodedSnapshotRegion &region, size_t &byte_address,
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
{
  if (ss.children_)
  {
    children_ = std::make_unique<std::array<SnapshotNode, 8>>();
    for (size_t i = 0; i < 8; ++i) 
    {
      (*children_)[i] = SnapshotNode((*ss.children_)[i]);
    }
  }
}


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
    case Snapshot::State::kUnobserved:
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
    state_ = Snapshot::State::kUnobserved;
    delete children_.release();
  }
  //else if (voxel_size < 1.0 && std::get<0>(res) > std::get<2>(res) && std::get<1>(res) == 0)
  //{
  //  state_ = Snapshot::State::kFree;
  //  delete children_.release();
  //}
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

void Snapshot::SnapshotNode::getRegion(EncodedSnapshotRegion &region, uint8_t &bits_used_in_final_byte) const
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

void Snapshot::SnapshotNode::update(const Snapshot::SnapshotNode &s2)
{
  switch (s2.state_)
  {
  case State::kFree:
  case State::kOccupied:
    state_ = s2.state_;
    children_ = nullptr;
    break;
  case State::kNonLeaf:
    if (!s2.children_)
    {
      throw std::runtime_error("Encountered non-leaf node with no children");
    }
    if (!children_)
    {
      children_ = std::make_unique<std::array<SnapshotNode, 8>>();
      for (auto &ch : *children_)
      {
        ch.state_ = state_;
      }
      state_ = State::kNonLeaf;
    }
    for (size_t i = 0; i < 8; ++i)
    {
      (*children_)[i].update((*s2.children_)[i]);
    }
    break;
  default: // aka State::kUnobserved
    break;
  }
}

void Snapshot::SnapshotNode::updateOccupancyMap(const glm::i16vec3 &region_key, SnapshotVoxelBlock &voxel,
                                                glm::u8vec3 min_ext, glm::u8vec3 max_ext)
{
  if (children_)
  {
    // Process each child
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
          (*children_)[ch_index++].updateOccupancyMap(region_key, voxel, new_min_ext, new_max_ext);
        }
      }
    }
  }
  else
  {
    // Populate each voxel in region with state
    for (uint8_t x = min_ext.x; x < max_ext.x; ++x)
    {
      for (uint8_t y = min_ext.y; y < max_ext.y; ++y)
      {
        for (uint8_t z = min_ext.z; z < max_ext.z; ++z)
        {
          voxel.update(Key(region_key, glm::u8vec3(x, y, z)), state_);
        }
      }
    }
  }
}

Snapshot::SnapshotNode &Snapshot::SnapshotNode::operator=(SnapshotNode &&ss) = default;

Snapshot::SnapshotNode &Snapshot::SnapshotNode::operator=(const SnapshotNode &ss)
{
  state_ = ss.state_;
  if (ss.children_)
  {
    children_ = std::make_unique<std::array<SnapshotNode, 8>>();
    for (size_t i = 0; i < 8; ++i)
    {
      (*children_)[i] = SnapshotNode((*ss.children_)[i]);
    }
  }
  else
  {
    if (children_)
    {
      delete children_.release();
    }
  }
  return *this;
}

Snapshot::SnapshotRegion::SnapshotRegion(const MapChunk &ch, const OccupancyMap &map)
  : region_size_(map.regionSpatialResolution().x)
  , region_key_(ch.region.coord)
  , stamp_(ch.stamp())
{
  SnapshotSrcVoxel src_voxel(map);
  root_ = std::make_unique<Snapshot::SnapshotNode>(
    ch.region.coord, src_voxel, glm::u8vec3(0, 0, 0),
    glm::u8vec3(map.regionVoxelDimensions().x, map.regionVoxelDimensions().y, map.regionVoxelDimensions().z));
  const auto region_spatial_resolution = map.regionSpatialResolution();
  if (region_spatial_resolution.x != region_spatial_resolution.y ||
      region_spatial_resolution.x != region_spatial_resolution.z)
  {
    throw std::runtime_error(std::string("Non-isotropic region spatial resolution unsupported: ") +
                             std::to_string(region_spatial_resolution.x) + "," +
                             std::to_string(region_spatial_resolution.y) + "," +
                             std::to_string(region_spatial_resolution.z));
  }
}

Snapshot::SnapshotRegion::SnapshotRegion(const Snapshot::EncodedSnapshotRegion &region, float region_size)
  : region_size_(region_size)
  , region_key_(region.region_key)
  , stamp_(region.stamp)
{
  size_t byte_address = 0;
  uint8_t bit_address = 0;
  root_ = std::make_unique<Snapshot::SnapshotNode>(region, byte_address, bit_address);
}

Snapshot::SnapshotRegion::SnapshotRegion(const Snapshot::SnapshotRegion &ss)
  : root_(std::make_unique<Snapshot::SnapshotNode>(*ss.root_))
  , region_size_(ss.region_size_)
  , region_key_(ss.region_key_)
  , stamp_(ss.stamp_)
{}

void Snapshot::SnapshotRegion::getVoxels(size_t layer, std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const
{
  glm::f32vec3 ch_centre(region_key_.x * region_size_, region_key_.y * region_size_, region_key_.z * region_size_);
  root_->getVoxels(layer, region_size_, ch_centre, voxels);
}


Snapshot::EncodedSnapshotRegion Snapshot::SnapshotRegion::getRegion() const
{
  EncodedSnapshotRegion region;
  region.stamp = stamp_;
  region.region_key = region_key_;
  region.data.reserve((numNode() >> SnapshotNode::kStateBits) + 1);
  uint8_t bits_used_in_final_byte = 0;
  root_->getRegion(region, bits_used_in_final_byte);
  return region;
}


void Snapshot::SnapshotRegion::update(const Snapshot::SnapshotRegion &s2)
{
  root_->update(*s2.root_);
  simplify();
}


void Snapshot::SnapshotRegion::updateOccupancyMap(ohm::OccupancyMap &map) const
{
  ohm::MapChunk *region = map.region(region_key_, true);
  SnapshotVoxelBlock voxel(map, region);
  root_->updateOccupancyMap(
    region_key_, voxel, glm::u8vec3(0, 0, 0),
    glm::u8vec3(map.regionVoxelDimensions().x, map.regionVoxelDimensions().y, map.regionVoxelDimensions().z));
  const auto touch_stamp = map.touch();
  region->dirty_stamp = touch_stamp;
  region->touched_stamps[map.layout().occupancyLayer()].store(touch_stamp, std::memory_order_relaxed);
}


Snapshot::Snapshot()
: imp_(new SnapshotDetail{ nullptr })
{}


Snapshot::~Snapshot() = default;


Snapshot::EncodedSnapshot Snapshot::buildSnapshot(const ohm::Aabb &cull_to)
{
  EncodedSnapshot ss;

  if (!imp_->occupancy_map)
  {
    return ss;
  }

  PROFILE(buildSnapshot);

  const OccupancyMap &src_map = *imp_->occupancy_map;

  ss.stamp = src_map.stamp();
  const auto &origin = src_map.origin();
  ss.origin.x = static_cast<float>(origin.x);
  ss.origin.y = static_cast<float>(origin.y);
  ss.origin.z = static_cast<float>(origin.z);
  const auto &region_spatial_dimensions = src_map.regionSpatialResolution();
  ss.region_spatial_dimensions.x = static_cast<float>(region_spatial_dimensions.x);
  ss.region_spatial_dimensions.y = static_cast<float>(region_spatial_dimensions.y);
  ss.region_spatial_dimensions.z = static_cast<float>(region_spatial_dimensions.z);

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

  for (const auto ch_ptr : chunks)
  {
    const MapChunk &ch = *ch_ptr;
    if (!ch.overlapsExtents(src_region.minExtents(), src_region.maxExtents()))
    {
      continue;
    }

    SnapshotRegion ss_region(ch, src_map);
    ss_region.simplify();
    ss.regions.push_back(ss_region.getRegion());
  }

#if PROFILING
  ohm::Profile::instance().report();
#endif  // PROFILING

  return ss;
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



