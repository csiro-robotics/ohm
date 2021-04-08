// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapLayer.h"

#include "MapChunk.h"
#include "OccupancyMap.h"
#include "VoxelLayout.h"

#include "private/MapLayerDetail.h"
#include "private/VoxelLayoutDetail.h"

#include <glm/glm.hpp>

#include <cstring>

namespace ohm
{
MapLayer::MapLayer(const char *name, unsigned layer_index, unsigned subsampling)
{
  name_ = name;
  voxel_layout_ = std::make_unique<VoxelLayoutDetail>();
  layer_index_ = layer_index;
  subsampling_ = subsampling;
  flags_ = 0;
}


MapLayer::~MapLayer() = default;


void MapLayer::clear()
{
  voxel_layout_->members.clear();
}


void MapLayer::copyVoxelLayout(const MapLayer &other)
{
  VoxelLayout voxels = voxelLayout();
  for (auto &&src_member : other.voxel_layout_->members)
  {
    voxels.addMember(src_member.name.data(), DataType::Type(src_member.type), src_member.clear_value);
  }
}


VoxelLayoutConst MapLayer::voxelLayout() const
{
  return VoxelLayoutConst(voxel_layout_.get());
}


VoxelLayout MapLayer::voxelLayout()
{
  return VoxelLayout(voxel_layout_.get());
}


MapLayoutMatch MapLayer::checkEquivalent(const MapLayer &other) const
{
  if (this == &other)
  {
    return MapLayoutMatch::kExact;
  }

  // Check the obvious first: number of layers and layer sizes.
  if (subsampling_ != other.subsampling_ || flags_ != other.flags_)
  {
    return MapLayoutMatch::kDifferent;
  }

  if (voxel_layout_->next_offset != other.voxel_layout_->next_offset ||
      voxel_layout_->voxel_byte_size != other.voxel_layout_->voxel_byte_size ||
      voxel_layout_->members.size() != other.voxel_layout_->members.size())
  {
    return MapLayoutMatch::kDifferent;
  }

  MapLayoutMatch match = MapLayoutMatch::kExact;

  if (name_ != other.name_)
  {
    // No name match. At best the layers are equivalent.
    match = MapLayoutMatch::kEquivalent;
  }

  for (size_t i = 0; i < voxel_layout_->members.size(); ++i)
  {
    const VoxelMember &a = voxel_layout_->members[i];
    const VoxelMember &b = other.voxel_layout_->members[i];
    if (a.clear_value != b.clear_value || a.type != b.type || a.offset != b.offset)
    {
      return MapLayoutMatch::kDifferent;
    }

    if (strncmp(a.name.data(), b.name.data(), a.name.size()) != 0)
    {
      match = MapLayoutMatch::kEquivalent;
    }
  }

  return match;
}

size_t MapLayer::voxelByteSize() const
{
  return voxel_layout_->voxel_byte_size;
}


size_t MapLayer::layerByteSize(const glm::u8vec3 &region_dim) const
{
  // Apply subsampling
  return volume(region_dim) * voxel_layout_->voxel_byte_size;
}


uint8_t *MapLayer::allocate(const glm::u8vec3 &region_dim) const
{
  return new uint8_t[layerByteSize(region_dim)];
}


void MapLayer::release(const uint8_t *voxels)
{
  delete[] voxels;
}


void MapLayer::clear(uint8_t *mem, const glm::u8vec3 &region_dim) const
{
  const glm::u8vec3 layer_dim = dimensions(region_dim);
  // Build a clear pattern. We progressively copy more and more memory.
  // - first build a single voxel at mem
  // - copy voxel pattern from mem to create layer_dim.x elements at mem (rows)
  // - copy first row into layer_dim.y - 1 rows to create layers
  // - copy first layer into layer_dim.z - 1 layers
  //
  // We also have a special case when the clear pattern is zero. In that case we use a single memset call.
  const size_t voxel_byte_size = voxel_layout_->voxel_byte_size;

  // Write one voxel into the destination location.
  uint8_t *dst = mem;
  bool zero_clear_pattern = true;
  for (size_t i = 0; i < voxel_layout_->members.size(); ++i)
  {
    // Grab the current member.
    VoxelMember &member = voxel_layout_->members[i];
    // Work out the member size by the difference in offets to the next member or the end of the voxel.
    size_t member_size =
      ((i + 1 < voxel_layout_->members.size()) ? voxel_layout_->members[i + 1].offset : voxel_byte_size) -
      member.offset;
    // Work out how may bytes to clear. Either the member size or the clear value size.
    const size_t clear_size = std::min(member_size, sizeof(member.clear_value));
    zero_clear_pattern = zero_clear_pattern && member.clear_value == 0;
    // Clear the bytes.
    memcpy(dst, &member.clear_value, clear_size);
    // Move to the next address.
    dst += member_size;
  }

  if (zero_clear_pattern)
  {
    // Voxel clear pattern is all zero. Just use a memset.
    memset(mem, 0, voxel_byte_size * layer_dim.x * layer_dim.y * layer_dim.z);
    return;
  }

  // Fill out a single voxel column (layer_dim.x)
  for (int x = 1; x < layer_dim.x; ++x, dst += voxel_byte_size)
  {
    memcpy(dst, mem, voxel_byte_size);
  }

  // Now repeat the column into the rows (layer_dim.y)
  for (int y = 1; y < layer_dim.y; ++y, dst += voxel_byte_size * layer_dim.x)
  {
    memcpy(dst, mem, voxel_byte_size * layer_dim.x);
  }

  // Now copy each 2D layer.
  for (int z = 1; z < layer_dim.z; ++z, dst += voxel_byte_size * layer_dim.x * layer_dim.y)
  {
    memcpy(dst, mem, voxel_byte_size * layer_dim.x * layer_dim.y);
  }

  assert(dst == mem + volume(region_dim) * voxel_byte_size);
}
}  // namespace ohm
