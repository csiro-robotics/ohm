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

#include <algorithm>
#include <cstring>

using namespace ohm;


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
    voxels.addMember(src_member.name, DataType::Type(src_member.type), src_member.clear_value);
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
    return MapLayoutMatch::Exact;
  }

  // Check the obvious first: number of layers and layer sizes.
  if (subsampling_ != other.subsampling_ || flags_ != other.flags_)
  {
    return MapLayoutMatch::Different;
  }

  if (voxel_layout_->next_offset != other.voxel_layout_->next_offset ||
      voxel_layout_->voxel_byte_size != other.voxel_layout_->voxel_byte_size ||
      voxel_layout_->members.size() != other.voxel_layout_->members.size())
  {
    return MapLayoutMatch::Different;
  }

  MapLayoutMatch match = MapLayoutMatch::Exact;

  if (name_ != other.name_)
  {
    // No name match. At best the layers are equivalent.
    match = MapLayoutMatch::Equivalent;
  }

  for (size_t i = 0; i < voxel_layout_->members.size(); ++i)
  {
    const VoxelMember &a = voxel_layout_->members[i];
    const VoxelMember &b = other.voxel_layout_->members[i];
    if (a.clear_value != b.clear_value || a.type != b.type || a.offset != b.offset)
    {
      return MapLayoutMatch::Different;
    }

    if (strncmp(a.name, b.name, sizeof(a.name)) != 0)
    {
      match = MapLayoutMatch::Equivalent;
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


void MapLayer::release(const uint8_t *voxels) const
{
  delete[] voxels;
}


void MapLayer::clear(uint8_t *mem, const glm::u8vec3 &region_dim) const
{
  // Build a clear pattern.
  uint8_t *pattern = static_cast<uint8_t *>(alloca(voxel_layout_->voxel_byte_size));
  uint8_t *dst = pattern;
  for (size_t i = 0; i < voxel_layout_->members.size(); ++i)
  {
    // Grab the current member.
    VoxelMember &member = voxel_layout_->members[i];
    // Work out the member size by the difference in offets to the next member or the end of the voxel.
    size_t member_size = ((i + 1 < voxel_layout_->members.size()) ? voxel_layout_->members[i + 1].offset :
                                                                    voxel_layout_->voxel_byte_size) -
                         member.offset;
    // Work out how may bytes to clear. Either the member size or the clear value size.
    const size_t clear_size = std::min(member_size, sizeof(member.clear_value));
    // Clear the bytes.
    memcpy(dst, &member.clear_value, clear_size);
    // Move to the next address.
    dst += member_size;
  }

  // We have built a clear pattern for the whole voxel. Now clear it.
  // Using volume() we deal with subsampling.
  const size_t voxel_count = volume(region_dim);
  dst = mem;
  for (size_t i = 0; i < voxel_count; ++i)
  {
    memcpy(dst, pattern, voxel_layout_->voxel_byte_size);
    dst += voxel_layout_->voxel_byte_size;
  }
}
