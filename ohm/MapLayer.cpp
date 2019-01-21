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

using namespace ohm;


MapLayer::MapLayer(const char *name, unsigned layer_index, unsigned subsampling)
  : imp_(new MapLayerDetail)
{
  imp_->name = name;
  imp_->voxel_layout = new VoxelLayoutDetail;
  imp_->layer_index = layer_index;
  imp_->subsampling = subsampling;
  imp_->flags = 0;
}


MapLayer::~MapLayer()
{
  delete imp_->voxel_layout;
  delete imp_;
}


void MapLayer::clear()
{
  imp_->voxel_layout->members.clear();
}


const char *MapLayer::name() const
{
  return imp_->name.c_str();
}


unsigned MapLayer::layerIndex() const
{
  return imp_->layer_index;
}


unsigned MapLayer::subsampling() const
{
  return imp_->subsampling;
}


unsigned MapLayer::flags() const
{
  return imp_->flags;
}


void MapLayer::setFlags(unsigned flags)
{
  imp_->flags = flags;
}


void MapLayer::copyVoxelLayout(const MapLayer &other)
{
  VoxelLayout voxels = voxelLayout();
  for (auto &&src_member : other.imp_->voxel_layout->members)
  {
    voxels.addMember(src_member.name, DataType::Type(src_member.type), src_member.clear_value);
  }
}


VoxelLayoutConst MapLayer::voxelLayout() const
{
  return VoxelLayoutConst(imp_->voxel_layout);
}


VoxelLayout MapLayer::voxelLayout()
{
  return VoxelLayout(imp_->voxel_layout);
}


glm::u8vec3 MapLayer::dimensions(const glm::u8vec3 &region_dim) const
{
  const glm::u8vec3 dim = region_dim / uint8_t(1 + imp_->subsampling);
  return glm::max(dim, glm::u8vec3(1));
}


size_t MapLayer::volume(const glm::u8vec3 &region_dim) const
{
  const glm::u8vec3 dim = dimensions(region_dim);
  return size_t(dim.x) * size_t(dim.y) * size_t(dim.z);
}


size_t MapLayer::voxelByteSize() const
{
  return imp_->voxel_layout->voxel_byte_size;
}


size_t MapLayer::layerByteSize(const glm::u8vec3 &region_dim) const
{
  // Apply subsampling
  return volume(region_dim) * imp_->voxel_layout->voxel_byte_size;
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
  uint8_t *pattern = static_cast<uint8_t *>(alloca(imp_->voxel_layout->voxel_byte_size));
  uint8_t *dst = pattern;
  for (size_t i = 0; i < imp_->voxel_layout->members.size(); ++i)
  {
    // Grab the current member.
    VoxelMember &member = imp_->voxel_layout->members[i];
    // Work out the member size by the difference in offets to the next member or the end of the voxel.
    size_t member_size = ((i + 1 < imp_->voxel_layout->members.size()) ? imp_->voxel_layout->members[i + 1].offset :
                                                                         imp_->voxel_layout->voxel_byte_size) -
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
    memcpy(dst, pattern, imp_->voxel_layout->voxel_byte_size);
    dst += imp_->voxel_layout->voxel_byte_size;
  }
}


const uint8_t *MapLayer::voxels(const MapChunk &chunk) const
{
  return chunk.voxel_maps[layerIndex()];
}


uint8_t *MapLayer::voxels(MapChunk &chunk) const
{
  return chunk.voxel_maps[layerIndex()];
}


void MapLayer::setLayerIndex(unsigned index)
{
  imp_->layer_index = index;
}
