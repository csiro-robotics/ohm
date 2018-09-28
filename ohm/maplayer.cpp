// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "maplayer.h"

#include "mapchunk.h"
#include "occupancymap.h"
#include "ohmvoxellayout.h"

#include "private/maplayerdetail.h"
#include "private/voxellayoutdetail.h"

#include <glm/glm.hpp>

#include <algorithm>

using namespace ohm;


MapLayer::MapLayer(const char *name, unsigned layerIndex, unsigned subsampling)
  : _detail(new MapLayerDetail)
{
  _detail->name = name;
  _detail->voxelLayout = new VoxelLayoutDetail;
  _detail->layerIndex = layerIndex;
  _detail->subsampling = subsampling;
  _detail->flags = 0;
}


MapLayer::~MapLayer()
{
  delete _detail->voxelLayout;
  delete _detail;
}


void MapLayer::clear()
{
  _detail->voxelLayout->members.clear();
}


const char *MapLayer::name() const
{
  return _detail->name.c_str();
}


unsigned MapLayer::layerIndex() const
{
  return _detail->layerIndex;
}


unsigned MapLayer::subsampling() const
{
  return _detail->subsampling;
}


unsigned MapLayer::flags() const
{
  return _detail->flags;
}


void MapLayer::setFlags(unsigned flags)
{
  _detail->flags = flags;
}


void MapLayer::copyVoxelLayout(const MapLayer &other)
{
  VoxelLayout voxels = voxelLayout();
  for (auto &&srcMember : other._detail->voxelLayout->members)
  {
    voxels.addMember(srcMember.name, DataType::Type(srcMember.type), srcMember.clearValue);
  }
}


VoxelLayoutConst MapLayer::voxelLayout() const
{
  return VoxelLayoutConst(_detail->voxelLayout);
}


VoxelLayout MapLayer::voxelLayout()
{
  return VoxelLayout(_detail->voxelLayout);
}


glm::u8vec3 MapLayer::dimensions(const glm::u8vec3 &regionDim) const
{
  const glm::u8vec3 dim = regionDim / uint8_t(1 + _detail->subsampling);
  return glm::max(dim, glm::u8vec3(1));
}


size_t MapLayer::volume(const glm::u8vec3 &regionDim) const
{
  const glm::u8vec3 dim = dimensions(regionDim);
  return size_t(dim.x) * size_t(dim.y) * size_t(dim.z);
}


size_t MapLayer::voxelByteSize() const
{
  return _detail->voxelLayout->voxelByteSize;
}


size_t MapLayer::layerByteSize(const glm::u8vec3 &regionDim) const
{
  // Apply subsampling
  return volume(regionDim) * _detail->voxelLayout->voxelByteSize;
}


uint8_t *MapLayer::allocate(const glm::u8vec3 &regionDim) const
{
  return new uint8_t[layerByteSize(regionDim)];
}


void MapLayer::release(const uint8_t *voxels) const
{
  delete[] voxels;
}


void MapLayer::clear(uint8_t *mem, const glm::u8vec3 &regionDim) const
{
  // Build a clear pattern.
  uint8_t *pattern = (uint8_t *)alloca(_detail->voxelLayout->voxelByteSize);
  uint8_t *dst = pattern;
  for (size_t i = 0; i < _detail->voxelLayout->members.size(); ++i)
  {
    // Grab the current member.
    VoxelMember &member = _detail->voxelLayout->members[i];
    // Work out the member size by the difference in offets to the next member or the end of the voxel.
    size_t memberSize = ((i + 1 < _detail->voxelLayout->members.size()) ? _detail->voxelLayout->members[i + 1].offset :
                                                                          _detail->voxelLayout->voxelByteSize) -
                        member.offset;
    // Work out how may bytes to clear. Either the member size or the clear value size.
    size_t clearSize = std::min(memberSize, sizeof(member.clearValue));
    // Clear the bytes.
    memcpy(dst, &member.clearValue, clearSize);
    // Move to the next address.
    dst += memberSize;
  }

  // We have built a clear pattern for the whole voxel. Now clear it.
  // Using volume() we deal with subsampling.
  size_t voxelCount = volume(regionDim);
  dst = mem;
  for (size_t i = 0; i < voxelCount; ++i)
  {
    memcpy(dst, pattern, _detail->voxelLayout->voxelByteSize);
    dst += _detail->voxelLayout->voxelByteSize;
  }
}


const uint8_t *MapLayer::voxels(const MapChunk &chunk) const
{
  return chunk.voxelMaps[layerIndex()];
}


uint8_t *MapLayer::voxels(MapChunk &chunk) const
{
  return chunk.voxelMaps[layerIndex()];
}
