// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelBuffer.h"

#include "VoxelBlock.h"

using namespace ohm;

template <typename VoxelBlock>
VoxelBuffer<VoxelBlock>::VoxelBuffer(ohm::VoxelBlock *block)
  : voxel_block_(block)
{
  if (block)
  {
    block->retain();
    voxel_memory_size_ = block->uncompressedByteSize();
    voxel_memory_ = block->voxelBytes();
  }
}

template <typename VoxelBlock>
VoxelBuffer<VoxelBlock>::VoxelBuffer(VoxelBuffer<VoxelBlock> &&other)
  : voxel_memory_(std::exchange(other.voxel_memory_, nullptr))
  , voxel_memory_size_(std::exchange(other.voxel_memory_size_, 0))
  , voxel_block_(std::exchange(other.voxel_block_, nullptr))
{}

template <typename VoxelBlock>
VoxelBuffer<VoxelBlock>::VoxelBuffer(const VoxelBuffer<VoxelBlock> &other)
  : voxel_memory_(other.voxel_memory_)
  , voxel_memory_size_(other.voxel_memory_size_)
  , voxel_block_(other.voxel_block_)
{
  if (voxel_block_)
  {
    voxel_block_->retain();
  }
}

template <typename VoxelBlock>
VoxelBuffer<VoxelBlock>::~VoxelBuffer()
{
  release();
}

template <typename VoxelBlock>
typename VoxelBuffer<VoxelBlock> &VoxelBuffer<VoxelBlock>::operator=(VoxelBuffer<VoxelBlock> &&other)
{
  std::swap(voxel_block_, other.voxel_block_);
  std::swap(voxel_memory_size_, other.voxel_memory_size_);
  std::swap(voxel_memory_, other.voxel_memory_);
  return *this;
}

template <typename VoxelBlock>
typename VoxelBuffer<VoxelBlock> &VoxelBuffer<VoxelBlock>::operator=(const VoxelBuffer<VoxelBlock> &other)
{
  release();
  voxel_block_ = other.voxel_block_;
  if (voxel_block_)
  {
    voxel_block_->retain();
    voxel_memory_size_ = voxel_block_->uncompressedByteSize();
    voxel_memory_ = voxel_block_->voxelBytes();
  }
  return *this;
}

template <typename VoxelBlock>
void VoxelBuffer<VoxelBlock>::release()
{
  if (voxel_block_)
  {
    voxel_block_->release();
    voxel_block_ = nullptr;
    voxel_memory_ = nullptr;
    voxel_memory_size_ = 0;
  }
}

// Instantiate VoxelBufferT for const and non-const buffers.
template class VoxelBuffer<VoxelBlock>;
template class VoxelBuffer<const VoxelBlock>;
