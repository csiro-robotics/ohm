// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELBUFFER_H
#define VOXELBUFFER_H

#include "OhmConfig.h"

#include <cinttypes>
#include <type_traits>

namespace ohm
{
  class VoxelBlock;

  /// Base class for @c VoxelBuffer and @c VoxelBufferConst with common functionality between the two.
  ///
  /// The voxel buffer classes ensure voxel data from a @c VoxelBlock are retained in uncompressed form so long as the
  /// buffer object persists. The @c VoxelBlock data are releases, allowing for background compression once the buffer
  /// object is released/out of scope.
  template <typename VoxelBlock>
  class VoxelBuffer
  {
  public:
    using VoxelPtr = typename std::conditional<std::is_const<VoxelBlock>::value, const uint8_t *, uint8_t *>::type;

    inline VoxelBuffer() {}
    VoxelBuffer(ohm::VoxelBlock *block);
    VoxelBuffer(VoxelBuffer<VoxelBlock> &&other);
    VoxelBuffer(const VoxelBuffer<VoxelBlock> &other);
    ~VoxelBuffer();

    typename VoxelBuffer<VoxelBlock> &operator=(typename VoxelBuffer<VoxelBlock> &&other);
    typename VoxelBuffer<VoxelBlock> &operator=(const typename VoxelBuffer<VoxelBlock> &other);

    bool isValid() const { return voxel_memory_ != nullptr; }

    VoxelPtr voxelMemory() const { return voxel_memory_; }
    /// Query the uncompressed byte size of the voxel memory for the retained layer.
    size_t voxelMemorySize() const { return voxel_memory_size_; }
    VoxelBlock *voxelBlock() const { return voxel_block_; }

    /// Explicitly release the buffer. Further usage is invalid.
    void release();

  protected:
    VoxelPtr voxel_memory_{ nullptr };
    size_t voxel_memory_size_{ 0 };
    ohm::VoxelBlock *voxel_block_{ nullptr };
  };

  extern ohm_API template class ohm_API VoxelBuffer<VoxelBlock>;
  extern ohm_API template class ohm_API VoxelBuffer<const VoxelBlock>;
}  // namespace ohm

#endif  // VOXELBUFFER_H
