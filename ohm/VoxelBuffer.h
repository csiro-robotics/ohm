// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELBUFFER_H
#define VOXELBUFFER_H

#include "OhmConfig.h"

#include "VoxelBlock.h"

#include <cinttypes>
#include <memory>
#include <type_traits>

namespace ohm
{
  /// Base class for @c VoxelBuffer and @c VoxelBufferConst with common functionality between the two.
  ///
  /// The voxel buffer classes ensure voxel data from a @c VoxelBlock are retained in uncompressed form so long as the
  /// buffer object persists. The @c VoxelBlock data are releases, allowing for background compression once the buffer
  /// object is released/out of scope.
  template <typename VoxelBlock>
  class VoxelBuffer
  {
  public:
    /// Define the pointer to the voxel memory. This is `const` if @c VoxelBlock is a const template argument.
    using VoxelPtr = typename std::conditional<std::is_const<VoxelBlock>::value, const uint8_t *, uint8_t *>::type;

    /// Default constructor : creates an invalid buffer reference object.
    inline VoxelBuffer() {}
    /// Constructor wrapping data from the given @c block . Immediately calls @c ohm::VoxelBuffer::retain() .
    /// @param block A pointer to the block to retain. Must not be null.
    VoxelBuffer(ohm::VoxelBlock *block);
    /// Overloaded constructor handling a @c std::unique_ptr wrapper around a @c ohm::VoxelBlock .
    /// @param block A unique pointer to the block to retain. Must not be null.
    inline VoxelBuffer(const ohm::VoxelBlock::Ptr &block)
      : VoxelBuffer(block.get())
    {}
    /// RValue constructor.
    /// @param other The object to construct from.
    VoxelBuffer(VoxelBuffer<VoxelBlock> &&other);
    /// Copy constructor constructor.
    /// @param other The object to construct from.
    VoxelBuffer(const VoxelBuffer<VoxelBlock> &other);

    /// Destructor, ensuring a call to @c ohm::VoxelBlock::release() for the wrapped block.
    ~VoxelBuffer();

    /// RValue assignment operator.
    /// @param other The object to assign from.
    /// @return `*this`
    VoxelBuffer<VoxelBlock> &operator=(VoxelBuffer<VoxelBlock> &&other);

    /// Copy assignment operator.
    /// @param other The object to assign from
    /// @return `*this`
    VoxelBuffer<VoxelBlock> &operator=(const VoxelBuffer<VoxelBlock> &other);

    /// Does the current object reference a valid @c ohm::VoxelBlock ?
    /// @return True if referencing a valid object.
    bool isValid() const { return voxel_memory_ != nullptr; }

    /// Access the raw memory stored by the wrapped @c VoxelBlock . The returned point is `const` when the template type
    /// is `const`.
    ///
    /// Must only be called if @c isValid() is `true`.
    /// @return A pointer to the referenced voxel memory.
    VoxelPtr voxelMemory() const { return voxel_memory_; }
    /// Query the uncompressed byte size of the voxel memory for the retained layer.
    /// @return The uncompressed size for the voxel memory, or zero when @c isValid() is `false`.
    size_t voxelMemorySize() const { return voxel_memory_size_; }
    /// Access the wrapped @c VoxelBlock pointer.
    /// @return The wrapped object pointer.
    VoxelBlock *voxelBlock() const { return voxel_block_; }

    /// Explicitly release the buffer. Further usage is invalid and @c isValid() will return `false`.
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
