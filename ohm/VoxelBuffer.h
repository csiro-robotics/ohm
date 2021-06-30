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
  inline VoxelBuffer() = default;
  /// Constructor wrapping data from the given @c block . Immediately calls @c ohm::VoxelBuffer::retain() .
  /// @param block A pointer to the block to retain. Must not be null.
  explicit VoxelBuffer(ohm::VoxelBlock *block);
  /// Overloaded constructor handling a @c std::unique_ptr wrapper around a @c ohm::VoxelBlock .
  /// @param block A unique pointer to the block to retain. Must not be null.
  inline explicit VoxelBuffer(const ohm::VoxelBlock::Ptr &block)
    : VoxelBuffer(block.get())
  {}
  /// RValue constructor.
  /// @param other The object to construct from.
  VoxelBuffer(VoxelBuffer<VoxelBlock> &&other) noexcept;
  /// Copy constructor constructor.
  /// @param other The object to construct from.
  VoxelBuffer(const VoxelBuffer<VoxelBlock> &other);

  /// Destructor, ensuring a call to @c ohm::VoxelBlock::release() for the wrapped block.
  ~VoxelBuffer();

  /// RValue assignment operator.
  /// @param other The object to assign from.
  /// @return `*this`
  VoxelBuffer<VoxelBlock> &operator=(VoxelBuffer<VoxelBlock> &&other) noexcept;

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

  /// Read the content for a voxel in the buffer. Must only be called if @c isValid() , @c voxel_index is in range
  /// and @c T is the contained data type, exactly matching the voxel data size.
  /// @param voxel_index The index of the voxel to read data for. Must be in range.
  /// @param[out] value The voxel content is written to this address.
  /// @tparam T The data type to read. Must exactly match the voxel size and content for the referenced voxel layer.
  template <typename T>
  void readVoxel(unsigned voxel_index, T *value)
  {
    memcpy(value, voxelMemory() + sizeof(T) * voxel_index, sizeof(T));
  }

  /// Write the content for a voxel in the buffer. Must only be called if @c isValid() , @c voxel_index is in range
  /// and @c T is the contained data type, exactly matching the voxel data size.
  /// @param voxel_index The index of the voxel to read data for. Must be in range.
  /// @param value The voxel content to write at @p voxel_index .
  /// @tparam T The data type to read. Must exactly match the voxel size and content for the referenced voxel layer.
  template <typename T>
  void writeVoxel(unsigned voxel_index, const T &value)
  {
    memcpy(voxelMemory() + sizeof(T) * voxel_index, &value, sizeof(T));
  }

  /// Explicitly release the buffer. Further usage is invalid and @c isValid() will return `false`.
  void release();

protected:
  VoxelPtr voxel_memory_{ nullptr };         ///< Pointer to the uncompressed voxel memory.
  size_t voxel_memory_size_{ 0 };            ///< Number of bytes referenced by the @c voxel_memory_ .
  ohm::VoxelBlock *voxel_block_{ nullptr };  ///< The @c VoxelBlock object owning the voxel memory.
};

extern template class VoxelBuffer<VoxelBlock>;
extern template class VoxelBuffer<const VoxelBlock>;
}  // namespace ohm

#endif  // VOXELBUFFER_H
