// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelLayout.h"

#include "private/VoxelLayoutDetail.h"

#include <cinttypes>
#include <cstring>
#include <string>

using namespace ohm;

template <typename T>
const char *VoxelLayoutT<T>::memberName(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return nullptr;
  }

  return detail_->members[member_index].name;
}


template <typename T>
size_t VoxelLayoutT<T>::memberOffset(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return 0;
  }

  return detail_->members[member_index].offset;
}


template <typename T>
DataType::Type VoxelLayoutT<T>::memberType(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return DataType::kUnknown;
  }

  return DataType::Type(detail_->members[member_index].type);
}


template <typename T>
size_t VoxelLayoutT<T>::memberSize(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return 0;
  }

  return DataType::size(detail_->members[member_index].type);
}


template <typename T>
uint64_t VoxelLayoutT<T>::memberClearValue(size_t member_index) const
{
  return detail_->members[member_index].clear_value;
}


template <typename T>
size_t VoxelLayoutT<T>::voxelByteSize() const
{
  return detail_->voxel_byte_size;
}


template <typename T>
size_t VoxelLayoutT<T>::memberCount() const
{
  return detail_->members.size();
}


template <typename T>
const void *VoxelLayoutT<T>::memberPtr(size_t member_index, const void *mem) const
{
  if (member_index >= detail_->members.size())
  {
    return nullptr;
  }

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(mem);
  return &bytes[memberOffset(member_index)];
}


template <typename T>
void *VoxelLayoutT<T>::memberPtr(size_t member_index, void *mem) const
{
  if (member_index >= detail_->members.size())
  {
    return nullptr;
  }

  uint8_t *bytes = reinterpret_cast<uint8_t *>(mem);
  return &bytes[memberOffset(member_index)];
}


namespace ohm
{
  template class VoxelLayoutT<VoxelLayoutDetail>;
  template class VoxelLayoutT<const VoxelLayoutDetail>;
}  // namespace ohm


namespace
{
  /// Determine the byte alignment for a type of size @p data_size to one of {1, 2, 4, 8}.
  /// @param data_size The byte size of the data size of interest.
  /// @return The best alignment value >= @c base_alignment from {1, 2, 4} or 8 when @p data_size >= 88.
  uint16_t getAlignmentForSize(uint16_t data_size)
  {
    // Support 1, 2, 4, 8 byte alignments.
    switch (data_size)
    {
    case 1:
      return 1;
    case 2:
      return 2;
    case 3:
    case 4:
      return 4;
    default:
      break;
    }

    return 8;
  }

  /// A helper function for updating the @c VoxelLayoutDetail and @c VoxelMember offset, alignment and size values.
  ///
  /// This function sets the @c VoxelMember::offset and updates @c VoxelLayoutDetail::next_offset and
  /// @c VoxelLayoutDetail::voxel_byte_size based on the @pcVoxelLayoutDetail::next_offset and@c VoxelMember::type size.
  ///
  /// The @c VoxelMember::offset is set to @p VoxelLayoutDetail::next_offset, the rounded up to the nearest suitable
  /// alignment for the data size (base don @c getAlignmentForSize()). This yields an alignment of one of {1, 2, 4, 8}.
  /// The @c VoxelLayoutDetail::voxel_byte_size is the sum of the member offset and it's data size aligned to the
  /// nearest 8 byte boundary with the exception of sizes [1, 4], for which the data size is 4 bytes.
  ///
  /// @param[in,out] detail The @c VoxelLayoutDetail to update the @c next_offset and @c voxel_byte_size for.
  /// @param[in,out] member The member to set the @c offset for.
  void updateOffsets(VoxelLayoutDetail *detail, VoxelMember *member)
  {
    uint16_t member_size = uint16_t(DataType::size(member->type));

    // Resolve packing. Member size dictates the required alignment at the next increment of 1, 2, 4 or 8 bytes.
    uint16_t alignment = getAlignmentForSize(member_size);
    if (alignment == 0)
    {
      alignment = 4;
    }
    // Set the initial member offset to the next target offset.
    member->offset = detail->next_offset;

    // Is the proposed offset correctly aligned?
    if (member->offset % alignment != 0)
    {
      // Not aligned. Pad to the required alignment.
      member->offset += alignment - member->offset % alignment;
    }

    // Adjust the next offset to the member offset + the size.
    detail->next_offset = uint16_t(member->offset + member_size);

    // Now we calculate the aligned voxel size to be aligned to 4 or 8 bytes.
    detail->voxel_byte_size = detail->next_offset;
    if (detail->voxel_byte_size <= 4)
    {
      detail->voxel_byte_size = 4;
    }
    else
    {
      detail->voxel_byte_size = 8 * ((detail->next_offset + 7) / 8);
    }
  }
}  // namespace

VoxelLayout::VoxelLayout()
  : VoxelLayoutT<VoxelLayoutDetail>(nullptr)
{}


VoxelLayout::VoxelLayout(VoxelLayoutDetail *detail)
  : VoxelLayoutT<VoxelLayoutDetail>(detail)
{}


VoxelLayout::VoxelLayout(const VoxelLayout &other) = default;


void VoxelLayout::addMember(const char *name, DataType::Type type, uint64_t clear_value)
{
  VoxelMember member{};
  strncpy(member.name, name, sizeof(member.name));
  member.name[sizeof(member.name) - 1] = '\0';
  member.clear_value = clear_value;
  member.type = type;
  member.offset = 0;

  updateOffsets(detail_, &member);
  detail_->members.push_back(member);
}


bool VoxelLayout::removeMember(const char *name)
{
  std::string name_str(name);
  bool removed = false;

  // Recalculate next offset and voxel size as we go.
  detail_->next_offset = detail_->voxel_byte_size = 0;
  for (auto iter = detail_->members.begin(); iter != detail_->members.end();)
  {
    VoxelMember &member = *iter;
    if (name_str.compare(member.name) == 0)
    {
      iter = detail_->members.erase(iter);
      removed = true;
    }
    else
    {
      // Update offsets.
      member.offset = detail_->next_offset;
      updateOffsets(detail_, &member);
      ++iter;
    }
  }

  return removed;
}


VoxelLayoutConst::VoxelLayoutConst()
  : VoxelLayoutT<const VoxelLayoutDetail>(nullptr)
{}


VoxelLayoutConst::VoxelLayoutConst(const VoxelLayoutDetail *detail)
  : VoxelLayoutT<const VoxelLayoutDetail>(detail)
{}


VoxelLayoutConst::VoxelLayoutConst(const VoxelLayoutConst &other) = default;


VoxelLayoutConst::VoxelLayoutConst(const VoxelLayout &other)
  : VoxelLayoutT<const VoxelLayoutDetail>(other.detail())
{}
