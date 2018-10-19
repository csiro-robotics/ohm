// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelLayout.h"

#include "private/VoxelLayoutDetail.h"

#include <cinttypes>
#include <cstring>

using namespace ohm;

template <typename T> const char *VoxelLayoutT<T>::memberName(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return nullptr;
  }

  return detail_->members[member_index].name;
}


template <typename T> size_t VoxelLayoutT<T>::memberOffset(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return 0;
  }

  return detail_->members[member_index].offset;
}


template <typename T> DataType::Type VoxelLayoutT<T>::memberType(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return DataType::kUnknown;
  }

  return DataType::Type(detail_->members[member_index].type);
}


template <typename T> size_t VoxelLayoutT<T>::memberSize(size_t member_index) const
{
  if (member_index >= detail_->members.size())
  {
    return 0;
  }

  return DataType::size(detail_->members[member_index].type);
}


template <typename T> uint64_t VoxelLayoutT<T>::memberClearValue(size_t member_index) const
{
  return detail_->members[member_index].clear_value;
}


template <typename T> size_t VoxelLayoutT<T>::voxelByteSize() const
{
  return detail_->voxel_byte_size;
}


template <typename T> size_t VoxelLayoutT<T>::memberCount() const
{
  return detail_->members.size();
}


template <typename T> const void *VoxelLayoutT<T>::memberPtr(size_t member_index, const void *mem) const
{
  if (member_index >= detail_->members.size())
  {
    return nullptr;
  }

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(mem);
  return &bytes[memberOffset(member_index)];
}


template <typename T> void *VoxelLayoutT<T>::memberPtr(size_t member_index, void *mem) const
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


VoxelLayout::VoxelLayout()
  : VoxelLayoutT<VoxelLayoutDetail>(nullptr)
{}


VoxelLayout::VoxelLayout(VoxelLayoutDetail *detail)
  : VoxelLayoutT<VoxelLayoutDetail>(detail)
{}


VoxelLayout::VoxelLayout(const VoxelLayout &other)
  : VoxelLayoutT<VoxelLayoutDetail>(other)
{}


void VoxelLayout::addMember(const char *name, DataType::Type type, uint64_t clear_value)
{
  VoxelMember member;
  strncpy(member.name, name, sizeof(member.name));
  member.name[sizeof(member.name) - 1] = '\0';
  member.clear_value = clear_value;
  member.type = type;
  member.offset = 0;

  // Resolve packing. The size dictates the expected alignment.
  uint16_t alignment = uint16_t(DataType::size(type));
  if (alignment == 0)
  {
    alignment = 4;
  }
  member.offset = detail_->next_offset;

  // Is the proposed offset correctly aligned?
  if (member.offset % alignment != 0)
  {
    // Not aligned. Padd to the required alignment.
    member.offset += alignment - member.offset % alignment;
  }

  detail_->members.push_back(member);

  // Now fix up the voxel alignment and size.
  detail_->next_offset = uint16_t(member.offset + DataType::size(type));

  // We allow alignments at 1, 2, 4, 8 bytes.
  // After that we must be 8 byte aligned. Hopefully this matches alignent of the compiler.
  switch (detail_->next_offset)
  {
  case 1:
    detail_->voxel_byte_size += 1;
    break;
  case 2:
    detail_->voxel_byte_size += 2;
    break;
  case 3:
  case 4:
    detail_->voxel_byte_size += 4;
    break;
  case 5:
  case 6:
  case 7:
  case 8:
    detail_->voxel_byte_size += 8;
    break;

  default:
    // Next 8 byte alignment.
    detail_->voxel_byte_size = detail_->next_offset;
    break;
  }

  // Allow 4 byte voxel size, but force 8 byte alignment for larger voxels.
  if (detail_->next_offset <= 4)
  {
    detail_->voxel_byte_size = 4;
  }
  else
  {
    detail_->voxel_byte_size = 8 * ((detail_->next_offset + 7) / 8);
  }
}


VoxelLayoutConst::VoxelLayoutConst()
  : VoxelLayoutT<const VoxelLayoutDetail>(nullptr)
{}


VoxelLayoutConst::VoxelLayoutConst(const VoxelLayoutDetail *detail)
  : VoxelLayoutT<const VoxelLayoutDetail>(detail)
{}


VoxelLayoutConst::VoxelLayoutConst(const VoxelLayoutConst &other)
  : VoxelLayoutT<const VoxelLayoutDetail>(other)
{}


VoxelLayoutConst::VoxelLayoutConst(const VoxelLayout &other)
  : VoxelLayoutT<const VoxelLayoutDetail>(other.detail())
{}
