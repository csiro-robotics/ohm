// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmvoxellayout.h"

#include "private/voxellayoutdetail.h"

#include <cinttypes>

using namespace ohm;

template <typename T> const char *VoxelLayoutT<T>::memberName(size_t memberIndex) const
{
  if (memberIndex >= _detail->members.size())
  {
    return nullptr;
  }

  return _detail->members[memberIndex].name;
}


template <typename T> size_t VoxelLayoutT<T>::memberOffset(size_t memberIndex) const
{
  if (memberIndex >= _detail->members.size())
  {
    return 0;
  }

  return _detail->members[memberIndex].offset;
}


template <typename T> DataType::Type VoxelLayoutT<T>::memberType(size_t memberIndex) const
{
  if (memberIndex >= _detail->members.size())
  {
    return DataType::Unknown;
  }

  return (DataType::Type)_detail->members[memberIndex].type;
}


template <typename T> size_t VoxelLayoutT<T>::memberSize(size_t memberIndex) const
{
  if (memberIndex >= _detail->members.size())
  {
    return 0;
  }

  return DataType::size(_detail->members[memberIndex].type);
}


template <typename T> uint64_t VoxelLayoutT<T>::memberClearValue(size_t memberIndex) const
{
  return _detail->members[memberIndex].clearValue;
}


template <typename T> size_t VoxelLayoutT<T>::voxelByteSize() const
{
  return _detail->voxelByteSize;
}


template <typename T> size_t VoxelLayoutT<T>::memberCount() const
{
  return _detail->members.size();
}


template <typename T> const void *VoxelLayoutT<T>::memberPtr(size_t memberIndex, const void *mem) const
{
  if (memberIndex >= _detail->members.size())
  {
    return nullptr;
  }

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(mem);
  return &bytes[memberOffset(memberIndex)];
}


template <typename T> void *VoxelLayoutT<T>::memberPtr(size_t memberIndex, void *mem) const
{
  if (memberIndex >= _detail->members.size())
  {
    return nullptr;
  }

  uint8_t *bytes = reinterpret_cast<uint8_t *>(mem);
  return &bytes[memberOffset(memberIndex)];
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


void VoxelLayout::addMember(const char *name, DataType::Type type, uint64_t clearValue)
{
  VoxelMember member;
  strncpy(member.name, name, sizeof(member.name));
  member.name[sizeof(member.name) - 1] = '\0';
  member.clearValue = clearValue;
  member.type = type;
  member.offset = 0;

  // Resolve packing. The size dictates the expected alignment.
  uint16_t alignment = uint16_t(DataType::size(type));
  if (alignment == 0)
  {
    alignment = 4;
  }
  member.offset = _detail->nextOffset;

  // Is the proposed offset correctly aligned?
  if (member.offset % alignment != 0)
  {
    // Not aligned. Padd to the required alignment.
    member.offset += alignment - member.offset % alignment;
  }

  _detail->members.push_back(member);

  // Now fix up the voxel alignment and size.
  _detail->nextOffset = uint16_t(member.offset + DataType::size(type));

  // We allow alignments at 1, 2, 4, 8 bytes.
  // After that we must be 8 byte aligned. Hopefully this matches alignent of the compiler.
  switch (_detail->nextOffset)
  {
  case 1:
    _detail->voxelByteSize += 1;
    break;
  case 2:
    _detail->voxelByteSize += 2;
    break;
  case 3:
  case 4:
    _detail->voxelByteSize += 4;
    break;
  case 5:
  case 6:
  case 7:
  case 8:
    _detail->voxelByteSize += 8;
    break;

  default:
    // Next 8 byte alignment.
    _detail->voxelByteSize = _detail->nextOffset;
    break;
  }

  // Allow 4 byte voxel size, but force 8 byte alignment for larger voxels.
  if (_detail->nextOffset <= 4)
  {
    _detail->voxelByteSize = 4;
  }
  else
  {
    _detail->voxelByteSize = 8 * ((_detail->nextOffset + 7) / 8);
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
