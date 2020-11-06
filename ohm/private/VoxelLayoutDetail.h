// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_VOXELLAYOUTDETAIL_H
#define OHM_VOXELLAYOUTDETAIL_H

#include "OhmConfig.h"

#include <array>
#include <vector>

namespace ohm
{
struct ohm_API VoxelMember
{
  /// Voxel member name field. Sized to set the overall structure size to 64 bytes.
  std::array<char, 52> name;  // NOLINT(readability-magic-numbers)
  uint64_t clear_value;
  uint16_t type;
  uint16_t offset;
};

struct ohm_API VoxelLayoutDetail
{
  std::vector<VoxelMember> members;
  uint16_t next_offset = 0u;
  uint16_t voxel_byte_size = 0u;
};
}  // namespace ohm

#endif  // OHM_VOXELLAYOUTDETAIL_H
