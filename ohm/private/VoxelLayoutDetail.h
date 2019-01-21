// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_VOXELLAYOUTDETAIL_H
#define OHM_VOXELLAYOUTDETAIL_H

#include "OhmConfig.h"

#include <vector>

namespace ohm
{
  struct VoxelMember
  {
    char name[52];
    uint64_t clear_value;
    uint16_t type;
    uint16_t offset;
  };

  struct VoxelLayoutDetail
  {
    std::vector<VoxelMember> members;
    uint16_t next_offset = 0u;
    uint16_t voxel_byte_size = 0u;
  };
}  // namespace ohm

#endif  // OHM_VOXELLAYOUTDETAIL_H
