// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMVOXELLAYOUTDETAIL_H_
#define OHMVOXELLAYOUTDETAIL_H_

#include "ohmconfig.h"

#include "maplayout.h"

#include <vector>

namespace ohm
{
  struct VoxelMember
  {
    char name[52];
    uint64_t clearValue;
    uint16_t type;
    uint16_t offset;
  };

  struct VoxelLayoutDetail
  {
    std::vector<VoxelMember> members;
    uint16_t nextOffset = 0u;
    uint16_t voxelByteSize = 0u;
  };
}

#endif // OHMVOXELLAYOUTDETAIL_H_
