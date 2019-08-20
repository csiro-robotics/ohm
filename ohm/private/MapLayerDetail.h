// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPLAYERDETAIL_H
#define OHM_MAPLAYERDETAIL_H

#include "OhmConfig.h"

#include <string>

namespace ohm
{
  struct VoxelLayoutDetail;

  struct ohm_API MapLayerDetail
  {
    std::string name;
    VoxelLayoutDetail *voxel_layout = nullptr;
    uint16_t layer_index = 0;
    uint16_t subsampling = 0;
    unsigned flags = 0;
  };
}  // namespace ohm

#endif  // OHM_MAPLAYERDETAIL_H
