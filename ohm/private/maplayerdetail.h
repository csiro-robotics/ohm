// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMMAPLAYERDETAIL_H_
#define OHMMAPLAYERDETAIL_H_

#include "ohmconfig.h"

#include <string>

namespace ohm
{
  struct VoxelLayoutDetail;

  struct MapLayerDetail
  {
    std::string name;
    VoxelLayoutDetail *voxelLayout = nullptr;
    unsigned short layerIndex = 0;
    unsigned short subsampling = 0;
    unsigned flags = 0;
  };
}

#endif // OHMMAPLAYERDETAIL_H_
