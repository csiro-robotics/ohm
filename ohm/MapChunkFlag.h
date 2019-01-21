// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPCHUNKFLAG_H_
#define MAPCHUNKFLAG_H_

#include "OhmConfig.h"

namespace ohm
{
  enum MapChunkFlag
  {
    MainMemDirty = (1 << 0),
    GpuMemDirty = (1 << 1),
    SubVoxels = (1 << 2),
  };
}

#endif  // MAPCHUNKFLAG_H_
