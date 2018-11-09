// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "DefaultLayer.h"

namespace ohm
{
  const char *defaultLayerName(DefaultLayer layer)
  {
    const char *names[kDlCount] =
    {
      "occupancy",
      "clearance",
      "coarseClearance"
    };

    if (layer >= 0 && layer < kDlCount)
    {
      return names[layer];
    }

    return nullptr;
  }
}
