// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMMAPLAYOUTDETAIL_H_
#define OHMMAPLAYOUTDETAIL_H_

#include "ohmconfig.h"

#include "maplayer.h"

#include <string>
#include <vector>

namespace ohm
{
  struct MapLayoutDetail
  {
    std::vector<MapLayer *> layers;

    inline ~MapLayoutDetail() { clear(); }

    inline void clear()
    {
      for (MapLayer *layer : layers)
      {
        delete layer;
      }
      layers.clear();
    }
  };
}

#endif // OHMMAPLAYOUTDETAIL_H_
