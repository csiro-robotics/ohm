// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPPERDETAIL_H_
#define MAPPERDETAIL_H_

#include "OhmConfig.h"

#include <vector>

namespace ohm
{
  class MappingProcess;
  class OccupancyMap;

  struct ohm_API MapperDetail
  {
    std::vector<MappingProcess *> processes;
    OccupancyMap *map = nullptr;
    unsigned next_process = 0;
  };
}  // namespace ohm

#endif  // MAPPERDETAIL_H_
