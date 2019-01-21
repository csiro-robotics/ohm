// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OccupancyType.h"

namespace ohm
{
  const char *occupancyTypeToString(int occupancy_type)
  {
    const char *type_names[] =  //
      {
        "null",       //
        "uncertain",  //
        "free",       //
        "occupied"    //
      };

    const int index = occupancy_type - ohm::Null;
    if (index >= 0 && unsigned(index) < sizeof(type_names) / sizeof(type_names[0]))
    {
      return type_names[index];
    }

    return "<unknown>";
  }
}  // namespace ohm
