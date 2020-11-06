// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OccupancyType.h"

#include <array>

namespace ohm
{
const char *occupancyTypeToString(int occupancy_type)
{
  static const std::array<const char *, 4> type_names =  //
    {
      "null",       //
      "uncertain",  //
      "free",       //
      "occupied"    //
    };

  const int index = occupancy_type - ohm::kNull;
  if (index >= 0 && unsigned(index) < type_names.size())
  {
    return type_names[index];
  }

  return "<unknown>";
}
}  // namespace ohm
