// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapMode.h"

#include <array>

namespace ohm
{
namespace
{
static const std::array<std::string, 4> mode_names =  //
  {
    //
    "planar",             //
    "fill",               //
    "layered-unordered",  //
    "layered",            //
  };
}

std::string heightmapModeToString(HeightmapMode mode)
{
  if (size_t(mode) < mode_names.size())
  {
    return mode_names[int(mode)];
  }

  return "<unknown>";
}
HeightmapMode heightmapModeFromString(const std::string &str, bool *valid_string)
{
  int mode_index = 0;
  for (const auto &name : mode_names)
  {
    if (str == name)
    {
      if (valid_string)
      {
        *valid_string = true;
      }
      return HeightmapMode(mode_index);
    }
    ++mode_index;
  }

  if (valid_string)
  {
    *valid_string = false;
  }
  return HeightmapMode::kPlanar;
}
}  // namespace ohm
