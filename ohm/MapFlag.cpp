// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapFlag.h"

#include <string>

namespace
{
  const char *map_flag_names[] = //
    { //
      "VoxelMean",
    };

  const unsigned kMapFlagsCount = unsigned(sizeof(map_flag_names) / sizeof(map_flag_names[0]));
} // namespace

namespace ohm
{
  const char *mapFlagToString(MapFlag flag)
  {
    unsigned bit = 1;
    for (unsigned i = 0; i < kMapFlagsCount; ++i, bit <<= 1)
    {
      if (unsigned(flag) & bit)
      {
        return map_flag_names[i];
      }
    }

    return "None";
  }


  MapFlag mapFlagFromString(const char *str)
  {
    std::string name(str);
    unsigned bit = 1;
    for (unsigned i = 0; i < kMapFlagsCount; ++i, bit <<= 1)
    {
      if (name.compare(map_flag_names[i]) == 0)
      {
        return MapFlag(bit);
      }
    }

    return MapFlag::kNone;
  }

} // namespace ohm
