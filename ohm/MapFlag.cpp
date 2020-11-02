// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapFlag.h"

#include <string>

namespace
{
const char *const kMapFlagNames[] =  //
  {
    //
    "VoxelMean",
  };

const unsigned kMapFlagsCount = unsigned(sizeof(kMapFlagNames) / sizeof(kMapFlagNames[0]));
}  // namespace

namespace ohm
{
const char *mapFlagToString(MapFlag flag)
{
  unsigned bit = 1;
  for (unsigned i = 0; i < kMapFlagsCount; ++i, bit <<= 1)
  {
    if (unsigned(flag) & bit)
    {
      return kMapFlagNames[i];
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
    if (name.compare(kMapFlagNames[i]) == 0)
    {
      return MapFlag(bit);
    }
  }

  return MapFlag::kNone;
}

}  // namespace ohm
