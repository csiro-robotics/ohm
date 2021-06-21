// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapFlag.h"

#include <array>
#include <string>

namespace
{
const std::array<const char *, 3> kMapFlagNames =  //
  { "VoxelMean", "Compressed", "Traversal" };
}  // namespace

namespace ohm
{
const char *mapFlagToString(MapFlag flag)
{
  unsigned bit = 1;
  for (unsigned i = 0; i < kMapFlagNames.size(); ++i, bit <<= 1u)
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
  for (unsigned i = 0; i < kMapFlagNames.size(); ++i, bit <<= 1u)
  {
    if (name == kMapFlagNames[i])
    {
      return MapFlag(bit);
    }
  }

  return MapFlag::kNone;
}

}  // namespace ohm
