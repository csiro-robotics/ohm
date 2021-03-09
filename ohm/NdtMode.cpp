// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "NdtMode.h"

#include <array>

namespace ohm
{
const std::array<std::string, 3> &ndtModeNames()
{
  static const std::array<std::string, 3> mode_names = { "none", "occupancy", "traversability" };
  return mode_names;
}

std::string ndtModeToString(NdtMode mode)
{
  if (unsigned(mode) < ndtModeNames().size())
  {
    return ndtModeNames()[int(mode)];
  }

  return "<unknown>";
}


NdtMode ndtModeFromString(const std::string &str)
{
  for (size_t i = 0; i < ndtModeNames().size(); ++i)
  {
    if (str == ndtModeNames()[i])
    {
      return NdtMode(i);
    }
  }

  return NdtMode::kNone;
}
}  // namespace ohm
