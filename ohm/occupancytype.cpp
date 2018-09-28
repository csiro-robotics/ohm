// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancytype.h"

namespace ohm
{
  const char *occupancyTypeToString(int occupancyType)
  {
    const char *typeNames[] =
    {
      "null",
      "uncertain",
      "free",
      "occupied"
    };

    int index = occupancyType - ohm::Null;
    if (index >= 0 && index < sizeof(typeNames) / sizeof(typeNames[0]))
    {
      return typeNames[index];
    }

    return "<unknown>";
  }
}
