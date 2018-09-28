// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "safeio.h"

#include <cstdio>

#if !defined(_MSC_VER)

int fopen_s(FILE **file, const char *name, const char *mode)
{
  *file = fopen(name, mode);
  if (file)
  {
    return 0;
  }
  return 1;
}

#else  // !defined(_MSC_VER)

namespace
{
  int avoidNoSymbolsLinkWarning = 0;
}

#endif // !defined(_MSC_VER)
