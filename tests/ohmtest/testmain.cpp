// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancygpu.h"

#include <gtest/gtest.h>

#include <stdio.h> /* defines FILENAME_MAX */
#ifdef WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

namespace
{
  std::string appDir;
}

const char *dataRelPath()
{
  return (!appDir.empty()) ? appDir.c_str() : "";
}

int main(int argc, char **argv)
{
  appDir = argv[0];
  // Strip the executable name
  auto lastSeparator = appDir.find_last_of("/\\");
  if (lastSeparator != std::string::npos)
  {
    appDir = appDir.substr(0, lastSeparator + 1);
  }
  else
  {
    // Current dir.
    appDir.clear();
  }

  ::testing::InitGoogleTest(&argc, argv);
  ohm::configureGpuFromArgs(argc, argv, true);
  return RUN_ALL_TESTS();
}
