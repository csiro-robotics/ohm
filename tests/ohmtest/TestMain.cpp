// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/OhmGpu.h>

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
  std::string app_dir;
}  // namespace

const char *dataRelPath()
{
  return (!app_dir.empty()) ? app_dir.c_str() : "";
}

int main(int argc, char **argv)
{
  app_dir = argv[0];
  // Strip the executable name
  auto lastSeparator = app_dir.find_last_of("/\\");
  if (lastSeparator != std::string::npos)
  {
    app_dir = app_dir.substr(0, lastSeparator + 1);
  }
  else
  {
    // Current dir.
    app_dir.clear();
  }

  ::testing::InitGoogleTest(&argc, argv);
  int err = ohm::configureGpuFromArgs(argc, argv, true);
  if (err)
  {
    std::cerr << "failed to initialise GPU" << std::endl;
    return err;
  }
  err = RUN_ALL_TESTS();
  return err;
}
