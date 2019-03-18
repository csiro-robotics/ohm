// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohmgpu/OhmGpu.h>

#include <gtest/gtest.h>

#include "ohmtestcommon/OhmTestUtil.h"

int main(int argc, char **argv)
{
  ohmtestutil::setApplicationPath(argv[0]);

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
