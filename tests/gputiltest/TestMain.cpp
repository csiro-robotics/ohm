// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gputil/gpuDevice.h>

#include <gtest/gtest.h>

gputil::Device g_gpu;

int main(int argc, char **argv)
{
  g_gpu.select(argc, argv);
  if (!g_gpu.isValid())
  {
    std::cerr << "GPU initialisation failed." << std::endl;
    return -1;
  }
  ::testing::InitGoogleTest(&argc, argv);
  std::cout << g_gpu.description() << std::endl;
  return RUN_ALL_TESTS();
}
