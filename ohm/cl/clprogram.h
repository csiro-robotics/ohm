// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CLPROGRAM_H_
#define CLPROGRAM_H_

#include "ohmconfig.h"

#include <clu/cl.hpp>
#include <clu/clukernel.h>

#include <string>
#include <vector>

namespace gputil
{
  class Device;
}

namespace ohm
{
  int initProgramFromSource(cl::Program &program, const gputil::Device &gpu, const char *sourceFileName,
                            const std::vector<std::string> *buildArgs = nullptr);

  int initProgramFromString(cl::Program &program, const gputil::Device &gpu, const char *sourceString,
                            const char *referenceName,
                            const std::vector<std::string> *buildArgs = nullptr);

  void calculateGrid(clu::KernelGrid &grid, clu::Kernel &kernel, const gputil::Device &gpu, const cl_int3 &calcExtents);
}

#endif // CLPROGRAM_H_
