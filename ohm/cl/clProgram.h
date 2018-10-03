// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CLPROGRAM_H
#define CLPROGRAM_H

#include "OhmConfig.h"

#include <clu/cl.hpp>
#include <clu/cluKernel.h>

#include <string>
#include <vector>

namespace gputil
{
  class Device;
}

namespace ohm
{
  int initProgramFromSource(cl::Program &program, const gputil::Device &gpu, const char *source_file_name,
                            const std::vector<std::string> *build_args = nullptr);

  int initProgramFromString(cl::Program &program, const gputil::Device &gpu, const char *source_string,
                            const char *reference_name,
                            const std::vector<std::string> *build_args = nullptr);

  void calculateGrid(clu::KernelGrid &grid, clu::Kernel &kernel, const gputil::Device &gpu, const cl_int3 &calc_extents);
}

#endif // CLPROGRAM_H
