// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gputhrow.h"

#include "gpuexception.h"

#include <iostream>

namespace gputil
{
  void log(const Exception &e)
  {
    std::cerr << "GPU Exception: " << e.message() << std::endl;
  }


  void log(const Exception &e, const char *file, int line)
  {
    std::cerr << file << "(" << line << "): GPU exception: " << e.message() << std::endl;
  }
}
