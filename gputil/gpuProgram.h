// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPROGRAM_H
#define GPUPROGRAM_H

#include "gpuConfig.h"

#include <string>
#include <vector>

namespace gputil
{
  class Device;
  struct ProgramDetail;

  struct BuildArgs
  {
    int version_major = -1;
    int version_minor = -1;
    int debug_level = 0;
    std::vector<std::string> *args;
  };

  /// Defines a compiled GPU program.
  ///
  /// This object has an empty CUDA implementation.
  class gputilAPI Program
  {
  public:
    Program();
    Program(Device &device, const char *program_name);
    Program(Program &&other);
    ~Program();

    bool isValid() const;

    const char *programName() const;

    Device device();

    int buildFromFile(const char *file_name, const BuildArgs &build_args);
    int buildFromSource(const char *source, size_t source_length, const BuildArgs &build_args);

    inline ProgramDetail *detail() { return imp_; }
    inline const ProgramDetail *detail() const { return imp_; }

    Program &operator=(Program &&other);

  private:
    ProgramDetail *imp_;
  };
}  // namespace gputil

#endif  // GPUPROGRAM_H
