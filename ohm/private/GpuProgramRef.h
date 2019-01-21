// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_GPUPROGRAMREF_H
#define OHM_GPUPROGRAMREF_H

#include "OhmConfig.h"

#include <gputil/gpuProgram.h>

#include <mutex>
#include <vector>

namespace gputil
{
  class Device;
}

namespace ohm
{
  /// A helper class for reference counting a gpu program.
  ///
  /// Usage:
  /// - Declare a static GpuProgramRef, initialising with either
  ///   - A source file path relative to the executable
  ///   - An inline source string
  /// - For each desired reference to the source, call addReference().
  ///   - E.g., in the constructor of the file using the program.
  /// - Call release when each reference is done with the program.
  ///   - E.g., in the constructor of the file using the program.
  class GpuProgramRef
  {
  public:
    enum SourceType
    {
      kSourceFile,
      kSourceString
    };

    GpuProgramRef(const char *name, SourceType source_type, const char *source_str, size_t source_str_length = 0u);
    GpuProgramRef(const char *name, SourceType source_type, const char *source_str, size_t source_str_length,
                  const std::initializer_list<std::string> &build_args);
    ~GpuProgramRef();

    inline gputil::Program &program() { return program_; }

    bool addReference(gputil::Device &gpu);
    void releaseReference();

    bool isValid();

  private:
    std::mutex program_mutex_;
    gputil::Program program_;
    volatile int program_ref_ = 0;
    std::string name_;
    std::string source_str_;
    SourceType source_type_;
    std::vector<std::string> build_args_;
  };
}  // namespace ohm

#endif  // OHM_GPUPROGRAMREF_H
