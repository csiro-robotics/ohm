// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPUPROGRAMREF_H
#define OHMGPU_GPUPROGRAMREF_H

#include "OhmGpuConfig.h"

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
    /// Type of source being used/referenced
    enum SourceType
    {
      /// Null value. For internal use.
      kSourceNull,
      /// Loading source from a file name.
      kSourceFile,
      /// Building from a provided source string.
      kSourceString
    };

    /// Create a program reference.
    /// @param name Program's reference name.
    /// @param source_type Identified how to treat @p source_str
    /// @param source_str Either a file name or string containing to code to build.
    /// @param source_str_length Optional length of @p source_str.
    GpuProgramRef(const char *name, SourceType source_type, const char *source_str, size_t source_str_length = 0u);
    /// Create a program reference with build arguments.
    /// @param name Program's reference name.
    /// @param source_type Identified how to treat @p source_str
    /// @param source_str Either a file name or string containing to code to build.
    /// @param source_str_length Optional length of @p source_str.
    /// @param build_args Additional build arguments.
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

#endif  // OHMGPU_GPUPROGRAMREF_H
