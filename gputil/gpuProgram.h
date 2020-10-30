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

/// Additional arguments specifying how to build a GPU program.
struct BuildArgs
{
  /// Target (major) API version to build for - e.g., the `1` in OpenCL 1.2. Optional.
  int version_major = -1;
  /// Target (minor) API version to build for - e.g., the `2` in OpenCL 1.2. Optional.
  int version_minor = -1;
  /// Specifies the level of debugging to enable. Increasing values increase the debug information, which varies
  /// depending on the underlying API and vendor. For CUDA, this is ignored as the debug level is set at C++
  /// compilation time. For OpenCL the levels are:
  /// - 0 : Disabled
  /// - 1 : Define `DEBUG` in the OpenCL code. For Intel, add '-g' option.
  /// - 2 : For Intel, add `-cl-opt-disable`
  ///
  /// Note that the @c Program uses the maximum of this value and the @c Device::debugGpu() value.
  int debug_level = 0;
  /// Additional arguments to pass to the GPU compiler.
  std::vector<std::string> *args = nullptr;
};

/// Defines a compiled GPU program.
///
/// This object has an empty CUDA implementation.
class gputilAPI Program
{
public:
  /// Create an empty/invalid program.
  Program();
  /// Create a program to run on @p device and refered to as @c program_name .
  /// @param device The @c Device to bind the program to.
  /// @param program_name The reference name for the program.
  Program(Device &device, const char *program_name);  // NOLINT(google-runtime-references)
  /// Move constructor.
  /// @param other The object to move.
  Program(Program &&other) noexcept;

  /// Destuctor - releases the GPU program.
  ~Program();

  /// Query the validity of this object.
  /// @return True if the program is valid.
  bool isValid() const;

  /// Query the program name.
  /// @return The program reference name.
  const char *programName() const;

  /// Query the @c Device the program is bound to.
  /// @return The program device.
  Device device();

  /// Build the program from the given source @p file_name .
  /// @param file_name Path to the source file - resolved on the given file system and working directory.
  /// @param build_args See @c BuildArgs .
  /// @return Zero on success or an SDK error code on failure.
  int buildFromFile(const char *file_name, const BuildArgs &build_args);
  /// Build the program from the given source string @p source .
  /// @param source The string containing the GPU source code.
  /// @param source_length The number of characters in @p source excluding the null terminator.
  /// @param build_args See @c BuildArgs .
  /// @return Zero on success or an SDK error code on failure.
  int buildFromSource(const char *source, size_t source_length, const BuildArgs &build_args);

  /// Internal program details.
  /// @return Internal details.
  inline ProgramDetail *detail() { return imp_; }
  /// Internal program details.
  /// @return Internal details.
  inline const ProgramDetail *detail() const { return imp_; }

  /// Copy assignment. Increments the program reference count.
  /// @param other The program to copy.
  /// @return `*this`
  Program &operator=(const Program &other);
  /// Move assignment.
  /// @param other The program to move.
  /// @return `*this`
  Program &operator=(Program &&other) noexcept;

private:
  ProgramDetail *imp_;
};
}  // namespace gputil

#endif  // GPUPROGRAM_H
