// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_H
#define OHMGPU_H

#include "OhmGpuConfig.h"

namespace gputil
{
  struct BuildArgs;
  class Device;
}  // namespace gputil

namespace ohm
{
  /// GPU/accelerator selection types.
  enum AccelType
  {
    kGpuAccel = (1 << 0),
    kCpuAccel = (1 << 1),
    kAnyAccel = kGpuAccel | kCpuAccel
  };

  /// Make a GPU/accelerator selection from command line arguments.
  ///
  /// May be given the entire set of command line arguments, but it will only respect the following:
  /// - --accel=[any,cpu,gpu] : choose accelerator type.
  /// - --platform=<hint> : platform name hint. Partial, lower case match is enough.
  /// - --device=<hint> : device name hint. Partial, lower case match is enough.
  /// - --clver=<version> : Minimum OpenCL version string; e.g., "1.2", "2.0" "2".
  /// - --gpu-debug : compile GPU code for debugging (Intel OpenCL)
  ///
  /// @param argc Number of arguments in @p argv.
  /// @param argv Command line arguments.
  /// @param show_device Log selected device to standard output?
  /// @return 0 on success.
  int ohmgpu_API configureGpuFromArgs(int argc, const char **argv, bool show_device = true);

  /// @overload
  inline int ohmgpu_API configureGpuFromArgs(int argc, char **argv, bool show_device = true)
  {
    return configureGpuFromArgs(argc, const_cast<const char **>(argv),  // NOLINT(cppcoreguidelines-pro-type-const-cast)
                                show_device);
  }

  /// Make a GPU/accelerator selection.
  ///
  /// @param accel Accelerator types allowed.
  /// @param device_name Device name hint. Same as "--device" in @c configureGpuFromArgs().
  /// @param show_device Print the selected device to stdout.
  /// @return 0 on success.
  int ohmgpu_API configureGpu(unsigned accel = kGpuAccel, const char *device_name = nullptr, bool show_device = true);

  /// Get the GPU device to use for GPU based code in this library.
  /// @return A reference to the GPU device to use.
  gputil::Device &ohmgpu_API gpuDevice();

  /// Provides information about the available command line options which control GPU behaviour.
  ///
  /// This populates @p argsInfo with an array of static string pointers arranges in pairs. The pairs
  /// specify a valid GPU command line option, followed by its English description.
  /// The number of pairs is given by the return value. Arguments are given without any command line prefix
  /// (e.g. "--" or "/"), and only with one supporting form (long form).
  ///
  /// The function may be called with @p argsInfo as @c nullptr in order to query the number of pairs.
  ///
  /// Each argument type is identified in @c argType if given. Types are:
  /// - 0 : switch (boolean)
  /// - 1 : string
  ///
  /// @param args_info Populated with pairs of command line arguments and their descriptions up to @c maxPairs items.
  /// Null to query the
  ///     number of arguments.
  /// @param max_pairs The number of pairs supported by the address at @c argsInfo.
  /// @param arg_type Array identifying the type for each argument.
  /// @return The total number of pairs available regardless of what has been written to @p argsInfo.
  unsigned ohmgpu_API gpuArgsInfo(const char **args_info, int *arg_type, unsigned max_pairs);

  /// GPU compilation string used to define the required GPU code standard.
  ///
  /// Used to set "-cl-std=x.x" in OpenCL compilation. Validated required extended features.
  ///
  /// This method will be deprecated once all kernels are migrated to use gputil::Program and gputil::Kernel.
  ///
  /// @return An argument string which should be included when building GPU code.
  const char ohmgpu_API *gpuBuildStdArg();

  /// Set the GPU target versions in @p build_args to define the required GPU code standard.
  ///
  /// Used to set "-cl-std=x.x" in OpenCL compilation. Validated required extended features.
  ///
  /// @return An argument string which should be included when building GPU code.
  void ohmgpu_API setGpuBuildVersion(gputil::BuildArgs &build_args);  // NOLINT(google-runtime-references)
}  // namespace ohm

#endif  // OHMGPU_H
