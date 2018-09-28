// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYGPU_H_
#define OCCUPANCYGPU_H_

#include "ohmconfig.h"

namespace gputil
{
  class Device;
}

namespace ohm
{
  /// GPU/accelerator selection types.
  enum AccelType
  {
    GpuAccel = (1 << 0),
    CpuAccel = (1 << 1),
    AnyAccel = GpuAccel | CpuAccel
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
  /// @param showDevice Log selected device to standard output?
  /// @return 0 on success.
  int ohm_API configureGpuFromArgs(int argc, const char **argv, bool showDevice = true);

  /// @overload
  inline int ohm_API configureGpuFromArgs(int argc, char **argv, bool showDevice = true)
  {
    return configureGpuFromArgs(argc, (const char **)argv, showDevice);
  }


  /// Make a GPU/accelerator selection.
  ///
  /// @param accel Accelerator types allowed.
  /// @param deviceName Device name hint. Same as "--device" in @c configureGpuFromArgs().
  /// @return 0 on success.
  int ohm_API configureGpu(unsigned accel = GpuAccel, const char *deviceName = nullptr, bool showDevice = true);

  /// Get the GPU device to use for GPU based code in this library.
  /// @return A reference to the GPU device to use.
  gputil::Device &ohm_API gpuDevice();

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
  /// @param argsInfo Populated with pairs of command line arguments and their descriptions up to @c maxPairs items. Null to query the
  ///     number of arguments.
  /// @param maxPairs The number of pairs supported by the address at @c argsInfo.
  /// @param argType Array identifying the type for each argument.
  /// @return The total number of pairs available regardless of what has been written to @p argsInfo.
  unsigned ohm_API gpuArgsInfo(const char **argsInfo, int *argType, unsigned maxPairs);
}

#endif  // OCCUPANCYGPU_H_
