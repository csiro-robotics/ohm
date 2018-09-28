// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUDEVICE_H_
#define GPUDEVICE_H_

#include "gpuconfig.h"
#include "gpuqueue.h"

#include <cinttypes>

namespace gputil
{
  struct DeviceDetail;

  /// Represents a GPU device and context.
  class gputilAPI Device
  {
  public:
    /// Specifies the level of debugging applied to the GPU.
    enum DebugLevel
    {
      /// No debugging.
      DL_Off,
      /// Minimal debugging. Optimisations are still active.
      DL_Low,
      /// Full debugging, optimisations disabled.
      DL_Full
    };

    /// Construct to access default device or as an invalid device.
    /// @param defaultDevice True to construct access the default device.
    Device(bool defaultDevice = false);

    /// Construct using @c select().
    /// @param argc Number of values in @c argv.
    /// @param argv Argument string to parse.
    Device(int argc, const char **argv, const char *defaultDevice = nullptr);
    /// @overload
    inline Device(int argc, char **argv, const char *defaultDevice = nullptr) : Device(argc, (const char **)argv, defaultDevice) {}

    /// Copy constructor.
    /// @param other Object to copy.
    Device(const Device &other);

    /// Move constructor.
    /// @param other Object to move.
    Device(Device &&other);

    /// Destructor.
    ~Device();

    /// Display name for the device.
    const char *name() const;

    /// Device info.
    const char *info() const;

    /// Get the default queue associated with this device.
    /// @return The default device queue.
    Queue defaultQueue() const;

    /// Create a new queue for this device.
    /// @param Creation flags for the queue. See @c Queue::QueueFlag.
    /// @return The new queue.
    Queue createQueue(unsigned flags = 0) const;

    /// Select an active device based on the given command line (style) arguments.
    ///
    /// TODO: More documentaiton here.
    ///
    /// For OpenCL you can do the following:
    /// - "--accel=[any,accel,gpu,cpu]" to select the accelerator type.
    /// - "--clver=#.#" to select a minimum required OpenCL version.
    /// - "--device=&lt;like-name&gt;" for a device approximately matching the given name.
    /// - "--platform=&lt;like-name&gt;" for a plaform approximately matching the given name.
    /// - "--vendor=&lt;like-name&gt;" for a vendor approximately matching the given name.
    /// - "--gpu-debug[=&lt;level&gt;]" set the @c debugGpu() level. Uses @c DL_Low (1) if no level is specified.
    ///
    /// @param argc Number of values in @c argv.
    /// @param argv Argument string to parse.
    /// @param defaultDevice Default device constraint to select if non specified.
    /// @return True on success.
    bool select(int argc, const char **argv, const char *defaultDevice = nullptr);

    /// @overload
    inline bool select(int argc, char **argv, const char *defaultDevice = nullptr) { return select(argc, (const char **)argv, defaultDevice); }

    /// Set the value for @c debugGpu().
    /// @param debug The @c DebugLevel to set.
    void setDebugGpu(DebugLevel debugLevel);

    /// Compile GPU code for debugging?
    /// This only affects OpenCL late compilation (not CUDA).
    /// @return The @c DebugLevel to execute with.
    DebugLevel debugGpu() const;

    /// Add a path to the GPU source @c searchPaths().
    /// @param path The path to add.
    void addSearchPath(const char *path);

    /// Retrieve the search paths used for finding GPU source files (OpenCL sources).
    /// The search paths are comma separated.
    /// @return The search paths to be used to find GPU sources.
    const char *searchPaths() const;

    /// Is the device valid?
    /// @return True if valid.
    bool isValid() const;

    /// Query the memory for the device.
    /// @return The device memory in bytes.
    uint64_t deviceMemory() const;

    /// Copy assignment.
    /// @param other Object to copy.
    /// @return *this
    Device &operator=(const Device &other);

    /// Move assignment.
    /// @param other Object to move.
    /// @return *this
    Device &operator=(Device &&other);

    /// @interal
    DeviceDetail *detail() const { return _imp; }

  private:
    DeviceDetail *_imp;
  };
}

#endif // GPUDEVICE_H_
