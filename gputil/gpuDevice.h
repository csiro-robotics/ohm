// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUDEVICE_H
#define GPUDEVICE_H

#include "gpuConfig.h"

#include "gpuDeviceInfo.h"
#include "gpuQueue.h"

#include <vector>

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
      kDlOff,
      /// Minimal debugging. Optimisations are still active.
      kDlLow,
      /// Full debugging, optimisations disabled.
      kDlFull
    };

    /// Construct to access default device or as an invalid device.
    /// @param default_device True to construct access the default device.
    Device(bool default_device = false);

    /// Create a device from the selected @c DeviceInfo.
    /// @param device_info Info structure about the desired device.
    Device(const DeviceInfo &device_info);

    /// Construct using @c select().
    /// @param argc Number of values in @c argv.
    /// @param argv Argument string to parse.
    /// @param default_device Partial device name to match as the default selection.
    Device(int argc, const char **argv, const char *default_device = nullptr);
    /// @overload
    inline Device(int argc, char **argv, const char *default_device = nullptr)
      : Device(argc, const_cast<const char **>(argv), default_device)
    {}

    /// Copy constructor.
    /// @param other Object to copy.
    Device(const Device &other);

    /// Move constructor.
    /// @param other Object to move.
    Device(Device &&other) noexcept;

    /// Destructor.
    ~Device();

    /// Enumerate all devices attached to the current host.
    static unsigned enumerateDevices(std::vector<DeviceInfo> &devices);

    /// Display name for the device.
    const char *name() const;

    /// Device description.
    const char *description() const;

    /// General device info.
    const DeviceInfo &info() const;

    /// Get the default queue associated with this device.
    /// @return The default device queue.
    Queue defaultQueue() const;

    /// Create a new queue for this device.
    /// @param flags Creation flags for the queue. See @c Queue::QueueFlag.
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
    /// @param default_device Default device constraint to select if non specified.
    /// @return True on success.
    bool select(int argc, const char **argv, const char *default_device = nullptr);

    /// @overload
    inline bool select(int argc, char **argv, const char *default_device = nullptr)
    {
      return select(argc, const_cast<const char **>(argv), default_device);
    }

    /// Select a device from the given @c DeviceInfo.
    /// @param device_info The selected device details.
    /// @return True on success, false on failure to match @p device_info.
    bool select(const DeviceInfo &device_info);

    /// Set the value for @c debugGpu().
    /// @param debug_level The @c DebugLevel to set.
    void setDebugGpu(DebugLevel debug_level);

    /// Compile GPU code for debugging?
    /// This only affects OpenCL late compilation (not CUDA).
    /// @return The @c DebugLevel to execute with.
    DebugLevel debugGpu() const;

    /// Check if the device supports the specified feature.
    ///
    /// Feature depend on the underlying platform. For example, "cl_khr_global_int32_base_atomics".
    ///
    /// @param feature_id Platform specific feature name.
    /// @return True if the feature is valid and supported.
    bool supportsFeature(const char *feature_id) const;

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
    Device &operator=(Device &&other) noexcept;

    /// @internal
    DeviceDetail *detail() const { return imp_; }

  private:
    DeviceDetail *imp_;
  };
}  // namespace gputil

#endif  // GPUDEVICE_H
