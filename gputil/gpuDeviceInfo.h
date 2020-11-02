// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUDEVICEINFO_H
#define GPUDEVICEINFO_H

#include "gpuConfig.h"

#include "gpuVersion.h"

#include <string>

namespace gputil
{
/// Enumeration of accelerator types.
enum DeviceType : uint16_t
{
  kDeviceNull,  ///< Invalid
  kDeviceGpu,   ///< A GPU accelerator device.
  kDeviceCpu,   ///< A CPU accelerator device
  kDeviceOther  ///< Some other accelerator type.
};

/// Structure detailing information about a GPU or accelerator device. Available details may vary depending on the
/// GPU SDK.
struct DeviceInfo
{
  /// The device name - e.g., the name of the GPU card.
  std::string name;
  /// Details of the accelerator platform - e.g., OpenCL &lt;version&gt; or CUDA.
  std::string platform;
  /// Accelerator API version - e.g., OpenCL 1.2 vs 2.0
  Version version;
  /// The accelerator type of the default.
  DeviceType type = DeviceType::kDeviceNull;

  /// Equality operator.
  /// @param other The structure to compare against.
  /// @return True if @c this exactly matches @p other .
  inline bool operator==(const DeviceInfo &other) const
  {
    return version == other.version && type == other.type && name == other.name && platform == other.platform;
  }
};
}  // namespace gputil

#endif  // GPUDEVICEINFO_H
