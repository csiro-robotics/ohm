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
  enum DeviceType : uint16_t
  {
    kDeviceNull,
    kDeviceGpu,
    kDeviceCpu,
    kDeviceOther
  };

  struct DeviceInfo
  {
    std::string name;
    std::string platform;
    Version version;
    DeviceType type = DeviceType::kDeviceNull;

    inline bool operator==(const DeviceInfo &other) const
    {
      return version == other.version && type == other.type && name.compare(other.name) == 0 &&
             platform.compare(other.platform) == 0;
    }
  };
}  // namespace gputil

#endif  // GPUDEVICEINFO_H
