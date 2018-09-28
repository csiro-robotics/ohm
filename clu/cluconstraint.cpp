//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#include "cluconstraint.h"

#include <algorithm>
#include <string>

namespace clu
{
  PlatformContraint platformNameLike(const char *name, bool ignoreCase)
  {
    std::string likeName(name);
    if (ignoreCase)
    {
      std::transform(likeName.begin(), likeName.end(), likeName.begin(), ::tolower);
    }

    PlatformContraint constraint = [likeName, ignoreCase](const cl::Platform &platform) -> bool
    {
      std::string platformName;
      platform.getInfo((cl_platform_info)CL_PLATFORM_NAME, &platformName);
      if (ignoreCase)
      {
        std::transform(platformName.begin(), platformName.end(), platformName.begin(), ::tolower);
      }
      return platformName.find(likeName) != std::string::npos;
    };

    return constraint;
  }


  PlatformContraint platformVendorLike(const char *name, bool ignoreCase)
  {
    std::string likeName(name);
    if (ignoreCase)
    {
      std::transform(likeName.begin(), likeName.end(), likeName.begin(), ::tolower);
    }

    PlatformContraint constraint = [likeName, ignoreCase](const cl::Platform &platform) -> bool
    {
      std::string vendor;
#ifndef NDEBUG
      std::string name;
      platform.getInfo((cl_platform_info)CL_PLATFORM_NAME, &name);
#endif // NDEBUG
      platform.getInfo((cl_platform_info)CL_PLATFORM_VENDOR, &vendor);
      if (ignoreCase)
      {
        std::transform(vendor.begin(), vendor.end(), vendor.begin(), ::tolower);
      }
      return vendor.find(likeName) != std::string::npos;
    };

    return constraint;
  }


  DeviceConstraint deviceVersionIs(int major, int minor)
  {
    DeviceConstraint constraint = [major, minor](const cl::Platform &, const cl::Device &device) -> bool
    {
      ::size_t size = 0;
      clGetDeviceInfo(device(), CL_DEVICE_VERSION, 0, 0, &size);
      cl::STRING_CLASS versionInfo;
      versionInfo.resize(size + 1);
      clGetDeviceInfo(device(), CL_DEVICE_VERSION, size, &versionInfo[0],
                      &size);
      int highVersion = 0;
      int lowVersion = 0;
      int index = 7;
      while (versionInfo[index] != '.')
      {
        highVersion *= 10;
        highVersion += versionInfo[index] - '0';
        ++index;
      }
      ++index;
      while (versionInfo[index] != ' ')
      {
        lowVersion *= 10;
        lowVersion += versionInfo[index] - '0';
        ++index;
      }
      return highVersion >= major && lowVersion >= minor;
    };

    return constraint;
  }


  DeviceConstraint deviceNameLike(const char *name, bool ignoreCase)
  {
    std::string likeName(name);
    if (ignoreCase)
    {
      std::transform(likeName.begin(), likeName.end(), likeName.begin(), ::tolower);
    }

    DeviceConstraint constraint = [likeName, ignoreCase](const cl::Platform &, const cl::Device &device) -> bool
    {
      std::string deviceName;
      device.getInfo((cl_device_info)CL_DEVICE_NAME, &deviceName);
      if (ignoreCase)
      {
        std::transform(deviceName.begin(), deviceName.end(), deviceName.begin(), ::tolower);
      }
      return deviceName.find(likeName) != std::string::npos;
    };

    return constraint;
  }


  DeviceConstraint deviceVendorLike(const char *name, bool ignoreCase)
  {
    std::string likeName(name);
    if (ignoreCase)
    {
      std::transform(likeName.begin(), likeName.end(), likeName.begin(), ::tolower);
    }

    DeviceConstraint constraint = [likeName, ignoreCase](const cl::Platform &platform, const cl::Device &device) -> bool
    {
      std::string platformVendor;
      std::string deviceVendor;
      platform.getInfo((cl_platform_info)CL_PLATFORM_VENDOR, &platformVendor);
      device.getInfo((cl_device_info)CL_DEVICE_VENDOR, &deviceVendor);
      if (ignoreCase)
      {
        std::transform(platformVendor.begin(), platformVendor.end(), platformVendor.begin(), ::tolower);
        std::transform(deviceVendor.begin(), deviceVendor.end(), deviceVendor.begin(), ::tolower);
      }
      return deviceVendor.find(likeName) != std::string::npos || platformVendor.find(likeName) != std::string::npos;
    };

    return constraint;
  }
}
