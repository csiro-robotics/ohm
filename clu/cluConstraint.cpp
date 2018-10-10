//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#include "cluConstraint.h"

#include <algorithm>
#include <string>

namespace clu
{
  PlatformContraint platformNameLike(const char *name, bool ignore_case)
  {
    std::string like_name(name);
    if (ignore_case)
    {
      std::transform(like_name.begin(), like_name.end(), like_name.begin(), ::tolower);
    }

    PlatformContraint constraint = [like_name, ignore_case](const cl::Platform &platform) -> bool
    {
      std::string platform_name;
      platform.getInfo(CL_PLATFORM_NAME, &platform_name);
      if (ignore_case)
      {
        std::transform(platform_name.begin(), platform_name.end(), platform_name.begin(), ::tolower);
      }
      return platform_name.find(like_name) != std::string::npos;
    };

    return constraint;
  }


  PlatformContraint platformVendorLike(const char *name, bool ignore_case)
  {
    std::string like_name(name);
    if (ignore_case)
    {
      std::transform(like_name.begin(), like_name.end(), like_name.begin(), ::tolower);
    }

    PlatformContraint constraint = [like_name, ignore_case](const cl::Platform &platform) -> bool
    {
      std::string vendor;
#ifndef NDEBUG
      std::string name;
      platform.getInfo(CL_PLATFORM_NAME, &name);
#endif // NDEBUG
      platform.getInfo(CL_PLATFORM_VENDOR, &vendor);
      if (ignore_case)
      {
        std::transform(vendor.begin(), vendor.end(), vendor.begin(), ::tolower);
      }
      return vendor.find(like_name) != std::string::npos;
    };

    return constraint;
  }


  DeviceConstraint deviceVersionIs(int major, int minor)
  {
    DeviceConstraint constraint = [major, minor](const cl::Platform &, const cl::Device &device) -> bool
    {
      ::size_t size = 0;
      clGetDeviceInfo(device(), CL_DEVICE_VERSION, 0, nullptr, &size);
      cl::string version_info;
      version_info.resize(size + 1);
      clGetDeviceInfo(device(), CL_DEVICE_VERSION, size, &version_info[0], &size);
      int high_version = 0;
      int low_version = 0;
      int index = 7;
      while (version_info[index] != '.')
      {
        high_version *= 10;
        high_version += version_info[index] - '0';
        ++index;
      }
      ++index;
      while (version_info[index] != ' ')
      {
        low_version *= 10;
        low_version += version_info[index] - '0';
        ++index;
      }
      return high_version >= major && low_version >= minor;
    };

    return constraint;
  }


  DeviceConstraint deviceNameLike(const char *name, bool ignore_case)
  {
    std::string like_name(name);
    if (ignore_case)
    {
      std::transform(like_name.begin(), like_name.end(), like_name.begin(), ::tolower);
    }

    DeviceConstraint constraint = [like_name, ignore_case](const cl::Platform &, const cl::Device &device) -> bool
    {
      std::string device_name;
      device.getInfo(CL_DEVICE_NAME, &device_name);
      if (ignore_case)
      {
        std::transform(device_name.begin(), device_name.end(), device_name.begin(), ::tolower);
      }
      return device_name.find(like_name) != std::string::npos;
    };

    return constraint;
  }


  DeviceConstraint deviceVendorLike(const char *name, bool ignore_case)
  {
    std::string like_name(name);
    if (ignore_case)
    {
      std::transform(like_name.begin(), like_name.end(), like_name.begin(), ::tolower);
    }

    DeviceConstraint constraint = [like_name, ignore_case](const cl::Platform &platform, const cl::Device &device) -> bool
    {
      std::string platform_vendor;
      std::string device_vendor;
      platform.getInfo(CL_PLATFORM_VENDOR, &platform_vendor);
      device.getInfo(CL_DEVICE_VENDOR, &device_vendor);
      if (ignore_case)
      {
        std::transform(platform_vendor.begin(), platform_vendor.end(), platform_vendor.begin(), ::tolower);
        std::transform(device_vendor.begin(), device_vendor.end(), device_vendor.begin(), ::tolower);
      }
      return device_vendor.find(like_name) != std::string::npos || platform_vendor.find(like_name) != std::string::npos;
    };

    return constraint;
  }
}
