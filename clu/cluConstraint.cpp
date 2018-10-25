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
  PlatformConstraint platformNameLike(const char *name, bool ignore_case)
  {
    std::string like_name(name);
    if (ignore_case)
    {
      std::transform(like_name.begin(), like_name.end(), like_name.begin(), ::tolower);
    }

    PlatformConstraint constraint = [like_name, ignore_case](const cl::Platform &platform) -> bool
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


  PlatformConstraint platformVendorLike(const char *name, bool ignore_case)
  {
    std::string like_name(name);
    if (ignore_case)
    {
      std::transform(like_name.begin(), like_name.end(), like_name.begin(), ::tolower);
    }

    PlatformConstraint constraint = [like_name, ignore_case](const cl::Platform &platform) -> bool
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


  PlatformConstraint platformVersionMin(unsigned major, unsigned minor)
  {
    PlatformConstraint constraint = [major, minor](const cl::Platform &platform) -> bool
    {
      cl_uint major_version = 0;
      cl_uint minor_version = 0;
      platformVersion(platform(), &major_version, &minor_version);
      return major_version > major || major_version == major && minor_version >= minor;
    };

    return constraint;
  }


  DeviceConstraint deviceVersionMin(unsigned major, unsigned minor)
  {
    DeviceConstraint constraint = [major, minor](const cl::Platform &, const cl::Device &device) -> bool
    {
      ::size_t size = 0;
      clGetDeviceInfo(device(), CL_DEVICE_VERSION, 0, nullptr, &size);
      cl::string version_info;
      version_info.resize(size + 1);
      clGetDeviceInfo(device(), CL_DEVICE_VERSION, size, &version_info[0], &size);

      cl_uint major_version = 0;
      cl_uint minor_version = 0;
      if (!parseVersion(version_info.c_str(), &major_version, &minor_version))
      {
        return false;
      }
      return major_version > major || major_version == major && minor_version >= minor;
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
