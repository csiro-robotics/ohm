//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#include "clu.h"

#include <mutex>
#include <sstream>

#include "cluconstraint.h"

namespace
{
  cl::Context s_primaryContext;
  cl::Device s_primaryDevice;
  std::mutex s_lock;


  struct InfoItem
  {
    int id;
    const char *label;
  };


  void split(const std::string &str, char delim, std::vector<std::string> &tokens)
  {
    size_t prev = 0;
    size_t pos = 0;
    do
    {
      pos = str.find(delim, prev);

      if (pos != std::string::npos)
      {
        tokens.push_back(str.substr(prev, pos - prev));
        prev = pos + 1;
      }
      else
      {
        tokens.push_back(str.substr(prev, str.length()));
      }
    } while (pos != std::string::npos);
  }
}

namespace clu
{
  enum ArgParse
  {
    AP_Ok,
    AP_MissingValue,
    AP_ParseFailure
  };

  cl::Platform createPlatform(cl_device_type type, const PlatformContraint *constraints, unsigned constraintCount)
  {
    std::vector<cl::Platform> platforms;
    std::vector<cl::Device> devices;
    cl::Platform::get(&platforms);

    for (cl::Platform &platform : platforms)
    {
      // Check constraints.
      bool constraintsOk = true;
      for (unsigned i = 0; i < constraintCount; ++i)
      {
        if (!(*constraints)(platform))
        {
          constraintsOk = false;
          break;
        }
      }

      if (!constraintsOk)
      {
        // A constraint has failed.
        continue;
      }

      if (type != CL_DEVICE_TYPE_ALL)
      {
        // Check device types.
        devices.clear();
        platform.getDevices(type, &devices);
        if (!devices.empty())
        {
          // Found a suitable platform.
          return platform;
        }
      }
      else
      {
        return platform;
      }
    }

    return cl::Platform();
  }


  cl::Platform createPlatform(cl_device_type type, const std::vector<PlatformContraint> &constraints)
  {
    return createPlatform(type, constraints.data(), unsigned(constraints.size()));
  }


  bool filterPlatforms(std::vector<cl::Platform> &platforms, cl_device_type type, const PlatformContraint *constraints, unsigned constraintCount)
  {
    for (auto iter = platforms.begin(); iter != platforms.end(); )
    {
      bool constraintsOk = true;
      for (unsigned i = 0; i < constraintCount; ++i)
      {
        if (!(*constraints)(*iter))
        {
          constraintsOk = false;
          break;
        }
      }

      if (constraintsOk)
      {
        ++iter;
      }
      else
      {
        iter = platforms.erase(iter);
      }
    }

    return !platforms.empty();
  }


  bool filterDevices(const cl::Platform &platform, std::vector<cl::Device> &devices, const DeviceConstraint *constraints, unsigned constraintCount)
  {
    for (auto iter = devices.begin(); iter != devices.end(); )
    {
      bool constraintsOk = true;
      for (unsigned i = 0; i < constraintCount; ++i)
      {
        if (!(*constraints)(platform, *iter))
        {
          constraintsOk = false;
          break;
        }
      }

      if (constraintsOk)
      {
        ++iter;
      }
      else
      {
        iter = devices.erase(iter);
      }
    }

    return !devices.empty();
  }


  unsigned listDevices(std::vector<cl::Device> &devices, const cl::Context &context)
  {
    cl_int clerr = 0;
    cl_int deviceCount = 0;
    devices.clear();
    clerr = clGetContextInfo(context(), CL_CONTEXT_NUM_DEVICES, sizeof(deviceCount), &deviceCount, nullptr);

    if (deviceCount == 0 || clerr != CL_SUCCESS)
    {
      return 0;
    }

    cl_device_id *deviceIds = (cl_device_id *)alloca(sizeof(cl_device_id) * deviceCount);
    clerr = clGetContextInfo(context(), CL_CONTEXT_DEVICES, sizeof(cl_device_id) * deviceCount, deviceIds, nullptr);
    if (clerr != CL_SUCCESS)
    {
      return 0;
    }

    devices.resize(deviceCount);
    for (cl_int i = 0; i < deviceCount; ++i)
    {
      devices[i] = cl::Device(deviceIds[i]);
    }

    return deviceCount;
  }


  cl_device_id getFirstDevice(const cl::Context &context, cl_int *err)
  {
    cl_int clerr = 0;
    cl_int deviceCount = 0;
    clerr = clGetContextInfo(context(), CL_CONTEXT_NUM_DEVICES, sizeof(deviceCount), &deviceCount, nullptr);

    if (deviceCount == 0 || clerr != CL_SUCCESS)
    {
      if (err)
      {
        *err = clerr;
      }
      return nullptr;
    }

    cl_device_id *deviceIds = (cl_device_id *)alloca(sizeof(cl_device_id) * deviceCount);
    clerr = clGetContextInfo(context(), CL_CONTEXT_DEVICES, sizeof(cl_device_id) * deviceCount, deviceIds, nullptr);
    if (err)
    {
      *err = clerr;
    }

    if (clerr != CL_SUCCESS)
    {
      return nullptr;
    }

    return deviceIds[0];
  }

#if 0
  cl::Context createContext(const cl::Platform &platform, cl_device_type type, const DeviceConstraint *constraints, unsigned constraintCount)
  {
    std::vector<cl::Device> devices;
    platform.getDevices(type, &devices);

    if (!filterDevices(platform, devices, constraints, constraintCount))
    {
      return cl::Context();
    }

    // Use the preferred platform and create a context
    cl_context_properties cprops[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)(platform)(), 0 };
    cl::Context context = cl::Context(devices, cprops);
    return context;
  }


  cl::Context createContext(const cl::Platform &platform, cl_device_type type, const std::vector<DeviceConstraint> &constraints)
  {
    return createContext(platform, type, constraints.data(), unsigned(constraints.size()));
  }
#endif // #

  cl::Context createContext(cl::Device *deviceOut, cl_device_type type,
                            const PlatformContraint *platformConstraint, unsigned platformConstraintCount,
                            const DeviceConstraint *deviceConstraints, unsigned deviceConstraintCount)
  {
    std::vector<cl::Platform> platforms;
    std::vector<cl::Device> devices;
    cl::Platform::get(&platforms);
    filterPlatforms(platforms, type, platformConstraint, platformConstraintCount);

    for (cl::Platform &platform : platforms)
    {
      devices.clear();
      platform.getDevices(type, &devices);
      filterDevices(platform, devices, deviceConstraints, deviceConstraintCount);
      if (!devices.empty())
      {
        // Select a single device.
        cl_context_properties cprops[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)(platform)(), 0 };
        cl::Context context = cl::Context(devices[0], cprops);

        if (context())
        {
          if (deviceOut)
          {
            *deviceOut = devices[0];
          }
          return context;
        }
      }
    }

    return cl::Context();
  }


  bool setPrimaryContext(const cl::Context &context, const cl::Device &device)
  {
    if (!context())
    {
      return false;
    }

    std::unique_lock<std::mutex> guard(s_lock);
    s_primaryContext = context;
    s_primaryDevice = device;
    return true;
  }


  void clearPrimaryContext()
  {
    std::unique_lock<std::mutex> guard(s_lock);
    s_primaryContext = cl::Context();
    s_primaryDevice = cl::Device();
  }


  bool initPrimaryContext(cl_device_type type,
                          const PlatformContraint *platformConstraints, unsigned platformConstraintCount,
                          const DeviceConstraint *deviceConstraints, unsigned deviceConstraintCount)
  {
    cl::Device defaultDevice;
    cl::Context context = createContext(&defaultDevice, type,
                                        platformConstraints, platformConstraintCount,
                                        deviceConstraints, deviceConstraintCount);
    return setPrimaryContext(context, defaultDevice);
  }


  bool initPrimaryContext(cl_device_type type,
                          const std::vector<PlatformContraint> &platformConstraint,
                          const std::vector<DeviceConstraint> &deviceConstraints)
  {
    return initPrimaryContext(type, platformConstraint.data(), unsigned(platformConstraint.size()),
                              deviceConstraints.data(), unsigned(deviceConstraints.size()));
  }


  bool getPrimaryContext(cl::Context &context, cl::Device &device)
  {
    std::unique_lock<std::mutex> guard(s_lock);
    context = s_primaryContext;
    device = s_primaryDevice;
    return context() != 0;
  }


  ArgParse argValue(std::string &val, const std::string &arg,
                    std::list<std::string>::const_iterator &iter,
                    const std::list<std::string>::const_iterator &end)
  {
    std::string::size_type eqpos = arg.find("=");
    std::list<std::string>::const_iterator next = iter;

    ++next;
    // Check for form
    if (eqpos == std::string::npos)
    {
      // Not of the form "--arg=value". Read next argument.
      if (next != end)
      {
        val = *next;
        if (val.find("--") != 0)
        {
          // Good value. Consume the next argument.
          iter = next;
          return AP_Ok;
        }
        else
        {
          // Argument missing value.
          return AP_MissingValue;
        }
      }
      // Not enough additional arguments.
      return AP_MissingValue;
    }

    // Of the form "--arg=value"
    val = arg.substr(eqpos + 1);
    return (!val.empty()) ? AP_Ok : AP_MissingValue;
  }


  void constraintsFromCommandLine(int argc, const char **argv,
                                  cl_device_type &type,
                                  std::vector<PlatformContraint> &platformConstraints,
                                  std::vector<DeviceConstraint> &deviceConstraints,
                                  const char *argPrefix)
  {
    std::list<std::string> args;
    for (int i = 0; i < argc; ++i)
    {
      args.push_back(argv[i]);
    }

    return constraintsFromArgs(args, type, platformConstraints, deviceConstraints, argPrefix);
  }


  void constraintsFromArgs(const std::list<std::string> &args,
                           cl_device_type &type,
                           std::vector<PlatformContraint> &platformConstraints,
                           std::vector<DeviceConstraint> &deviceConstraints,
                           const char *argPrefix)
  {
    std::string arg, val;
    std::string prefix = (argPrefix) ? std::string("--") + std::string(argPrefix) : "--";
    std::vector<std::string> tokens;
    ArgParse parseResult = AP_Ok;

    type = CL_DEVICE_TYPE_GPU | CL_DEVICE_TYPE_ACCELERATOR;

    for (auto iter = args.begin(); iter != args.end(); ++iter)
    {
      arg = *iter;
      if (arg.find(prefix) == 0)
      {
        arg.erase(0, prefix.length());
        if (arg.find("accel") == 0)
        {
          parseResult = argValue(val, arg, iter, args.end());
          if (parseResult == AP_Ok)
          {
            tokens.clear();
            split(val, ',', tokens);
            type = 0;
            for (const std::string &typeStr : tokens)
            {
              if (typeStr.compare("any") == 0)
              {
                type |= CL_DEVICE_TYPE_ALL;
              }
              else if (typeStr.compare("accel") == 0)
              {
                type |= CL_DEVICE_TYPE_ACCELERATOR;
              }
              else if (typeStr.compare("cpu") == 0)
              {
                type |= CL_DEVICE_TYPE_CPU;
              }
              else if (typeStr.compare("gpu") == 0)
              {
                type |= CL_DEVICE_TYPE_GPU;
              }
              else
              {
                parseResult = AP_ParseFailure;
              }
            }
          }
        }
        else if (arg.find("clver") == 0)
        {
          parseResult = argValue(val, arg, iter, args.end());
          if (parseResult == AP_Ok)
          {
            int major = 0, minor = 0;
            // Expecting: major[.minor]
            tokens.clear();
            split(val, '.', tokens);
            if (!tokens.empty() && tokens.size() <= 2)
            {
              std::istringstream str(tokens[0]);
              str >> major;
              if (str.fail())
              {
                parseResult = AP_ParseFailure;
              }
              if (tokens.size() > 1)
              {
                str = std::istringstream(tokens[1]);
                str >> minor;
                if (str.fail())
                {
                  parseResult = AP_ParseFailure;
                }
              }

              if (parseResult == AP_Ok)
              {
                // Add a device version constraint.
                deviceConstraints.push_back(deviceVersionIs(major, minor));
              }
            }
            else
            {
              parseResult = AP_ParseFailure;
            }
          }
        }
        else if (arg.find("device") == 0)
        {
          parseResult = argValue(val, arg, iter, args.end());
          if (parseResult == AP_Ok)
          {
            // Add a device constraint.
            deviceConstraints.push_back(deviceNameLike(val.c_str(), true));
          }
        }
        else if (arg.find("platform") == 0)
        {
          parseResult = argValue(val, arg, iter, args.end());
          if (parseResult == AP_Ok)
          {
            // Add a device constraint.
            platformConstraints.push_back(platformNameLike(val.c_str(), true));
          }
        }
        else if (arg.find("vendor") == 0)
        {
          parseResult = argValue(val, arg, iter, args.end());
          if (parseResult == AP_Ok)
          {
            // Add a device constraint.
            // platformConstraints.push_back(platformVendorLike(val.c_str(), true));
            deviceConstraints.push_back(deviceVendorLike(val.c_str(), true));
          }
        }

        if (parseResult != AP_Ok)
        {
          switch (parseResult)
          {
          case AP_ParseFailure:
            fprintf(stderr, "Failed parsing argument '%s' value '%s'\n", arg.c_str(), val.c_str());
            break;
          case AP_MissingValue:
            fprintf(stderr, "Argument '%s' missing value\n", arg.c_str());
            break;
          default:
            fprintf(stderr, "Error processing argument '%s'\n", arg.c_str());
            break;
          }
        }
      }
    }
  }


  const char *errorCodeString(cl_int error)
  {
#define CL_ERR_CASE(CODE) case CODE: return #CODE;
#define CL_ERR_CASE2(CODE, msg) case CODE: return msg;

    switch (error)
    {
      CL_ERR_CASE(CL_SUCCESS)
      CL_ERR_CASE(CL_DEVICE_NOT_FOUND)
      CL_ERR_CASE(CL_DEVICE_NOT_AVAILABLE)
      CL_ERR_CASE(CL_COMPILER_NOT_AVAILABLE)
      CL_ERR_CASE(CL_MEM_OBJECT_ALLOCATION_FAILURE)
      CL_ERR_CASE(CL_OUT_OF_RESOURCES)
      CL_ERR_CASE(CL_OUT_OF_HOST_MEMORY)
      CL_ERR_CASE(CL_PROFILING_INFO_NOT_AVAILABLE)
      CL_ERR_CASE(CL_MEM_COPY_OVERLAP)
      CL_ERR_CASE(CL_IMAGE_FORMAT_MISMATCH)
      CL_ERR_CASE(CL_IMAGE_FORMAT_NOT_SUPPORTED)
      CL_ERR_CASE(CL_BUILD_PROGRAM_FAILURE)
      CL_ERR_CASE(CL_MAP_FAILURE)
      CL_ERR_CASE(CL_MISALIGNED_SUB_BUFFER_OFFSET)
      CL_ERR_CASE(CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST)
      CL_ERR_CASE(CL_COMPILE_PROGRAM_FAILURE)
      CL_ERR_CASE(CL_LINKER_NOT_AVAILABLE)
      CL_ERR_CASE(CL_LINK_PROGRAM_FAILURE)
      CL_ERR_CASE(CL_DEVICE_PARTITION_FAILED)
      CL_ERR_CASE(CL_KERNEL_ARG_INFO_NOT_AVAILABLE)
      CL_ERR_CASE(CL_INVALID_VALUE)
      CL_ERR_CASE(CL_INVALID_DEVICE_TYPE)
      CL_ERR_CASE(CL_INVALID_PLATFORM)
      CL_ERR_CASE(CL_INVALID_DEVICE)
      CL_ERR_CASE(CL_INVALID_CONTEXT)
      CL_ERR_CASE(CL_INVALID_QUEUE_PROPERTIES)
      CL_ERR_CASE(CL_INVALID_COMMAND_QUEUE)
      CL_ERR_CASE(CL_INVALID_HOST_PTR)
      CL_ERR_CASE(CL_INVALID_MEM_OBJECT)
      CL_ERR_CASE(CL_INVALID_IMAGE_FORMAT_DESCRIPTOR)
      CL_ERR_CASE(CL_INVALID_IMAGE_SIZE)
      CL_ERR_CASE(CL_INVALID_SAMPLER)
      CL_ERR_CASE(CL_INVALID_BINARY)
      CL_ERR_CASE(CL_INVALID_BUILD_OPTIONS)
      CL_ERR_CASE(CL_INVALID_PROGRAM)
      CL_ERR_CASE(CL_INVALID_PROGRAM_EXECUTABLE)
      CL_ERR_CASE(CL_INVALID_KERNEL_NAME)
      CL_ERR_CASE(CL_INVALID_KERNEL_DEFINITION)
      CL_ERR_CASE(CL_INVALID_KERNEL)
      CL_ERR_CASE(CL_INVALID_ARG_INDEX)
      CL_ERR_CASE(CL_INVALID_ARG_VALUE)
      CL_ERR_CASE(CL_INVALID_ARG_SIZE)
      CL_ERR_CASE(CL_INVALID_KERNEL_ARGS)
      CL_ERR_CASE(CL_INVALID_WORK_DIMENSION)
      CL_ERR_CASE(CL_INVALID_WORK_GROUP_SIZE)
      CL_ERR_CASE(CL_INVALID_WORK_ITEM_SIZE)
      CL_ERR_CASE(CL_INVALID_GLOBAL_OFFSET)
      CL_ERR_CASE(CL_INVALID_EVENT_WAIT_LIST)
      CL_ERR_CASE(CL_INVALID_EVENT)
      CL_ERR_CASE(CL_INVALID_OPERATION)
      CL_ERR_CASE(CL_INVALID_GL_OBJECT)
      CL_ERR_CASE(CL_INVALID_BUFFER_SIZE)
      CL_ERR_CASE(CL_INVALID_MIP_LEVEL)
      CL_ERR_CASE(CL_INVALID_GLOBAL_WORK_SIZE)
      CL_ERR_CASE(CL_INVALID_PROPERTY)
      CL_ERR_CASE(CL_INVALID_IMAGE_DESCRIPTOR)
      CL_ERR_CASE(CL_INVALID_COMPILER_OPTIONS)
      CL_ERR_CASE(CL_INVALID_LINKER_OPTIONS)
      CL_ERR_CASE(CL_INVALID_DEVICE_PARTITION_COUNT)

#if CL_VERSION_2_0
      CL_ERR_CASE(CL_INVALID_PIPE_SIZE)
      CL_ERR_CASE(CL_INVALID_DEVICE_QUEUE)
#endif // CL_VERSION_2_0

      // extension errors
#ifdef CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR
      CL_ERR_CASE(CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR)
#endif // CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR

#ifdef CL_PLATFORM_NOT_FOUND_KHR
      CL_ERR_CASE(CL_PLATFORM_NOT_FOUND_KHR)
#endif // CL_PLATFORM_NOT_FOUND_KHR

#ifdef CL_INVALID_D3D10_DEVICE_KHR
      CL_ERR_CASE(CL_INVALID_D3D10_DEVICE_KHR)
#endif // CL_INVALID_D3D10_DEVICE_KHR

#ifdef CL_INVALID_D3D10_RESOURCE_KHR
      CL_ERR_CASE(CL_INVALID_D3D10_RESOURCE_KHR)
#endif // CL_INVALID_D3D10_RESOURCE_KHR

#ifdef CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR
      CL_ERR_CASE(CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR)
#endif // CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR

#ifdef CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR
      CL_ERR_CASE(CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR)
#endif // CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR

    default:
      return "Unknown OpenCL error";
    }

#undef CL_ERR_CASE
#undef CL_ERR_CASE2
  }


  void printPlatformInfo(std::ostream &out, const cl::Platform &platform, const char *prefix, const char *endl)
  {
    static InfoItem items[] =
    {
      { CL_PLATFORM_NAME, "Name" },
      { CL_PLATFORM_VERSION, "Version" },
      { CL_PLATFORM_VENDOR, "Vendor" },
      //{ CL_PLATFORM_PROFILE, "Profile" },
      //{ CL_PLATFORM_EXTENSIONS, "Extensions" },
    };
    std::string infoStr, infoStr2;
    if (platform())
    {
      for (int i = 0; i < sizeof(items) / sizeof(items[0]); ++i)
      {
        platform.getInfo((cl_platform_info)items[i].id, &infoStr);
        if (i > 0)
        {
          out << endl;
        }
        // Use c_str() otherwise we get extra '\0' characters.
        out << prefix << items[i].label << ": " << infoStr.c_str();
      }
    }
  }


  void printDeviceInfo(std::ostream &out, const cl::Device &device, const char *prefix, const char *endl)
  {
    static InfoItem items[] =
    {
      { CL_DEVICE_NAME, "Name" },
      { CL_DEVICE_VERSION, "Version" },
      //{ CL_PLATFORM_PROFILE, "Profile" },
      //{ CL_PLATFORM_EXTENSIONS, "Extensions" },
    };
    std::string infoStr;
    if (device())
    {
      for (int i = 0; i < sizeof(items) / sizeof(items[0]); ++i)
      {
        device.getInfo((cl_device_info)items[i].id, &infoStr);
        if (i > 0)
        {
          out << endl;
        }
        // Use c_str() otherwise we get extra '\0' characters.
        out << prefix << items[i].label << ": " << infoStr.c_str();
      }
    }
  }
}
