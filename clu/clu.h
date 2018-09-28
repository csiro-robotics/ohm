//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#ifndef CLU_H_
#define CLU_H_

#include "cl.hpp"

#include <functional>
#include <iostream>
#include <list>
#include <vector>

namespace clu
{
  /// Constraint function prototype used in platform selection.
  typedef std::function<bool(const cl::Platform &)> PlatformContraint;
  /// Constraint function prototype used in device selection.
  typedef std::function<bool(const cl::Platform &, const cl::Device &)> DeviceConstraint;

  /// Create the OpenCL platform optionally applying the given constraints.
  ///
  /// The selected device must pass all the @p constraints given, each returning true, in order
  /// to be selected. If no platform passes all tests, then an invalid platform is returned.
  /// The first valid platform is always returned including when there are no constraints.
  ///
  /// See @c clucontraint.h for some constraint examples.
  ///
  /// @param type The required OpenCL device type.
  /// @param constraints The array of constraints to pass or null for no constraints.
  /// @param constraintCount The number of @p constraints.
  /// @return The first valid platform found, passing all constraints.
  cl::Platform createPlatform(cl_device_type type, const PlatformContraint *constraints = nullptr, unsigned constraintCount = 0);
  /// Create the OpenCL platform optionally applying the given constraints.
  ///
  /// The selected device must pass all the @p constraints given, each returning true, in order
  /// to be selected. If no platform passes all tests, then an invalid platform is returned.
  ///
  /// @param type The required OpenCL device type.
  /// @param constraints The array of constraints to pass or empty for no constraints.
  /// @return The first valid platform found, passing all constraints.
  cl::Platform createPlatform(cl_device_type type, const std::vector<PlatformContraint> &constraints);

  /// Filter the given list of @p platforms, removing all which do not match @p type or pass
  /// @p constraints.
  ///
  /// See @c clucontraint.h for some constraint examples.
  ///
  /// @param[in,out] platforms The Platforms to filter.
  /// @param type The required device type(s).
  /// @param constraints The constraints to filter by.
  /// @param constraintCount The number of @p constraints.
  /// @return True if there is at least one valid platform left in @p platforms.
  bool filterPlatforms(std::vector<cl::Platform> &platforms, cl_device_type type,
                       const PlatformContraint *constraints, unsigned constraintCount);

  /// Filter the given list of @p devices, removing all which do not match @p type or pass
  /// @p constraints.
  ///
  /// See @c clucontraint.h for some constraint examples.
  ///
  /// @param platform The platform to which the @p devices belong.
  /// @param[in,out] devices The Platforms to filter.
  /// @param type The required device type(s).
  /// @param constraints The device constraints to filter by.
  /// @param constraintCount The number of @p constraints.
  /// @return True if there is at least one valid device left in @p devices.
  bool filterDevices(const cl::Platform &platform, std::vector<cl::Device> &devices,
                     const DeviceConstraint *constraints, unsigned constraintCount);

  /// List all the available devices in @p context.
  /// @param devices Populated with the devices from @p context.
  /// @param context The OpenCL context of interest.
  /// @return The number of items in @p devices.
  unsigned listDevices(std::vector<cl::Device> &devices, const cl::Context &context);

  /// Retrieve the first available device from @p context.
  /// @param context The OpenCL context of interest.
  /// @param err On failure, set to any error value occurring, or @c CL_SUCCESS on success.
  ///     Ignored when null.
  /// @return The first device of @p context, or null on failure.
  cl_device_id getFirstDevice(const cl::Context &context, cl_int *err = nullptr);

#if 0
  /// Create the OpenCL context optionally applying the given device constraints.
  ///
  /// The selected context must have device(s) passing all the @p constraints given, each
  /// returning true, in order to be selected. If no context has devices passes all tests,
  /// then an invalid platform is returned. The first valid context is always returned
  /// including when there are no constraints.
  ///
  /// See @c clucontraint.h for some constraint examples.
  ///
  /// @param platform The OpenCL platform to create the context for.
  /// @param type The required OpenCL device type.
  /// @param constraints The array of device constraints to pass or null for no constraints.
  /// @param contraintCount The number of @p constraints.
  /// @return The first valid context found, passing all constraints.
  cl::Context createContext(const cl::Platform &platform, cl_device_type type,
                            const DeviceConstraint *constraints = nullptr,
                            unsigned constraintCount = 0);

  /// Create the OpenCL context optionally applying the given device constraints.
  ///
  /// The selected context must have device(s) passing all the @p constraints given, each
  /// returning true, in order to be selected. If no context has devices passes all tests,
  /// then an invalid platform is returned. The first valid context is always returned
  /// including when there are no constraints.
  ///
  /// See @c clucontraint.h for some constraint examples.
  ///
  /// @param platform The OpenCL platform to create the context for.
  /// @param type The required OpenCL device type.
  /// @param constraints The array of device constraints to pass or null for no constraints.
  /// @return The first valid context found, passing all constraints.
  cl::Context createContext(const cl::Platform &platform, cl_device_type type,
                            const std::vector<DeviceConstraint> &constraints);
#endif // #

  /// Create the OpenCL platform and context optionally applying the given platform and
  /// device constraints.
  ///
  /// The selected platform must pass all @p platformConstraints while the selected
  /// context must pass all @p deviceContraints.
  ///
  /// See @c clucontraint.h for some constraint examples.
  ///
  /// @param deviceOut Set to the selected OpenCL device.
  /// @param type The required OpenCL device type.
  /// @param platformConstraint The array of platform constraints to pass or null for no constraints.
  /// @param platformConstraintCount The number of @p platformConstraint.
  /// @param deviceConstraints The array of device constraints to pass or null for no constraints.
  /// @param deviceConstraintCount The number of @p deviceConstraints.
  /// @return The first valid context found, passing all constraints.
  cl::Context createContext(cl::Device *deviceOut, cl_device_type type,
                            const PlatformContraint *platformConstraint, unsigned platformConstraintCount,
                            const DeviceConstraint *deviceConstraints, unsigned deviceConstraintCount);

  /// Initialise the primary OpenCL context. Retrieved via @c getPrimaryContext().
  /// @param device The device to set.
  /// @param context The context to set.
  bool setPrimaryContext(const cl::Context &context, const cl::Device &device);
  /// Releases and clears the primary context available from @c getPrimaryContext().
  void clearPrimaryContext();
  /// Initialise the primary context (@c getPrimaryContext()) according to the given constraints.
  /// @param type The required OpenCL device type.
  /// @param platformConstraint The array of platform constraints to pass or null for no constraints.
  /// @param platformConstraintCount The number of @p platformConstraint.
  /// @param deviceConstraints The array of device constraints to pass or null for no constraints.
  /// @param deviceConstraintCount The number of @p deviceConstraints.
  /// @return True if all constraints are met and the context initialised.
  bool initPrimaryContext(cl_device_type type,
                          const PlatformContraint *platformConstraint, unsigned platformConstraintCount,
                          const DeviceConstraint *deviceConstraints, unsigned deviceConstraintCount);
  /// Initialise the primary context (@c getPrimaryContext()) according to the given constraints.
  /// @param type The required OpenCL device type.
  /// @param platformConstraint The array of platform constraints to pass or null for no constraints.
  /// @param deviceConstraints The array of device constraints to pass or null for no constraints.
  /// @return True if all constraints are met and the context initialised.
  bool initPrimaryContext(cl_device_type type,
                          const std::vector<PlatformContraint> &platformConstraint,
                          const std::vector<DeviceConstraint> &deviceConstraints);
  /// Get the stored primary context.
  /// @param[out] ocl Set to the primary context.
  /// @return True on success, false if not context has been initialised.
  bool getPrimaryContext(cl::Context &context, cl::Device &device);

  /// Read and initialise constraints from command line arguments.
  ///
  /// All arguments described below must be provided in the form:
  /// @code{.unparsed}
  /// --<prefix><name>=<value>
  ///   or
  /// --<prefix><name> <value>
  /// @endcode
  /// Where &lt;name&gt; matches an argument in the table below. The &lt;prefix&gt; must match
  /// @p argPrefix. It is nothing by default.
  ///
  /// Supports the following arguments:
  /// Argument | Values               | Description
  /// -------- | -------------------- | ---------------------------------------------------------
  /// accel    | any, accel, cpu, gpu | The accelerator type. The default is all.
  /// clver    | 1.0, 1.2, 2.0, ...   | The minimum required OpenCL version; @c deviceVersionIs()
  /// device   | &lt;string&gt;       | Uses @c deviceNameLike() to match the device name.
  /// platform | &lt;string&gt;       | Uses @c platformNameLike() to match the platform name.
  /// vendor   | &lt;string&gt;       | Uses @c platformVendorLike() to match the platform vendor.
  ///
  /// @param argc Nubmer of arguments in @p argv.
  /// @param argv Command line arguments.
  /// @param[out] type Set to the requested @c accel type or @c CL_DEVICE_TYPE_ALL.
  /// @param platformContraints Populated with requested platform constraints.
  /// @param deviceConstraints Populated with requested device constraints.
  /// @param argPrefix Optional prefix to required an all arguments described above.
  void constraintsFromCommandLine(int argc, const char **argv,
                                  cl_device_type &type,
                                  std::vector<PlatformContraint> &platformConstraints,
                                  std::vector<DeviceConstraint> &deviceConstraints,
                                  const char *argPrefix = nullptr);

  /// @overload
  inline void constraintsFromCommandLine(int argc, char **argv,
                                  cl_device_type &type,
                                  std::vector<PlatformContraint> &platformConstraints,
                                  std::vector<DeviceConstraint> &deviceConstraints,
                                  const char *argPrefix = nullptr)
  {
    return constraintsFromCommandLine(argc, (const char **)argv,
        type, platformConstraints, deviceConstraints, argPrefix);
  }

  /// @overload
  void constraintsFromArgs(const std::list<std::string> &args,
                           cl_device_type &type,
                           std::vector<PlatformContraint> &platformConstraints,
                           std::vector<DeviceConstraint> &deviceConstraints,
                           const char *argPrefix = nullptr);

  /// Convert a known OpenCL error code to an English string.
  /// @param error The error code.
  /// @return An English representation of @p error.
  const char *errorCodeString(cl_int error);

  /// Dump platform information about @p platform to @p out.
  ///
  /// Lists:
  /// - @c CL_PLATFORM_NAME
  /// - @c CL_PLATFORM_VERSION
  /// - @c CL_PLATFORM_VENDOR
  /// @param out Stream to write to.
  /// @param platform The platform to dump information about.
  /// @param prefix Optional prefix to add to each line to @p out.
  /// @param endl End of line character used.
  void printPlatformInfo(std::ostream &out, const cl::Platform &platform, const char *prefix = "", const char *endl = "\n");

  /// Dump device information about @p device to @p out.
  ///
  /// Lists:
  /// - @c CL_DEVICE_NAME
  /// - @c CL_DEVICE_VERSION
  /// @param out Stream to write to.
  /// @param device The device to dump information about.
  /// @param prefix Optional prefix to add to each line to @p out.
  /// @param endl End of line character used.
  void printDeviceInfo(std::ostream &out, const cl::Device &device, const char *prefix, const char *endl = "\n");

  /// A helper error logging function.
  ///
  /// Logs an error message to @p out if @p clerr is not CL_SUCCESS.
  /// The message is formatted:
  /// @code{.unparsed}
  ///   out << "Error " << context [ << index << ] " : " clu::errorCodeString(clerr) << std::endl;
  /// @endcode
  /// Where the 'index' is conditionally inserted when it is not negative.
  ///
  /// @param out The stream to log to.
  /// @param clerr The error value.
  /// @param context A context for the error string (user defined).
  /// @param index An index associated with @p context - e.g., context = "arg" index = 3.
  /// @return True if @p clerr is @c CL_SUCCESS, false otherwise.
  inline bool checkError(std::ostream &out, cl_int clerr, const char *context, cl_uint index = ~cl_uint(0u))
  {
    if (clerr != CL_SUCCESS)
    {
      out << "Error " << context;
      if (index != ~cl_uint(0u))
      {
        out << index;
      }
      out << " : " << clu::errorCodeString(clerr) << std::endl;
      return false;
    }
    return true;
  }
}

#endif // CLU_H_
