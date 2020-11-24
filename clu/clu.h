//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#ifndef CLU_H
#define CLU_H

#include "cluConfig.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif  // __GNUC__
#include "cl2.hpp"
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif  // __GNUC__

#include <functional>
#include <iostream>
#include <list>
#include <vector>

namespace clu
{
/// Constraint function prototype used in platform selection.
using PlatformConstraint = std::function<bool(const cl::Platform &)>;
/// Constraint function prototype used in device selection.
using DeviceConstraint = std::function<bool(const cl::Platform &, const cl::Device &)>;

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
/// @param constraint_count The number of @p constraints.
/// @return The first valid platform found, passing all constraints.
cl::Platform createPlatform(cl_device_type type, const PlatformConstraint *constraints = nullptr,
                            unsigned constraint_count = 0);
/// Create the OpenCL platform optionally applying the given constraints.
///
/// The selected device must pass all the @p constraints given, each returning true, in order
/// to be selected. If no platform passes all tests, then an invalid platform is returned.
///
/// @param type The required OpenCL device type.
/// @param constraints The array of constraints to pass or empty for no constraints.
/// @return The first valid platform found, passing all constraints.
cl::Platform createPlatform(cl_device_type type, const std::vector<PlatformConstraint> &constraints);

/// Filter the given list of @p platforms, removing all which do not match @p type or pass
/// @p constraints.
///
/// See @c clucontraint.h for some constraint examples.
///
/// @param[in,out] platforms The Platforms to filter.
/// @param type The required device type(s).
/// @param constraints The constraints to filter by.
/// @param constraint_count The number of @p constraints.
/// @return True if there is at least one valid platform left in @p platforms.
bool filterPlatforms(std::vector<cl::Platform> &platforms, cl_device_type type, const PlatformConstraint *constraints,
                     unsigned constraint_count);

/// Filter the given list of @p devices, removing all which do not match @p type or pass
/// @p constraints.
///
/// See @c clucontraint.h for some constraint examples.
///
/// @param platform The platform to which the @p devices belong.
/// @param[in,out] devices The Platforms to filter.
/// @param constraints The device constraints to filter by.
/// @param constraint_count The number of @p constraints.
/// @return True if there is at least one valid device left in @p devices.
bool filterDevices(const cl::Platform &platform, std::vector<cl::Device> &devices, const DeviceConstraint *constraints,
                   unsigned constraint_count);

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
#endif  // #

/// Create the OpenCL platform and context optionally applying the given platform and
/// device constraints.
///
/// The selected platform must pass all @p platformConstraints while the selected
/// context must pass all @p deviceContraints.
///
/// See @c clucontraint.h for some constraint examples.
///
/// @param device_out Set to the selected OpenCL device.
/// @param type The required OpenCL device type.
/// @param platform_constraint The array of platform constraints to pass or null for no constraints.
/// @param platform_constraint_count The number of @p platformConstraint.
/// @param device_constraints The array of device constraints to pass or null for no constraints.
/// @param device_constraint_count The number of @p deviceConstraints.
/// @return The first valid context found, passing all constraints.
cl::Context createContext(cl::Device *device_out, cl_device_type type, const PlatformConstraint *platform_constraint,
                          unsigned platform_constraint_count, const DeviceConstraint *device_constraints,
                          unsigned device_constraint_count);

/// Initialise the primary OpenCL context. Retrieved via @c getPrimaryContext().
/// @param device The device to set.
/// @param context The context to set.
bool setPrimaryContext(const cl::Context &context, const cl::Device &device);
/// Releases and clears the primary context available from @c getPrimaryContext().
void clearPrimaryContext();
/// Initialise the primary context (@c getPrimaryContext()) according to the given constraints.
/// @param type The required OpenCL device type.
/// @param platform_constraint The array of platform constraints to pass or null for no constraints.
/// @param platform_constraint_count The number of @p platformConstraint.
/// @param device_constraints The array of device constraints to pass or null for no constraints.
/// @param device_constraint_count The number of @p deviceConstraints.
/// @return True if all constraints are met and the context initialised.
bool initPrimaryContext(cl_device_type type, const PlatformConstraint *platform_constraint,
                        unsigned platform_constraint_count, const DeviceConstraint *device_constraints,
                        unsigned device_constraint_count);
/// Initialise the primary context (@c getPrimaryContext()) according to the given constraints.
/// @param type The required OpenCL device type.
/// @param platform_constraint The array of platform constraints to pass or null for no constraints.
/// @param device_constraints The array of device constraints to pass or null for no constraints.
/// @return True if all constraints are met and the context initialised.
bool initPrimaryContext(cl_device_type type, const std::vector<PlatformConstraint> &platform_constraint,
                        const std::vector<DeviceConstraint> &device_constraints);
/// Get the stored primary context.
/// @param[out] context Set to the primary context.
/// @param device The OpenCL device to operate on.
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
/// clver    | 1.0, 1.2, 2.0, ...   | The minimum required OpenCL version; @c deviceVersionMin()
/// device   | &lt;string&gt;       | Uses @c deviceNameLike() to match the device name.
/// platform | &lt;string&gt;       | Uses @c platformNameLike() to match the platform name.
/// vendor   | &lt;string&gt;       | Uses @c platformVendorLike() to match the platform vendor.
///
/// @param argc Number of arguments in @p argv.
/// @param argv Command line arguments.
/// @param[out] type Set to the requested @c accel type or @c CL_DEVICE_TYPE_ALL.
/// @param platform_constraints Populated with requested platform constraints.
/// @param device_constraints Populated with requested device constraints.
/// @param arg_prefix Optional prefix to required an all arguments described above.
void constraintsFromCommandLine(int argc, const char **argv, cl_device_type &type,
                                std::vector<PlatformConstraint> &platform_constraints,
                                std::vector<DeviceConstraint> &device_constraints, const char *arg_prefix = nullptr);

/// @overload
inline void constraintsFromCommandLine(int argc, char **argv, cl_device_type &type,
                                       std::vector<PlatformConstraint> &platform_constraints,
                                       std::vector<DeviceConstraint> &device_constraints,
                                       const char *arg_prefix = nullptr)
{
  return constraintsFromCommandLine(argc,
                                    const_cast<const char **>(argv),  // NOLINT(cppcoreguidelines-pro-type-const-cast)
                                    type, platform_constraints, device_constraints, arg_prefix);
}

/// @overload
void constraintsFromArgs(const std::list<std::string> &args, cl_device_type &type,
                         std::vector<PlatformConstraint> &platform_constraints,
                         std::vector<DeviceConstraint> &device_constraints, const char *arg_prefix = nullptr);

/// Parse an OpenCL version string. This supports both device and platform version strings of the form:
/// "OpenCL <major>.<minor>".
/// @param version_string The version string to parse.
/// @param[out] version_major The parsed major version.
/// @param[out] version_minor The parsed minor version.
/// @param True on success.
bool parseVersion(const char *version_string, cl_uint *version_major, cl_uint *version_minor);

/// Resolve the platform version.
/// @param platform The platform to get the version for.
/// @param[out] version_major The parsed major version.
/// @param[out] version_minor The parsed minor version.
void platformVersion(cl_platform_id platform, cl_uint *version_major, cl_uint *version_minor);

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
void printPlatformInfo(std::ostream &out, const cl::Platform &platform, const char *prefix = "",
                       const char *endl = "\n");

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
}  // namespace clu

#endif  // CLU_H
