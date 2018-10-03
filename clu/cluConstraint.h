#ifndef CLUCONSTRAINT_H
#define CLUCONSTRAINT_H

#include "clu.h"

namespace clu
{
  /// Create a platform constraint requiring the @c CL_PLATFORM_NAME is line @p name.
  ///
  /// To pass the @c CL_PLATFORM_NAME must contain the @p name string optionally ignoring
  /// case.
  /// @param name The partial name match required.
  /// @param ignore_case True for case insensitive checks.
  /// @return The platform constraint function object.
  PlatformContraint platformNameLike(const char *name, bool ignore_case);

  /// Create a platform constraint requiring the @c CL_PLATFORM_VENDOR is line @p name.
  ///
  /// To pass the @c CL_PLATFORM_VENDOR must contain the @p name string optionally ignoring
  /// case.
  /// @param name The partial name match required.
  /// @param ignore_case True for case insensitive checks.
  /// @return The platform constraint function object.
  PlatformContraint platformVendorLike(const char *name, bool ignore_case);

  /// Create a device version constraint.
  /// The device must have an OpenCL version at least as high as @p major, @p minor version
  /// The version may be higher.
  ///
  /// For example, @c deviceVersionIs(2, 0) creates a constraint requiring OpenCL 2.0 or
  /// higher.
  ///
  /// @param major The minimum required major version.
  /// @param minor The minimum required minor version.
  /// @return The device constraint function object.
  DeviceConstraint deviceVersionIs(int major, int minor);

  /// Create a device constraint requiring the @c CL_DEVICE_NAME is line @p name.
  ///
  /// To pass the @c CL_DEVICE_NAME must contain the @p name string optionally ignoring
  /// case.
  /// @param name The partial name match required.
  /// @param ignore_case True for case insensitive checks.
  /// @return The device constraint function object.
  DeviceConstraint deviceNameLike(const char *name, bool ignore_case);

  /// Create a device constraint requiring the @c CL_PLATFORM_VENDOR is line @p name.
  ///
  /// To pass the @c CL_DEVICE_VENDOR must contain the @p name string optionally ignoring
  /// case.
  /// @param name The partial name match required.
  /// @param ignore_case True for case insensitive checks.
  /// @return The device constraint function object.
  DeviceConstraint deviceVendorLike(const char *name, bool ignore_case);
}

#endif
