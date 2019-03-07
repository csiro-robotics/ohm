//
// Project configuration header. This is a generated header; do not modify
// it directly. Instead, modify the config.h.in version and run CMake again.
//
#ifndef OHMGPUCONFIG_H
#define OHMGPUCONFIG_H

#include "OhmGpuExport.h"

#include <ohm/OhmConfig.h>

// clang-format off

/// Target OpenCL standard. 'max' => maximum device version (min 1.2)
#define OHM_OPENCL_STD "@OHM_OPENCL_STD@"
/// OpenCL required features to enable OpenCL 2.0 code.
#define OHM_OPENCL_2_FEATURES "@OHM_OPENCL_2_FEATURES@"

// clang-format on

#endif  // OHMGPUCONFIG_H
