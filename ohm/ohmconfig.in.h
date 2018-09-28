//
// Project configuration header. This is a generated header; do not modify
// it directly. Instead, modify the config.h.in version and run CMake again.
//
#ifndef OHMCONFIG_H_
#define OHMCONFIG_H_

#include "ohmexport.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif // _USE_MATH_DEFINES
#ifndef NOMINMAX
#define NOMINMAX
#endif // NOMINMAX
#include <cmath>

#ifdef _MSC_VER
// Avoid dubious security warnings for plenty of legitimate code
# ifndef _SCL_SECURE_NO_WARNINGS
#   define _SCL_SECURE_NO_WARNINGS
# endif // _SCL_SECURE_NO_WARNINGS
# ifndef _CRT_SECURE_NO_WARNINGS
#   define _CRT_SECURE_NO_WARNINGS
# endif // _CRT_SECURE_NO_WARNINGS
//#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif // _MSC_VER

// Enable various validation tests throughout this library.
#cmakedefine OHM_VALIDATION
#cmakedefine OHM_THREADS
#cmakedefine OHM_PROFILE
#cmakedefine OHM_EMBED_GPU_CODE

// Enable GPU acceleration.
/// GPU usage: 0 for off, one of the OHM_GPU_XXX values when on.
#define OHM_GPU @OHM_GPU@

#define OHM_GPU_OPENCL @OHM_GPU_OPENCL@
#define OHM_GPU_CUDA @OHM_GPU_CUDA@

#define OHM_OPENCL_STD "@OHM_OPENCL_STD@"

#cmakedefine TES_ENABLE
#ifdef TES_ENABLE
namespace tes { class Server; }
namespace ohm
{
  /// Debug visualisation server pointer. Must be set by the executable to enable its use in this library.
  /// That is, this is considered a borrowed pointer in this library.
  extern tes::Server *g_3es; // Symbol defined in occupancyutil.cpp.
}
#endif // TES_ENABLE

#endif // OHMCONFIG_H_
