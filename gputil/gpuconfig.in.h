// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUCONFIG_H_
#define GPUCONFIG_H_

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif // _USE_MATH_DEFINES
#ifndef NOMINMAX
#define NOMINMAX
#endif // NOMINMAX
#ifndef NOMINMAX
#define NOMINMAX
#endif // NOMINMAX
#include <cmath>

// Shared library import/export configuration:
#cmakedefine GPUTIL_SHARED

#ifdef GPUTIL_SHARED

// Built as a shared library/DLL.
# ifdef WIN32
#   ifdef GPUTIL_EXPORTS
#     define gputilAPI __declspec(dllexport)
#   else  // GPUTIL_EXPORTS
#     define gputilAPI __declspec(dllimport)
#   endif // GPUTIL_EXPORTS
#   define gputil_HIDDEN
# else  // WIN32
#   ifdef GPUTIL_EXPORTS
#     define gputilAPI __attribute__((visibility("default")))
#     define gputil_HIDDEN __attribute__((visibility("hidden")))
#   else  // GPUTIL_EXPORTS
#     define gputilAPI
#     define gputil_HIDDEN
#   endif // GPUTIL_EXPORTS
#endif // WIN32

#else  // GPUTIL_SHARED

// Not built as a shared library/DLL.
#define gputilAPI
#define gputil_HIDDEN

#endif // GPUTIL_SHARED

#define GPUTIL_NONE 0
#define GPUTIL_OPENCL 1
#define GPUTIL_CUDA 2

#define GPUTIL_TYPE @GPUTIL_TYPE@

#endif // GPUCONFIG_H_
