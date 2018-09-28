//
// Project configuration header. This is a generated header; do not modify
// it directly. Instead, modify the config.h.in version and run CMake again.
//
#ifndef OHMQUERYCONFIG_H_
#define OHMQUERYCONFIG_H_

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

#cmakedefine TES_ENABLE
#ifdef TES_ENABLE
namespace tes { class Server; }
extern tes::Server *g_tesServer;
//#include "debugids.h"
#endif // TES_ENABLE

#endif // OHMQUERYCONFIG_H_
