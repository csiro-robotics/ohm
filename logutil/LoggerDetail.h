// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef LOGUTIL_LOGGERDETAIL_H
#define LOGUTIL_LOGGERDETAIL_H

#include "LogUtilConfig.h"

#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace logutil
{
namespace logger_detail
{
/// Prepare the @p out stream for logging, imbuing the locale.
///
/// Other effects:
/// - Set @c std::boolalpha
///
/// @param out The stream to prepare.
inline void prepareStream(std::ostream &out)
{
  out.imbue(std::locale(""));
  out << std::boolalpha;
}

template <typename T, typename... Args>
inline void message(std::ostream &stream, const T &value)
{
  stream << value;
}

template <typename T, typename... Args>
inline void message(std::ostream &stream, const T &value, Args... args)
{
  stream << value;
  message(stream, args...);
}
}  // namespace logger_detail
}  // namespace logutil

#endif  // LOGUTIL_LOGGERDETAIL_H
