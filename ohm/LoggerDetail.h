// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_LOGGERDETAIL_H
#define OHM_LOGGERDETAIL_H

#include "OhmConfig.h"

#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace ohm
{
namespace logger
{
namespace detail
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
}  // namespace detail
}  // namespace logger
}  // namespace ohm

#endif  // OHM_LOGGERDETAIL_H
