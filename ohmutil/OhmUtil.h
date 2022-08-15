// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_OHMUTIL_H
#define OHMUTIL_OHMUTIL_H

#include <ohmutil/OhmUtilExport.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace ohm
{
namespace util
{
const size_t kThousand = 1000u;
/// Log a @c std::chrono::clock::duration to an output stream.
///
/// The resulting string displays in the smallest possible unit to show three three
/// decimal places with display units ranging from seconds to nanoseconds. The table below
/// shows some example times.
///
/// Time(s)     | Display
/// ----------- | --------
/// 0.000000018 | 18ns
/// 0.000029318 | 29.318us
/// 0.0295939   | 29.593ms
/// 0.93        | 930ms
/// 15.023      | 15.023s
/// 15.000025   | 15.000s
///
/// Note that times are truncated, not rounded.
///
/// @tparam D The duration type of the form @c std::chrono::clock::duration.
/// @param out The output stream to log to.
/// @param duration The duration to convert to string.
template <typename D>
inline void logDuration(std::ostream &out, const D &duration)
{
  const bool negative = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() < 0;
  const char *sign = (!negative) ? "" : "-";
  D abs_duration = (!negative) ? duration : duration * -1;
  auto s = std::chrono::duration_cast<std::chrono::seconds>(abs_duration).count();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(abs_duration).count();
  ms = ms % kThousand;

  if (s)
  {
    out << sign << s << "." << std::setw(3) << std::setfill('0') << ms << "s";
  }
  else
  {
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(abs_duration).count();
    us = us % kThousand;

    if (ms)
    {
      out << sign << ms << "." << std::setw(3) << std::setfill('0') << us << "ms";
    }
    else
    {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(abs_duration).count();
      ns = ns % kThousand;

      if (us)
      {
        out << sign << us << "." << std::setw(3) << std::setfill('0') << ns << "us";
      }
      else
      {
        out << sign << ns << "ns";
      }
    }
  }
}

/// Convert a @c std::chrono::clock::duration type to a display string using the same
/// logic as @c logTime().
/// @tparam D The duration type of the form @c std::chrono::clock::duration.
/// @param time_str The string to populate with the result.
/// @param duration The duration to convert to string.
template <typename D>
inline void timeString(std::string &time_str, const D &duration)
{
  std::ostringstream out;
  logDuration(out, duration);
  out.flush();
  time_str = out.str();
}

template <typename N>
void delimitedInteger(std::string &str, const N &integer, char delimiter = ',')
{
  N thousands = integer % ohm::util::kThousand;
  N remainder = integer / ohm::util::kThousand;

  str.clear();
  while (remainder > 0 || thousands > 0)
  {
    std::ostringstream os;
    if (remainder)
    {
      os << delimiter << std::setfill('0') << std::setw(3);
    }
    os << thousands << str;
    str = os.str();
    thousands = remainder % ohm::util::kThousand;
    remainder = remainder / ohm::util::kThousand;
  }
}
}  // namespace util
}  // namespace ohm

template <typename T, typename R>
inline std::ostream &operator<<(std::ostream &out, const std::chrono::duration<T, R> &d)
{
  ohm::util::logDuration(out, d);
  return out;
}

#endif  // OHMUTIL_OHMUTIL_H
