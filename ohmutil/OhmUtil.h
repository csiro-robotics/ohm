// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_OHMUTIL_H
#define OHMUTIL_OHMUTIL_H

#include "OhmUtilExport.h"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace util
{
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
    ms = ms % 1000;

    if (s)
    {
      out << sign << s << "." << std::setw(3) << std::setfill('0') << ms << "s";
    }
    else
    {
      auto us = std::chrono::duration_cast<std::chrono::microseconds>(abs_duration).count();
      us = us % 1000;

      if (ms)
      {
        out << sign << ms << "." << std::setw(3) << std::setfill('0') << us << "ms";
      }
      else
      {
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(abs_duration).count();
        ns = ns % 1000;

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

  /// Convert a byte value to a memory usage display string converting to the largest apporpriate byte unit.
  /// For example, this displays 2048 bytes as 2KiB, rather than a byte value. String display supports up to exbibyte.
  ///
  /// Note: unlink harddrive manufacturers, this function uses base 1024 units not base 1000.
  /// @param[out] str The result is written here.
  /// @param bytes The byte value to convert.
  inline void makeMemoryDisplayString(std::string &str, uint64_t bytes)
  {
    const char *unit_suffix[] =
    {
      "B",
      "KiB",
      "MiB",
      "GiB",
      "TiB",
      "PiB",
      "EiB"
    };

    unsigned unit_index = 0;
    uint64_t prev_bytes = 0;
    char decimal_places[3] = { 0, 0, 0 };
    bool need_fractional = false;
    while (bytes > 1024u && unit_index < sizeof(unit_suffix) / sizeof(unit_suffix[0]))
    {
      prev_bytes = bytes;
      bytes /= 1024u;
      ++unit_index;
    }

    if (unit_index)
    {
      // Use prevBytes for fractional display, but only to 3 decimal places.
      prev_bytes = prev_bytes % 1024u;
      // Convert to fixed point thousands.
      prev_bytes *= 1024;
      prev_bytes /= 1000;
      for (int i = 0; i < 3 && prev_bytes; ++i)
      {
        need_fractional = true;
        decimal_places[2 - i] = char(prev_bytes % 10);
        prev_bytes = (prev_bytes) ? prev_bytes / 10 : prev_bytes;
      }
    }

    std::ostringstream out;
    out << bytes;
    if (need_fractional)
    {
      out << '.';
      for (int i = 0; i < 3 && decimal_places[i] > 0; ++i)
      {
        out << int(decimal_places[i]);
      }
    }
    out << ' ' << unit_suffix[unit_index];
    out.flush();
    str = out.str();
  }
}


template <typename N>
void delimetedInteger(std::string &str, const N &integer, char delimiter = ',')
{
  N thousands = integer % 1000;
  N remainder = integer / 1000;

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
    thousands = remainder % 1000;
    remainder = remainder / 1000;
  }
}


template <typename T, typename R>
inline std::ostream &operator << (std::ostream &out, const std::chrono::duration<T, R> &d)
{
  util::logDuration(out, d);
  return out;
}

#endif // OHMUTIL_OHMUTIL_H
