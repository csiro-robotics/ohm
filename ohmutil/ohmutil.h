// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_H_
#define OHMUTIL_H_

#include "ohmutilexport.h"

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
    D absDuration = (!negative) ? duration : duration * -1;
    auto s = std::chrono::duration_cast<std::chrono::seconds>(absDuration).count();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(absDuration).count();
    ms = ms % 1000;

    if (s)
    {
      out << sign << s << "." << std::setw(3) << std::setfill('0') << ms << "s";
    }
    else
    {
      auto us = std::chrono::duration_cast<std::chrono::microseconds>(absDuration).count();
      us = us % 1000;

      if (ms)
      {
        out << sign << ms << "." << std::setw(3) << std::setfill('0') << us << "ms";
      }
      else
      {
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(absDuration).count();
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
  /// @param timeStr The string to populate with the result.
  /// @param duration The duration to convert to string.
  template <typename D>
  inline void timeString(std::string &timeStr, const D &duration)
  {
    std::ostringstream out;
    logDuration(out, duration);
    out.flush();
    timeStr = out.str();
  }

  /// Convert a byte value to a memory usage display string converting to the largest apporpriate byte unit.
  /// For example, this displays 2048 bytes as 2KiB, rather than a byte value. String display supports up to exbibyte.
  ///
  /// Note: unlink harddrive manufacturers, this function uses base 1024 units not base 1000.
  /// @param[out] str The result is written here.
  /// @param bytes The byte value to convert.
  inline void makeMemoryDisplayString(std::string &str, uint64_t bytes)
  {
    const char *unitSuffix[] =
    {
      "B",
      "KiB",
      "MiB",
      "GiB",
      "TiB",
      "PiB",
      "EiB"
    };

    int unitIndex = 0;
    uint64_t prevBytes = 0;
    char decimalPlaces[3] = { 0, 0, 0 };
    bool needFractional = false;
    while (bytes > 1024u && unitIndex < sizeof(unitSuffix) / sizeof(unitSuffix[0]))
    {
      prevBytes = bytes;
      bytes /= 1024u;
      ++unitIndex;
    }

    if (unitIndex)
    {
      // Use prevBytes for fractional display, but only to 3 decimal places.
      prevBytes = prevBytes % 1024u;
      // Convert to fixed point thousands.
      prevBytes *= 1024;
      prevBytes /= 1000;
      for (int i = 0; i < 3 && prevBytes; ++i)
      {
        needFractional = true;
        decimalPlaces[2 - i] = char(prevBytes % 10);
        prevBytes = (prevBytes) ? prevBytes / 10 : prevBytes;
      }
    }

    std::ostringstream out;
    out << bytes;
    if (needFractional)
    {
      out << '.';
      for (int i = 0; i < 3 && decimalPlaces[i] > 0; ++i)
      {
        out << int(decimalPlaces[i]);
      }
    }
    out << ' ' << unitSuffix[unitIndex];
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

#endif // OHMUTIL_H_
