// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_OHMUTIL_H
#define OHMUTIL_OHMUTIL_H

#include "OhmUtilExport.h"

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
const size_t kKibiSize = 1024u;
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

/// Defines the set of byte units starting from bytes, kibibytes, mibibytes, etc.
enum class ByteMagnitude : int
{
  kByte,
  kKiB,
  kMiB,
  kGiB,
  kTiB,
  kPiB,
  kEiB,
  kZiB,
  kYiB
};

/// A helper structure for displaying byte values. Can be used as an io stream manipulator to write raw byte values into
/// a text stream for display.
class Bytes
{
public:
  /// Byte scale conversion values from any @c ByteMagnitude to @c ByteMagnitude::kByte.
  static const std::array<size_t, 9> ByteScale;

  /// Display string suffixes for @c ByteMagnitude values.
  static const std::array<const char *const, 9> MagnitudeSuffix;

  /// Construct a byte value optionally specifying the byte magnitude.
  /// @param byte_value The byte value to store.
  /// @param magnitude The magnitude of the byte value.
  explicit inline Bytes(size_t byte_value, ByteMagnitude magnitude = ByteMagnitude::kByte)
    : value_(byte_value)
    , magnitude_(magnitude)
  {}

  /// Copy constructor.
  /// @param other Object to copy
  inline Bytes(const Bytes &other) = default;
  /// Assignment operator.
  /// @param other Object to copy
  /// @return @c *this
  inline Bytes &operator=(const Bytes &other) = default;

  /// Static conversion from any @c ByteMagnitude into @c ByteMagnitude::kByte.
  /// @param value The byte value
  /// @param magnitude The magnitude of @c value
  /// @return The @c value expressed in bytes.
  static size_t byteSize(size_t value, ByteMagnitude magnitude) { return value * ByteScale[int(magnitude)]; }

  /// Convert this value into bytes regardless of stored @c ByteMagnitude.
  /// @return The @c value() expressed in bytes.
  inline size_t byteSize() const { return byteSize(value_, magnitude_); }

  /// Query the current @c value().
  /// @return The current byte value.
  inline size_t value() const { return value_; }
  /// Set a new @c value. The @c magnitude() is unaffected.
  /// @param value The new value.
  inline void setValue(size_t value) { value_ = value; }
  /// Query the current stored magnitude.
  /// @return The stored @c ByteMagnitude.
  inline ByteMagnitude magnitude() const { return magnitude_; }
  /// Set the @c magnitude. @c value() is unaffected.
  /// @param magnitude The new @c ByteMagnitude to set.
  inline void setMagnitude(ByteMagnitude magnitude) { magnitude_ = magnitude; }

  /// Convert this byte value to a new magnitude. Precision may be lost.
  /// @param magnitude The new magnitude to convert to.
  /// @return A @c Byte object at the desired @c ByteMagnitude expressing this object's byte value (approx).
  inline Bytes convertTo(ByteMagnitude magnitude) const
  {
    const size_t raw_byte_size = byteSize();
    const size_t inverse_scale = ByteScale[int(magnitude)];
    return Bytes(raw_byte_size / inverse_scale, magnitude);
  }

  /// Convert the byte value to the most compact string representation expressed to three decimal places and appropriate
  /// @c MagnitudeSuffix.
  /// @return A string representation of this object.
  std::string toString() const;

private:
  size_t value_ = 0;
  ByteMagnitude magnitude_ = ByteMagnitude::kByte;
};

}  // namespace util
}  // namespace ohm

/// Convert a byte value to a memory usage display string converting to the largest appropriate byte unit.
/// For example, this displays 2048 bytes as 2KiB, rather than a byte value. String display supports up to exbibyte.
///
/// Note: unlink harddrive manufacturers, this function uses base 1024 units not base 1000.
/// @param[out] str The result is written here.
/// @param bytes The byte value to convert.
inline std::ostream &operator<<(std::ostream &out, const ohm::util::Bytes &bytes)
{
  unsigned magnitude_index = 0;
  uint64_t prev_bytes = 0;
  std::array<char, 3> decimal_places = { 0, 0, 0 };
  bool need_fractional = false;
  size_t bytes_value = bytes.byteSize();
  while (bytes_value >= ohm::util::kKibiSize && magnitude_index < ohm::util::Bytes::MagnitudeSuffix.size())
  {
    prev_bytes = bytes_value;
    bytes_value /= ohm::util::kKibiSize;
    ++magnitude_index;
  }

  if (magnitude_index)
  {
    // Use prevBytes for fractional display, but only to 3 decimal places.
    prev_bytes = prev_bytes % ohm::util::kKibiSize;
    // Convert to fixed point thousands.
    prev_bytes *= ohm::util::kKibiSize;
    prev_bytes /= ohm::util::kKibiSize;
    for (int i = 0; i < 3 && prev_bytes; ++i)
    {
      need_fractional = true;
      decimal_places[2 - i] = char(prev_bytes % 10);             // NOLINT(readability-magic-numbers)
      prev_bytes = (prev_bytes) ? prev_bytes / 10 : prev_bytes;  // NOLINT(readability-magic-numbers)
    }
  }

  out << bytes_value;
  if (need_fractional)
  {
    out << '.';
    for (int i = 0; i < 3 && decimal_places[i] > 0; ++i)
    {
      out << int(decimal_places[i]);
    }
  }
  out << ohm::util::Bytes::MagnitudeSuffix[magnitude_index];
  return out;
}

inline std::string ohm::util::Bytes::toString() const
{
  std::ostringstream s;
  s << *this;
  return s.str();
}

template <typename T, typename R>
inline std::ostream &operator<<(std::ostream &out, const std::chrono::duration<T, R> &d)
{
  ohm::util::logDuration(out, d);
  return out;
}

#endif  // OHMUTIL_OHMUTIL_H
