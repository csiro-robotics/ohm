// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef LOGUTIL_LOGUTIL_H
#define LOGUTIL_LOGUTIL_H

#include "LogUtilConfig.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace logutil
{
///
const size_t kKibiSize = 1024u;

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
class logutil_API Bytes
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

  /// Default constructor. Zero byte size.
  inline Bytes()
    : Bytes(0u)
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
  inline static size_t byteSize(size_t value, ByteMagnitude magnitude) { return value * ByteScale[int(magnitude)]; }

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
  const size_t kThousand = 1000u;
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

/// Parse a @c Byte value from @p in.
///
/// The expected format must be a number followed by a valid @c Bytes::MagnitudeSuffix : `<number>[B,KiB,GiB,...]`
///
/// The number specifies the byte value and the suffix identifies the magnitude.
///
/// The number must be zero or positive. It may also be a floating point value in which case it is converted into a the
/// magnitude below. Such a non-integer value is not valid for the 'B' (bytes) suffix.
///
/// The suffix is case insensitive.
///
/// The suffix may be omitted when @p read_suffix is false, in which case the `<number>` must be an integer byte value.
///
/// On failure, the stream fail bit is set.
///
/// @param in The stream to read from.
/// @param bytes The byte structure to parse into.
/// @return True on success.
bool logutil_API parseBytes(std::istream &in, Bytes &bytes, bool read_suffix = true);

/// Convert a byte value to a memory usage display string converting to the largest appropriate byte unit.
/// For example, this displays 2048 bytes as 2KiB, rather than a byte value. String display supports up to exbibyte.
///
/// Note: unlike hard-drive manufacturers, this function uses base 1024 units not base 1000.
/// @param[out] str The result is written here.
/// @param bytes The byte value to convert.
inline std::ostream &operator<<(std::ostream &out, const Bytes &bytes)
{
  unsigned magnitude_index = 0;
  uint64_t prev_bytes = 0;
  std::array<char, 3> decimal_places = { 0, 0, 0 };
  bool need_fractional = false;
  size_t bytes_value = bytes.byteSize();
  while (bytes_value >= kKibiSize && magnitude_index < Bytes::MagnitudeSuffix.size())
  {
    prev_bytes = bytes_value;
    bytes_value /= kKibiSize;
    ++magnitude_index;
  }

  if (magnitude_index)
  {
    // Use prevBytes for fractional display, but only to 3 decimal places.
    prev_bytes = prev_bytes % kKibiSize;
    // Convert to fixed point thousands.
    prev_bytes *= kKibiSize;
    prev_bytes /= kKibiSize;
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
  out << Bytes::MagnitudeSuffix[magnitude_index];
  return out;
}

inline std::istream &operator>>(std::istream &in, Bytes &bytes)
{
  parseBytes(in, bytes);
  return in;
}

inline std::string Bytes::toString() const
{
  std::ostringstream s;
  s << *this;
  return s.str();
}
}  // namespace logutil

namespace std
{
namespace chrono
{
template <typename T, typename R>
inline ostream &operator<<(ostream &out, const duration<T, R> &d)
{
  logutil::logDuration(out, d);
  return out;
}
}  // namespace chrono
}  // namespace std

#endif  // LOGUTIL_LOGUTIL_H
