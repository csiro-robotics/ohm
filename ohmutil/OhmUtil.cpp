// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmUtil.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace ohm
{
namespace util
{
const std::array<size_t, 9> Bytes::ByteScale =  //
  {
    1ull,
    1024ull,
    1024ull * 1024ull,
    1024ull * 1024ull * 1024ull,
    1024ull * 1024ull * 1024ull * 1024ull,
    1024ull * 1024ull * 1024ull * 1024ull * 1024ull,
    1024ull * 1024ull * 1024ull * 1024ull * 1024ull * 1024ull,
    1024ull * 1024ull * 1024ull * 1024ull * 1024ull * 1024ull * 1024ull,
    1024ull * 1024ull * 1024ull * 1024ull * 1024ull * 1024ull * 1024ull * 1024ull,
  };

const std::array<const char *const, 9> Bytes::MagnitudeSuffix =  //
  {
    "B",    //
    "KiB",  //
    "MiB",  //
    "GiB",  //
    "TiB",  //
    "PiB",  //
    "EiB",  //
    "ZiB",  //
    "YiB",  //
  };

bool parseBytes(std::istream &in, ohm::util::Bytes &bytes, bool read_suffix)
{
  double numeric_value = 0;
  std::string suffix;

  // Read values.
  in >> numeric_value;

  // Validate it's a +ve value.
  if (numeric_value < 0)
  {
    in.setstate(std::ios::failbit);
    bytes = Bytes(0);
    return false;
  }

  if (in.good() && read_suffix)
  {
    // Need suffix.
    in >> suffix;
  }

  // Process result.
  double integer_part = 0;
  double fraction_part = std::modf(numeric_value, &integer_part);
  ByteMagnitude byte_magnitude = ByteMagnitude::kByte;
  if (suffix.empty())
  {
    // No suffix. Assume bytes. A floating point value is invalid for this.
    if (fraction_part != 0.0)
    {
      // Has a factional part. Fail.
      in.setstate(std::ios::failbit);
      bytes = Bytes(0);
      return false;
    }

    byte_magnitude = ByteMagnitude::kByte;
  }
  else
  {
    std::transform(suffix.begin(), suffix.end(), suffix.begin(), std::tolower);
    // Try match the suffix.
    bool parsed_suffix = false;
    for (size_t i = 0; i < Bytes::MagnitudeSuffix.size(); ++i)
    {
      std::string sfx = Bytes::MagnitudeSuffix[i];
      std::transform(sfx.begin(), sfx.end(), sfx.begin(), std::tolower);
      if (suffix == sfx)
      {
        byte_magnitude = ByteMagnitude(i);
        parsed_suffix = true;
        break;
      }
    }

    if (!parsed_suffix)
    {
      // Unparsed suffix.
      in.setstate(std::ios::failbit);
      bytes = Bytes(0);
      return false;
    }
  }

  // Slide the magnitude if we have a fractional part.
  while (fraction_part > 0 && byte_magnitude != ByteMagnitude::kByte &&
         unsigned(byte_magnitude) < Bytes::MagnitudeSuffix.size())
  {
    byte_magnitude = ByteMagnitude(int(byte_magnitude) + 1);
    fraction_part = std::modf(numeric_value * 1024.0, &integer_part);
  }

  // Finalise.
  bytes = Bytes(uint64_t(integer_part), byte_magnitude);
  return true;
}
}  // namespace util
}  // namespace ohm
