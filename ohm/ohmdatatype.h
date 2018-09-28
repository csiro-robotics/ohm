// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDATATYPE_H_
#define OHMDATATYPE_H_

#include "ohmconfig.h"

namespace ohm
{
  /// Data type enumeration, particularly for @c VoxelLayout data.
  struct DataType
  {
    enum Type
    {
      Unknown,
      Int8,
      UInt8,
      Int16,
      UInt16,
      Int32,
      UInt32,
      Int64,
      UInt64,
      Float,
      Double,
      // NYI: May implement in future to support more interesting voxel structures.
      Struct,

      User = 256
    };

    /// Returns a string representation of @p type.
    /// @param type The data type value to convert to string.
    /// @return A string name for @p type or "&lt;invalid&gt;" if @p type is invalid. Any user type is "user".
    static const char *name(unsigned type);

    /// Returns the byte size of @p type if known.
    /// @param type The data type value to return a byte size for.
    /// @return The byte size of @p type or zero if the size is unknown, variable or @p type is invalid.
    static size_t size(unsigned type);
  };
}

#endif // OHMDATATYPE_H_
