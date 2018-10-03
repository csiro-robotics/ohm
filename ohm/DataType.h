// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_DATATYPE_H
#define OHM_DATATYPE_H

#include "OhmConfig.h"

namespace ohm
{
  /// Data type enumeration, particularly for @c VoxelLayout data.
  struct DataType
  {
    enum Type
    {
      kUnknown,
      kInt8,
      kUInt8,
      kInt16,
      kUInt16,
      kInt32,
      kUInt32,
      kInt64,
      kUInt64,
      kFloat,
      kDouble,
      // NYI: May implement in future to support more interesting voxel structures.
      kStruct,

      kUser = 256
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

#endif // OHM_DATATYPE_H
