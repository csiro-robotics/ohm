// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SERIALISEUTIL_H
#define SERIALISEUTIL_H

#include "OhmConfig.h"

#include "Stream.h"

namespace ohm
{
  /// Explicitly typed stream writing, uncompressed.
  template <typename T, typename S>
  inline bool writeUncompressed(OutputStream &stream, const S &val)
  {
    const T val2 = static_cast<T>(val);
    return stream.writeUncompressed(&val2, unsigned(sizeof(val2))) == sizeof(val2);
  }


  /// Explicitly typed stream writing.
  template <typename T, typename S>
  inline bool write(OutputStream &stream, const S &val)
  {
    const T val2 = static_cast<T>(val);
    return stream.write(&val2, unsigned(sizeof(val2))) == sizeof(val2);
  }


  /// Explicitly typed stream reading, uncompressed.
  template <typename T, typename S>
  inline bool readRaw(InputStream &stream, S &val)
  {
    T val2{ 0 };
    if (stream.readRaw(&val2, unsigned(sizeof(val2))) != sizeof(val2))
    {
      return false;
    }
    val = static_cast<S>(val2);
    return true;
  }


  /// Explicitly typed stream reading.
  template <typename T, typename S>
  inline bool read(InputStream &stream, S &val)
  {
    T val2{ 0 };
    if (stream.read(&val2, unsigned(sizeof(val2))) != sizeof(val2))
    {
      return false;
    }
    val = static_cast<S>(val2);
    return true;
  }
}

#endif // SERIALISEUTIL_H
