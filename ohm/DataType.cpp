// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "DataType.h"

#include <array>

namespace ohm
{
const char *DataType::name(unsigned type)
{
  static const std::array<const char *, 12> names =  //
    {
      "<unknown>",  //
      "int8_t",     //
      "uint8_t",    //
      "int16_t",    //
      "uint16_t",   //
      "int32_t",    //
      "uint32_t",   //
      "int64_t",    //
      "uint64_t",   //
      "float",      //
      "double",     //
      "struct"      //
    };

  if (type < kUser)
  {
    if (type < names.size())
    {
      return names[type];
    }

    return "<invalid>";
  }

  return "user";
}


size_t DataType::size(unsigned type)
{
  static const std::array<size_t, 12> sizes =  //
    {
      0u,  // unknown
      1u,  // int8
      1u,  // uint8
      2u,  // int16
      2u,  // uin16
      4u,  // int32
      4u,  // uint32
      8u,  // int64
      8u,  // uint64
      4u,  // float
      8u,  // double
      0u   // struct
    };

  if (type < kUser)
  {
    if (type < sizes.size())
    {
      return sizes[type];
    }

    return 0u;
  }

  return 0u;
}
}  // namespace ohm
