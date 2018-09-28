// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmdatatype.h"

using namespace ohm;

const char *DataType::name(unsigned type)
{
  static const char *names[] =
  {
    "<unknown>",
    "int8_t",
    "uint8_t",
    "int16_t",
    "uint16_t",
    "int32_t",
    "uint32_t",
    "int64_t",
    "uint64_t",
    "float",
    "double",
    "struct",
  };

  if (type < User)
  {
    if (type < sizeof(names) / sizeof(names[0]))
    {
      return names[type];
    }

    return "<invalid>";
  }

  return "user";
}


size_t DataType::size(unsigned type)
{
  static size_t sizes[] =
  {
    0u,
    1u,
    1u,
    2u,
    2u,
    4u,
    4u,
    8u,
    8u,
    4u,
    8u,
    0u,
  };

  if (type < User)
  {
    if (type < sizeof(sizes) / sizeof(sizes[0]))
    {
      return sizes[type];
    }

    return 0u;
  }

  return 0u;
}
