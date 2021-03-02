// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmUtil.h"

namespace ohm::util
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
}  // namespace ohm::util
