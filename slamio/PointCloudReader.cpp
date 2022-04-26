// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloudReader.h"

#include <array>

namespace slamio
{
const char *const *timeFieldNames(size_t &count)
{
  static const std::array<const char *, 8> names =  //
    {
      //
      "gps_time",       //
      "gpstime",        //
      "internal_time",  //
      "internaltime",   //
      "offset_time",    //
      "offsettime",     //
      "timestamp",      //
      "time"            //
    };
  count = names.size();
  return names.data();
}

const char *const *returnNumberFieldNames(size_t &count)
{
  {
    static const std::array<const char *, 4> names =  //
      {
        //
        "returnnumber",   //
        "return_number",  //
        "returnnum",      //
        "return_num",     //
      };
    count = names.size();
    return names.data();
  }
}

PointCloudReader::~PointCloudReader() = default;
}  // namespace slamio
