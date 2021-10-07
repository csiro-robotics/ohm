// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPSERIALISEV0_5_H
#define MAPSERIALISEV0_5_H

#include "OhmConfig.h"

#include "MapSerialiseV0.4.h"

namespace ohm
{
namespace v0_5
{
inline int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
                size_t region_count)
{
  return v0_4::load(stream, detail, progress, version, region_count);
}
}  // namespace v0_5
}  // namespace ohm

#endif  // MAPSERIALISEV0_3_H
