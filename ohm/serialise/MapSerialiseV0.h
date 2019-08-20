// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPSERIALISEV0_H
#define MAPSERIALISEV0_H

#include "OhmConfig.h"

namespace ohm
{
  class InputStream;
  struct MapChunk;
  struct MapVersion;
  struct OccupancyMapDetail;
  class SerialiseProgress;

  namespace v0
  {
    int load(InputStream &stream, OccupancyMapDetail &detail,  // NOLINT(google-runtime-references)
             SerialiseProgress *progress, const MapVersion &version, size_t region_count);
    int loadChunk(InputStream &stream, MapChunk &chunk,  // NOLINT(google-runtime-references)
                  const OccupancyMapDetail &detail);
  }  // namespace v0
}  // namespace ohm

#endif  // MAPSERIALISEV0_H
