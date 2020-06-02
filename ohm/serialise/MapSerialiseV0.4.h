// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPSERIALISEV0_3_H
#define MAPSERIALISEV0_3_H

#include "OhmConfig.h"

#include "MapSerialiseV0.1.h"

namespace ohm
{
  class InputStream;
  struct MapChunk;
  struct MapVersion;
  struct OccupancyMapDetail;
  class SerialiseProgress;

  namespace v0_4
  {
    int load(InputStream &stream, OccupancyMapDetail &detail,  // NOLINT(google-runtime-references)
             SerialiseProgress *progress, const MapVersion &version, size_t region_count);

    int load(InputStream &stream, OccupancyMapDetail &detail,  // NOLINT(google-runtime-references)
             SerialiseProgress *progress, const MapVersion &version, size_t region_count, const ChunkFunc &load_chunk);

    int loadChunk(InputStream &stream, MapChunk &chunk,  // NOLINT(google-runtime-references)
                  const OccupancyMapDetail &detail);
  }  // namespace v0_4
}  // namespace ohm

#endif  // MAPSERIALISEV0_3_H
