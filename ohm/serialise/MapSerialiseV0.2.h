// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPSERIALISEV0_2_H
#define MAPSERIALISEV0_2_H

#include "OhmConfig.h"

#include "MapSerialiseV0.1.h"

namespace ohm
{
  class InputStream;
  struct MapChunk;
  class MapInfo;
  class MapValue;
  struct MapVersion;
  struct OccupancyMapDetail;
  class SerialiseProgress;

  namespace v0_2
  {
    int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
             size_t region_count);
    int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
             size_t region_count, const ChunkFunc &load_chunk);

    int loadMapInfo(InputStream &in, MapInfo &info);
    int loadItem(InputStream &in, MapValue &value);
  }  // namespace v0_2
}  // namespace ohm

#endif  // MAPSERIALISEV0_2_H
