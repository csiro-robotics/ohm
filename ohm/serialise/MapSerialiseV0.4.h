// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPSERIALISEV0_4_H
#define MAPSERIALISEV0_4_H

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
int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
         size_t region_count);

int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
         size_t region_count, const ChunkFunc &load_chunk);

int loadChunk(InputStream &stream, MapChunk &chunk, const OccupancyMapDetail &detail);
}  // namespace v0_4
}  // namespace ohm

#endif  // MAPSERIALISEV0_3_H
