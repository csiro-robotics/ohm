// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPSERIALISEV0_1_H
#define MAPSERIALISEV0_1_H

#include "OhmConfig.h"

#include <functional>

namespace ohm
{
class InputStream;
struct MapChunk;
struct MapVersion;
struct OccupancyMapDetail;
class SerialiseProgress;

using ChunkFunc = std::function<int(InputStream &stream, MapChunk &, const OccupancyMapDetail &)>;

namespace v0_1
{
int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
         size_t region_count);
int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
         size_t region_count, const ChunkFunc &load_chunk);
int loadLayout(InputStream &stream, OccupancyMapDetail &map);
int loadChunk(InputStream &stream, MapChunk &chunk, const OccupancyMapDetail &detail);
}  // namespace v0_1
}  // namespace ohm

#endif  // MAPSERIALISEV0_1_H
