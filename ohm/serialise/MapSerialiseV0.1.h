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
int load(InputStream &stream, OccupancyMapDetail &detail,  // NOLINT(google-runtime-references)
         SerialiseProgress *progress, const MapVersion &version, size_t region_count);
int load(InputStream &stream, OccupancyMapDetail &detail,  // NOLINT(google-runtime-references)
         SerialiseProgress *progress, const MapVersion &version, size_t region_count, const ChunkFunc &load_chunk);
int loadLayout(InputStream &stream, OccupancyMapDetail &map);  // NOLINT(google-runtime-references)
int loadChunk(InputStream &stream, MapChunk &chunk,            // NOLINT(google-runtime-references)
              const OccupancyMapDetail &detail);
}  // namespace v0_1
}  // namespace ohm

#endif  // MAPSERIALISEV0_1_H
