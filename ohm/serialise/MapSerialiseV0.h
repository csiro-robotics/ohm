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
int load(InputStream &stream, OccupancyMapDetail &detail, SerialiseProgress *progress, const MapVersion &version,
         size_t region_count);
int loadChunk(InputStream &stream, MapChunk &chunk, const OccupancyMapDetail &detail);
}  // namespace v0
}  // namespace ohm

#endif  // MAPSERIALISEV0_H
