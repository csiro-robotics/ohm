// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMHEIGHTMAP_HEIGHTMAPMAPSERIALISE_H
#define OHMHEIGHTMAP_HEIGHTMAPMAPSERIALISE_H

#include "OhmHeightmapConfig.h"

#include <ohm/MapSerialise.h>

#include <glm/fwd.hpp>

#include <cinttypes>
#include <string>

namespace ohm
{
class Heightmap;

/// An enumeration of potential serialisation errors.
enum HeightmapSerialisationError
{
  /// @c MapInfo does not represent a heightmap.
  kSeHeightmapInfoMismatch = kSeExtensionCode + 1,
};

/// Load a save heightmap into a @c Heightmap object. Saving can be done directly on the @c Occupancy map stored in
/// @c Heightmap::heightmap() .
/// @param filename The heightmap file path to load.
/// @param heightmap The heightmap object to load into.
/// @param progress Optional progress reporting interface.
/// @param[out] version_out Set to the map file version number if provided.
int ohmheightmap_API load(const std::string &filename, Heightmap &heightmap, SerialiseProgress *progress = nullptr,
                          MapVersion *version_out = nullptr);
}  // namespace ohm

#endif  // OHMHEIGHTMAP_HEIGHTMAPMAPSERIALISE_H
