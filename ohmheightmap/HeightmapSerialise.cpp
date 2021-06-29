// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapSerialise.h"

#include "Heightmap.h"

#include "private/HeightmapDetail.h"

#include <ohm/MapInfo.h>
#include <ohm/OccupancyMap.h>

namespace
{
struct RegisterExtensionCodes
{
  RegisterExtensionCodes()
  {
    ohm::registerSerialiseExtensionErrorCodeString(int(ohm::kSeHeightmapInfoMismatch), "heightmap info mismatch");
  }
};

const RegisterExtensionCodes s_register_heightmap_errors;
}  // namespace

namespace ohm
{
int load(const std::string &filename, Heightmap &heightmap, SerialiseProgress *progress, MapVersion *version_out)
{
  HeightmapDetail &detail = *heightmap.detail();

  int err = load(filename, *detail.heightmap, progress, version_out);
  if (err)
  {
    return err;
  }

  // TODO(KS): Set axis from map info.
  const MapInfo &info = detail.heightmap->mapInfo();
  if (!bool(info.get("heightmap")))
  {
    return kSeHeightmapInfoMismatch;
  }

  detail.fromMapInfo(info);

  return err;
}
}  // namespace ohm
