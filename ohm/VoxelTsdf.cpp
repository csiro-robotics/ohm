// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelTsdf.h"

#include "MapInfo.h"

namespace ohm
{
void updateMapInfo(MapInfo &info, const TsdfOptions &tsdf_options)
{
  info.set(MapValue("tsdf-max-weight", tsdf_options.max_weight));
  info.set(MapValue("tsdf-default-truncation-distance", tsdf_options.default_truncation_distance));
  info.set(MapValue("tsdf-dropoff-epsilon", tsdf_options.dropoff_epsilon));
  info.set(MapValue("tsdf-sparsity-compensation-factor", tsdf_options.sparsity_compensation_factor));
}


void fromMapInfo(TsdfOptions &tsdf_options, const MapInfo &info)
{
  tsdf_options.max_weight =
    static_cast<float>(info.get("tsdf-max-weight", MapValue("tsdf-max-weight", tsdf_options.max_weight)));
  tsdf_options.default_truncation_distance = static_cast<float>(
    info.get("tsdf-default-truncation-distance",
             MapValue("tsdf-default-truncation-distance", tsdf_options.default_truncation_distance)));
  tsdf_options.dropoff_epsilon = static_cast<float>(
    info.get("tsdf-dropoff-epsilon", MapValue("tsdf-dropoff-epsilon", tsdf_options.dropoff_epsilon)));
  tsdf_options.sparsity_compensation_factor = static_cast<float>(
    info.get("tsdf-sparsity-compensation-factor",
             MapValue("tsdf-sparsity-compensation-factor", tsdf_options.sparsity_compensation_factor)));
}
}  // namespace ohm
