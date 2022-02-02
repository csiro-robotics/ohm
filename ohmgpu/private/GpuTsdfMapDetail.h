// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUTSDFMAPDETAIL_H
#define GPUTSDFMAPDETAIL_H

#include "OhmGpuConfig.h"

#include "private/GpuMapDetail.h"

#include <ohm/VoxelTsdf.h>

namespace ohm
{
struct GpuTsdfMapDetail : public GpuMapDetail
{
  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the tsdf layer.
  int tsdf_uidx = -1;
  /// Tsdf mapping options.
  TsdfOptions tsdf_options;

  GpuTsdfMapDetail(OccupancyMap *map, bool borrowed_map)
    : GpuMapDetail(map, borrowed_map)
  {
    // Require original samples for TSDF for the distance calculations.
    use_original_ray_buffers = true;
  }
};
}  // namespace ohm

#endif  // GPUTSDFMAPDETAIL_H
