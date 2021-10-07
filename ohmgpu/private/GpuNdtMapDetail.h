// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUNDTMAPDETAIL_H
#define GPUNDTMAPDETAIL_H

#include "OhmGpuConfig.h"

#include "private/GpuMapDetail.h"

#include <ohm/NdtMap.h>
#include <ohm/NdtMode.h>

namespace ohm
{
struct GpuNdtMapDetail : public GpuMapDetail
{
  GpuProgramRef *cov_hit_program_ref = nullptr;
  gputil::Kernel cov_hit_kernel;
  NdtMap ndt_map;

  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the covariance layer.
  int cov_uidx = -1;
  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the intensity layer.
  int intensity_uidx = -1;
  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the hit/hiss layer.
  int hit_miss_uidx = -1;

  GpuNdtMapDetail(OccupancyMap *map, bool borrowed_map, NdtMode mode)
    : GpuMapDetail(map, borrowed_map)
    , ndt_map(map, true, mode)  // Ensures correct initialisation for NDT operations.
  {}
};
}  // namespace ohm

#endif  // GPUNDTMAPDETAIL_H
