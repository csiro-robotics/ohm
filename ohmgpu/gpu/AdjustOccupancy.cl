// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef ADJUSTOCCUPANCY_CL
#define ADJUSTOCCUPANCY_CL

inline __device__ float calculateOccupancyAdjustment(const GpuKey *voxel_key, const GpuKey *end_key, bool is_end_voxel,
                                                     bool is_sample_voxel, float voxel_resolution,
                                                     LineWalkData *line_data)
{
  // We need to handle broken ray segments. They will come through with is_end_voxel true, but is_sample_voxel false.
  // We need to make a zero adjustment in that case.
  const float adjustment = (!is_sample_voxel || line_data->region_update_flags & kRfEndPointAsFree) ?
                             line_data->ray_adjustment :
                             line_data->sample_adjustment;
  return (is_end_voxel && !is_sample_voxel) ? 0 : adjustment;
}

#endif  // ADJUSTOCCUPANCY_CL
