// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef ADJUSTOCCUPANCY_CL
#define ADJUSTOCCUPANCY_CL

inline __device__ float calculateOccupancyAdjustment(const GpuKey *voxelKey, bool isEndVoxel, bool isSampleVoxel,
                                                     const GpuKey *startKey, const GpuKey *endKey,
                                                     float voxel_resolution, LineWalkData *line_data)
{
  // We need to handle broken ray segments. They will come through with isEndVoxel true, but isSampleVoxel false.
  // We need to make a zero adjustment in that case.
  const float adjustment = (!isSampleVoxel || line_data->region_update_flags & kRfEndPointAsFree) ?
                             line_data->ray_adjustment :
                             line_data->sample_adjustment;
  return (isEndVoxel && !isSampleVoxel) ? 0 : adjustment;
}

#endif  // ADJUSTOCCUPANCY_CL
