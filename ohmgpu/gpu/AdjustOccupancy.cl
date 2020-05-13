// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef ADJUSTOCCUPANCY_CL
#define ADJUSTOCCUPANCY_CL

inline __device__ float calculateOccupancyAdjustment(bool isEndVoxel, struct LineWalkData *line_data, float voxel_resolution)
{
  const float adjustment = (!isEndVoxel || line_data->region_update_flags & kRfEndPointAsFree) ?
            line_data->ray_adjustment : line_data->sample_adjustment;
  return adjustment;
}

#endif // ADJUSTNDT_CL
