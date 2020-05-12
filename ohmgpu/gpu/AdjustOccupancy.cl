
inline __device__ float calculateAdjustment(bool isEndVoxel, struct LineWalkData *line_data)
{
  const float adjustment = (!isEndVoxel || line_data->region_update_flags & kRfEndPointAsFree) ?
            line_data->ray_adjustment : line_data->sample_adjustment;
}
