
inline __device__ float calculateAdjustment(bool isEndVoxel, struct LineWalkData *line_data)
{
  if (!isEndVoxel)
  {
  const float adjustment = (!isEndVoxel || line_data->region_update_flags & kRfEndPointAsFree) ?
            line_data->ray_adjustment : line_data->sample_adjustment;
  }

  // NDT should do sample update in a separate process in order to update the covariance, so we should not get here.
  return line_data->sample_adjustment;
}
