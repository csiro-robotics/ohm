
float4 slerp(float4 from, float4 to, float interpolation_factor);
float3 quaternion_rotate(float4 rotation, float3 point);


float4 slerp(float4 from, float4 to, float interpolation_factor)
{
  if (all(isequal(from, to)))
  {
    return from;
  }

  float coeff0, coeff1, angle, sin_angle, cos_angle, inv_sin;

  cos_angle = dot(from, to);

  float4 temp = (cos_angle >= 0) ? to : -1.0f * to;
  cos_angle = (cos_angle >= 0) ? cos_angle : -1.0f * cos_angle;

  // numerical round-off error could create problems in call to acos
  if (1.0f - cos_angle > 1e-12f)
  {
    angle = acos(cos_angle);
    sin_angle = sin(angle);  // fSin >= 0 since fCos >= 0

    inv_sin = 1.0f / sin_angle;
    coeff0 = sin((1.0f - interpolation_factor) * angle) * inv_sin;
    coeff1 = sin(interpolation_factor * angle) * inv_sin;
  }
  else
  {
    coeff0 = 1.0f - interpolation_factor;
    coeff1 = interpolation_factor;
  }

  temp.x = coeff0 * from.x + coeff1 * temp.x;
  temp.y = coeff0 * from.y + coeff1 * temp.y;
  temp.z = coeff0 * from.z + coeff1 * temp.z;
  temp.w = coeff0 * from.w + coeff1 * temp.w;

  return temp;
}


float3 quaternion_rotate(float4 rotation, float3 v)
{
  const float xx = rotation.x * rotation.x;
  const float xy = rotation.x * rotation.y;
  const float xz = rotation.x * rotation.z;
  const float xw = rotation.x * rotation.w;
  const float yy = rotation.y * rotation.y;
  const float yz = rotation.y * rotation.z;
  const float yw = rotation.y * rotation.w;
  const float zz = rotation.z * rotation.z;
  const float zw = rotation.z * rotation.w;

  float3 res;

  res.x = (1 - 2 * (yy + zz)) * v.x +
          (2 * (xy - zw)) * v.y +
          (2 * (xz + yw)) * v.z;

  res.y = (2 * (xy + zw)) * v.x +
          (1 - 2 * (xx + zz)) * v.y +
          (2 * (yz - xw)) * v.z;

  res.z = (2 * (xz - yw)) * v.x +
          (2 * (yz + xw)) * v.y +
          (1 - 2 * (xx + yy)) * v.z;

  return res;
}


__kernel void transformTimestampedPoints(__global float3 *points, unsigned point_count,
                                         __global float *transform_timestamps,
                                         __global float3 *transform_positions,
                                         __global float4 *transform_rotations,
                                         unsigned transform_count)
{
  // // Load transform timestamps into local memory.
  // // This is the only data accessed multiple times.
  // const unsigned transform_step = min(transform_count / get_global_size(0), 1);
  // for (unsigned i = 0; i < transform_step; ++i)
  // {
  //   const unsigned transform_index = get_global_id(0) + i * get_global_size(0);
  //   if (transform_index < transform_count)
  //   {
  //     local_transform_times[transform_index] = timestamps[transform_index];
  //   }
  // }

  // barrier(CLK_LOCAL_MEM_FENCE);

  // Process points.
  if (get_global_id(0) >= point_count)
  {
    // Out of range.
    return;
  }

  float3 sample_point = points[get_global_id(0) * 2 + 1];
  const float sample_time = points[get_global_id(0) * 2 + 0].x;

  // Find the appropriate transforms. Binary search the transforms.
  unsigned from_index = 0;
  unsigned to_index = transform_count - 1;

  if (transform_count > 2)
  {
    // Binary search.
    while (from_index <= to_index)
    {
      const unsigned mid_low = (from_index + to_index) / 2;
      const unsigned mid_high = min(mid_low + 1, transform_count - 1);
      // Adapted binary search for the index braketing sample_time.
      if (sample_time >= transform_timestamps[mid_low] && sample_time <= transform_timestamps[mid_high])
      {
        from_index = mid_low;
        to_index = mid_high;
        break;
      }
      else if (sample_time <= transform_timestamps[mid_low])
      {
        to_index = mid_low - 1;
      }
      else
      {
        from_index = mid_low + 1;
      }
    }
  }

  // Have resolved the transform. Linearly interpoloate position and spherically rotation.
  const float interpolation_factor = (sample_time - transform_timestamps[from_index]) /
                                     (transform_timestamps[to_index] - transform_timestamps[from_index]);
  const float3 sensor_position = transform_positions[from_index] +
                                 interpolation_factor * (transform_positions[to_index] - transform_positions[from_index]);
  const float4 sensor_rotation = slerp(transform_rotations[from_index], transform_rotations[to_index], interpolation_factor);

  // Rotate and translate the local sample.
  sample_point = sensor_position + quaternion_rotate(sensor_rotation, sample_point);

  // Record the results.
  points[get_global_id(0) * 2 + 0] = sensor_position;
  points[get_global_id(0) * 2 + 1] = sample_point;
}
