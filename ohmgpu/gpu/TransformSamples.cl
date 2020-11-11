// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpu_ext.h"  // Must be first

__device__ float4 slerp(float4 from, float4 to, float interpolation_factor);
__device__ float4 quaternion_rotate_quaterion(float4 a, float4 b);
__device__ float3 quaternion_rotate_point(float4 rotation, float3 point);


__device__ float4 slerp(float4 from, float4 to, float interpolation_factor)
{
  // bool b = all(isequal(from, to));
  const bool all_equal = all(isequal(from, to));
  // if (all(isequal(from, to)))
  if (all_equal)
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


__device__ float4 quaternion_rotate_quaterion(float4 a, float4 b)
{
  float4 q;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  return q;
}


__device__ float3 quaternion_rotate_point(float4 rotation, float3 v)
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

  res.x = (1 - 2 * (yy + zz)) * v.x + (2 * (xy - zw)) * v.y + (2 * (xz + yw)) * v.z;

  res.y = (2 * (xy + zw)) * v.x + (1 - 2 * (xx + zz)) * v.y + (2 * (yz - xw)) * v.z;

  res.z = (2 * (xz - yw)) * v.x + (2 * (yz + xw)) * v.y + (1 - 2 * (xx + yy)) * v.z;

  return res;
}


__kernel void transformTimestampedPoints(__global float3 *points, uint point_count,
                                         __global float *transform_timestamps, __global float3 *transform_positions,
                                         __global float4 *transform_rotations, uint transform_count, uint batch_size)
{
  // // Load transform timestamps into local memory.
  // // This is the only data accessed multiple times.
  // const uint transform_step = min(transform_count / get_global_size(0), 1);
  // for (uint i = 0; i < transform_step; ++i)
  // {
  //   const uint transform_index = get_global_id(0) + i * get_global_size(0);
  //   if (transform_index < transform_count)
  //   {
  //     local_transform_times[transform_index] = timestamps[transform_index];
  //   }
  // }

  // barrier(CLK_LOCAL_MEM_FENCE);

  // printf("hi %u / %u\n", get_global_id(0), get_global_size(0));

  for (uint i = 0; i < batch_size; ++i)
  {
    const unsigned sample_index = get_global_id(0) * batch_size + i;
    if (sample_index >= point_count)
    {
      // Out of range.
      // printf("Should return %u : %u\n", get_global_id(0), sample_index);
      return;
    }

    // printf("Thread %u processing %u\n", get_global_id(0), sample_index);

    float3 sample_point = points[sample_index * 2 + 1];
    float sample_time = points[sample_index * 2 + 0].x;

    // if (isGlobalThread(0, 0, 0))
    // {
    //   printf("%u / %u : %f %f %f\n", sample_index, point_count, sample_point.x, sample_point.y, sample_point.z);
    // }

    // Find the appropriate transforms. Binary search the transforms.
    uint from_index = 0;
    uint to_index = transform_count - 1;

    if (transform_count > 2)
    {
      // Binary search.
      const uint iter_limit = 100000;
      uint iter_count = 0;

      if (transform_timestamps[0] <= sample_time && sample_time <= transform_timestamps[transform_count - 1])
      {
        while (from_index <= to_index && iter_count < iter_limit)
        {
          ++iter_count;
          const uint mid_low = (from_index + to_index) / 2;
          const uint mid_high = min(mid_low + 1, transform_count - 1);
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

#ifdef DEBUG
        if (iter_count >= iter_limit)
        {
          printf("transformTimestampedPoints(): Binary search failure (%u): %u / %u. search-bound(%u, %u), max(%u)",
                 get_global_id(0), iter_count, iter_limit, from_index, to_index, transform_count);
          printf("Search Time: %f\n", sample_time);
          printf("Times[%u]:\n", transform_count);
          for (uint i = 0; i < transform_count; ++i)
          {
            printf("  %f\n", transform_timestamps[i]);
          }
        }
#endif  // DEBUG
      }
      else
      {
#if DEBUG
        printf("transformTimestampedPoints()[%u]: out of range %f: [%f, %f]\n", get_global_id(0), sample_time,
               transform_timestamps[0], transform_timestamps[transform_count - 1]);
#endif  // DEBUG
        if (sample_time < transform_timestamps[0])
        {
          sample_time = transform_timestamps[0];
          from_index = to_index = 0;
        }
        else
        {
          sample_time = transform_timestamps[transform_count - 1];
          from_index = to_index = transform_count - 1;
        }
      }
    }

    // Have resolved the transform. Linearly interpoloate position and spherically rotation.
    const float interpolation_factor = (sample_time - transform_timestamps[from_index]) /
                                       (transform_timestamps[to_index] - transform_timestamps[from_index]);
    const float3 sensor_position =
      transform_positions[from_index] +
      interpolation_factor * (transform_positions[to_index] - transform_positions[from_index]);
    const float4 sensor_rotation = quaternion_rotate_quaterion(
      transform_rotations[from_index],
      slerp(transform_rotations[from_index], transform_rotations[to_index], interpolation_factor));

    // printf("GPU: %f(%f)  T(%f %f %f) R(%f %f %f %f)\n", sample_time, interpolation_factor, sensor_position.x,
    // sensor_position.y,
    //         sensor_position.z, sensor_rotation.w, sensor_rotation.x, sensor_rotation.y, sensor_rotation.z);

    // Rotate and translate the local sample.
    sample_point = sensor_position + quaternion_rotate_point(sensor_rotation, sample_point);

    // Record the results.
    points[sample_index * 2 + 0] = sensor_position;
    points[sample_index * 2 + 1] = sample_point;
  }
}
