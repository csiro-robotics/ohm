// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpu_ext.h"  // Must be first

#include "GpuKey.h"
#include "MapCoord.h"

#include "LineWalkMarkers.cl"

#define WALK_NAME        LineKeys
#define WALK_VISIT_VOXEL lineKeysVisitVoxel

__device__ bool lineKeysVisitVoxel(const GpuKey *voxel_key, const GpuKey *start_key, const GpuKey *end_key,
                                   int voxel_marker, float enter_range, float exit_range, const int *stepped,
                                   void *user_data);

// Must be included after above defined
#include "LineWalk.cl"

typedef struct LineWalkData_t
{
  __global GpuKey *line_out;
  uint max_keys;
  uint key_count;
} LineWalkData;

__device__ void calculateLineKeys(__global GpuKey *line_out, uint max_keys, const GpuKey *start_key,
                                  const GpuKey *end_key, const float3 start_point, const float3 end_point,
                                  const int3 region_dim, float voxel_resolution);


__device__ bool lineKeysVisitVoxel(const GpuKey *voxel_key, const GpuKey *start_key, const GpuKey *end_key,
                                   int voxel_marker, float enter_range, float exit_range, const int *stepped,
                                   void *user_data)
{
  LineWalkData *line_data = (LineWalkData *)user_data;
  // Dereferencing line_data->line_out[1 + line_data->key_count++] caused issues on Intel OpenCL. Defer the increment
  // and use pointer arithmetic as a workaround.
  copyKey(line_data->line_out + line_data->key_count + 1, voxel_key);
  // Increment both as a workaround.
  ++line_data->key_count;
  return line_data->key_count + 1 < line_data->max_keys;
}


__device__ void calculateLineKeys(__global GpuKey *line_out, uint max_keys, const GpuKey *start_key,
                                  const GpuKey *end_key, const float3 start_point, const float3 end_point,
                                  const int3 region_dim, float voxel_resolution)
{
  LineWalkData line_data;
  line_data.line_out = line_out;  // + 1;
  line_data.max_keys = max_keys;
  line_data.key_count = 0;

  walkVoxelsLineKeys(start_key, end_key, start_point, end_point, region_dim, voxel_resolution, kLineWalkFlagNone,
                     &line_data);

  // Write result count to the first entry.
  line_out[0].region[0] = line_out[0].region[1] = line_out[0].region[2] = line_data.key_count;
  // printf("%lu => %u keys\n", get_global_id(0), line_data.key_count);
}


__kernel void calculateLines(__global GpuKey *lines_out, uint max_keys_per_line,
                             const __global float3 *query_point_pairs, uint query_count, int3 region_dim,
                             float voxel_resolution)
{
  const bool valid_thread = (get_global_id(0) < query_count);

  if (!valid_thread)
  {
    return;
  }

  float3 start_point = (valid_thread) ? query_point_pairs[get_global_id(0) * 2 + 0] : make_float3(0, 0, 0);
  float3 end_point = (valid_thread) ? query_point_pairs[get_global_id(0) * 2 + 1] : make_float3(0, 0, 0);
  __global GpuKey *line_out = lines_out + (get_global_id(0) * max_keys_per_line);
  // We convert region_dim from an int3 to an array to allow indexed access.
  GpuKey start_key, end_key;

  coordToKey(&start_key, &start_point, &region_dim, voxel_resolution);
  coordToKey(&end_key, &end_point, &region_dim, voxel_resolution);

  // printf("Query Count: %u\nValid? %d\n", query_count, valid_thread ? 1 : 0);
  // printf("RD: %d %d %d   Res: %f\n", region_dim.x, region_dim.y, region_dim.z, voxel_resolution);
  // printf("From %f %f %f to %f %f %f\n", start_point.x, start_point.y, start_point.z, end_point.x, end_point.y,
  // end_point.z); printf("Keys: " KEY_F " to " KEY_F "\n", KEY_A(start_key), KEY_A(end_key));

  // Adjust start/end point to be relative to the centre of the start_key voxel.
  start_point -= voxelCentre(&start_key, &region_dim, voxel_resolution);
  end_point -= voxelCentre(&start_key, &region_dim, voxel_resolution);

  calculateLineKeys(line_out, max_keys_per_line, &start_key, &end_key, start_point, end_point, region_dim,
                    voxel_resolution);
}
