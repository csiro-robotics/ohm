// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

//------------------------------------------------------------------------------
// Note on building this code.
// This code is intended to be used for updating regions both with and without
// sub voxel positioning. This is mostly the same, but the voxel type changes
// and we need an extract function call when updating a voxel with sub voxel
// positioning. To achive this, this code is intended to be includes into a
// compiled twice with the following defines changed on each compilation:
// - REGION_UPDATE_KERNEL : the kernel name for the entry point
// - REGION_UPDATE_SUFFIX : suffix applied to distinguish potentially repeaded
//  code and types.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Debug switches for the compiling code to enable (or just uncomment here).
// Deliberately before incldues to configure those files.
//------------------------------------------------------------------------------

// Report regions we can't resolve via printf().
// #define REPORT_MISSING_REGIONS

// Limit the number of cells we can traverse in the line traversal. This is a worst case limit.
//#define LIMIT_LINE_WALK_ITERATIONS
// Limit the number of times we try update a voxel value. Probably best to always have this enabled.
#ifndef LIMIT_VOXEL_WRITE_ITERATIONS
#define LIMIT_VOXEL_WRITE_ITERATIONS
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS


//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#define GPUTIL_ATOMICS_64 1
#include "gpu_ext.h"  // Must be first

#include "MapCoord.h"
#include "RayFlag.h"
#include "VoxelTsdfCompute.h"

#include "LineWalkMarkers.cl"
#include "Regions.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

#define WALK_VISIT_VOXEL visitVoxelTsdf
#define WALK_NAME        Tsdf

#ifndef TSDF_UPDATE_BASE_CL
// User data for voxel visit callback.
typedef struct TsdfWalkData_t
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global VoxelTsdf *tsdf_voxels;
  // Array of offsets for each regionKey into tsdf. These are byte offsets.
  __global ulonglong *tsdf_offsets;
  // Array of region keys for currently loaded regions.
  __global int3 *region_keys;
  /// The voxel size.
  float voxel_resolution;
  /// Maximum TSDF voxel weight.
  float max_weight;
  /// Default TSDF truncation distance
  float default_truncation_distance;
  /// Non-zero/+ve to enable voxel dropoff. Recommended to set to the voxel size.
  float dropoff_epsilon;
  /// Non-zero/+ve to enable sparsity compensation.
  float sparsity_compensation_factor;
  // The region currently being traversed. Also used to reduce searching the region_keys and region_mem_offsets.
  int3 current_region;
  // Size of a region in voxels.
  int3 region_dimensions;
  // Number of regions in region_keys/region_mem_offsets.
  uint region_count;
  // Index of the @c current_region into region_keys and corresponding xxx_offsets arrays.
  uint current_region_index;
  uint region_update_flags;
  /// A reference sensor position. This is in a frame local to the centre of the voxel containing the sample coordinate.
  float3 sensor;
  /// A reference sample position. This is in a frame local to the centre of the voxel containing this sample
  /// coordinate. Note: for TSDF this is not always the same as the voxel identified by @c end_key in the visit
  /// function. Ray truncation and filtering may modify the @pc end_key so we only process part of the ray.
  float3 sample;
} TsdfWalkData;
#endif  // TSDF_UPDATE_BASE_CL

// A somewhat tacked on implementation of Truncated Signed Distance Fields.
// Implement the voxel traversal function. We update the value of the voxel using atomic instructions.
//
// Note: TSDF ray tracing is actually done in reverse. This can greatly reduce voxel contension improving TSDF
// performance (as the CAS loop limit is hit less often) and quality (as be abandon data less often).
__device__ bool visitVoxelTsdf(const GpuKey *voxel_key, const GpuKey *start_key, const GpuKey *end_key,
                               int voxel_marker, float enter_range, float exit_range, const int *stepped,
                               void *user_data)
{
  TsdfWalkData *tsdf_data = (TsdfWalkData *)user_data;

  // Resolve memory offset for the region of interest.
  if (!regionsResolveRegion(voxel_key, &tsdf_data->current_region, &tsdf_data->current_region_index,
                            tsdf_data->region_keys, tsdf_data->region_count))
  {
    // We can fail to resolve regions along the in the line. This can occurs for several reasons:
    // - Floating point error differences between CPU and GPU line walking means that the GPU may walk into the edge
    //   of a region not hit when walking the regions on CPU.
    // - Regions may not be uploaded due to extents limiting on CPU.
#ifdef REPORT_MISSING_REGIONS
    printf("%u region missing: " KEY_F "\n", get_global_id(0), KEY_A(*voxel_key));
#endif
    return true;
  }

  // We assume this voxel lies in the tsdf_data->current_region.
  // Work out which voxel to modify.
  const ulonglong vi_local = voxel_key->voxel[0] + voxel_key->voxel[1] * tsdf_data->region_dimensions.x +
                             voxel_key->voxel[2] * tsdf_data->region_dimensions.x * tsdf_data->region_dimensions.y;
  ulonglong vi =
    (tsdf_data->tsdf_offsets[tsdf_data->current_region_index] / sizeof(*tsdf_data->tsdf_voxels)) + vi_local;

  if (voxel_key->voxel[0] < tsdf_data->region_dimensions.x && voxel_key->voxel[1] < tsdf_data->region_dimensions.y &&
      voxel_key->voxel[2] < tsdf_data->region_dimensions.z)
  {
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
    // Under high contension we can end up repeatedly failing to write the voxel value.
    // The primary concern is not deadlocking the GPU, so we put a hard limit on the numebr of
    // attempts made.
    const int iteration_limit = 20;
    int iterations = 0;
#endif

    // Calculate the current voxel centre in the same space as tsdf_data->sensor and tsdf_data->sample. Remember,
    // those values are both calculated relative to the centre of the voxel containing tsdf_data->sample
    const bool reverse_walk = tsdf_data->region_update_flags & kRfReverseWalk;
    const int3 voxel_diff = keyDiff(voxel_key, (!reverse_walk) ? end_key : start_key, tsdf_data->region_dimensions);
    const float3 voxel_centre =
      make_float3(voxel_diff.x * tsdf_data->voxel_resolution, voxel_diff.y * tsdf_data->voxel_resolution,
                  voxel_diff.z * tsdf_data->voxel_resolution);

    /// Use a union of VoxelTsdf and atomic_ulong (64-bits) so we can write the value back in one operation.
    union
    {
      VoxelTsdf voxel;
      ulonglong value;
    } initial, updated_tsdf;

    __global atomic_ulong *tsdf_voxel_ptr = (__global atomic_ulong *)&tsdf_data->tsdf_voxels[vi];

    do
    {
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
      if (iterations++ > iteration_limit)
      {
        break;
      }
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS

      initial.value = gputilAtomicLoadU64(tsdf_voxel_ptr);
      updated_tsdf.voxel.weight = initial.voxel.weight;
      updated_tsdf.voxel.distance = initial.voxel.distance;

      if (!calculateTsdf(tsdf_data->sensor, tsdf_data->sample, voxel_centre, tsdf_data->default_truncation_distance,
                         tsdf_data->max_weight, tsdf_data->dropoff_epsilon, tsdf_data->sparsity_compensation_factor,
                         &updated_tsdf.voxel.weight, &updated_tsdf.voxel.distance))
      {
        // Weight too low. Nothing more to do for this voxel.
        return true;
      }
      // Now try write the value, looping if we fail to write the new value.
      // mem_fence(CLK_GLOBAL_MEM_FENCE);
    } while (
      (updated_tsdf.voxel.distance != initial.voxel.distance || updated_tsdf.voxel.weight != initial.voxel.weight) &&
      !gputilAtomicCasU64(tsdf_voxel_ptr, initial.value, updated_tsdf.value));
  }

  return true;
}

// Must be included after WALK_NAME and WALK_VISIT_VOXEL and the WALK_VISIT_VOXEL function is defined
#include "LineWalk.cl"

//------------------------------------------------------------------------------
// Kernel
//------------------------------------------------------------------------------

/// Integrate rays into voxel map regions.
///
/// Invoked one thread per ray (per @p line_keys pair).
///
/// Like keys are provided in start/end key pairs in @p line_keys where there are @p line_count pairs. Each thread
/// extracts it's start/end pair and performs a line walking algorithm from start to end key. The lines start end
/// points are also provided, relative to the centre of the starting voxel. These start/end coordinate pairs are in
/// @p local_lines. The coordinates for each line are local to the starting voxel centre in order to avoid precision
/// issues which may be introduced in converting from a common double precision frame on CPU into a single precision
/// frame in GPU (we do not support double precision GPU due to the limited driver support).
///
/// For each voxel key along the line, we resolve a voxel in @p occupancy by cross referencing in
/// @p occupancy_region_keys_global, @p occupancy_region_mem_offsets_global and @p region_count. Voxels are split into
/// regions in contiguous chunks in @p occupancy. The @c GpuKey::region for a voxel is matched in lookup @p
/// occupancy_region_keys_global and the index into @p occupancy_region_keys_global recorded. This index is used to
/// lookup @p occupancy_region_mem_offsets_global, which provides a byte offset (not elements) from @p occupancy at
/// which the voxel memory for this voxel begins. Each voxel region has a number of voxels equal to
/// <tt>region_dimensions.x * region_dimensions.y * region_dimensions.z</tt>.
///
/// Once voxel memory is resolved, the value of that voxel is updated by either adding @p ray_adjustment for all but
/// the last voxel in the line, or @p sample_adjustment for the last voxel (exception listed below). The value is
/// clamped to the range <tt>[voxel_value_min, voxel_value_max]</tt>. This adjustment is made in global memory using
/// atomic operations. Success is not guaranteed, but is highly probably. This contension has performance impacts, but
/// was found to be the best overall approach for performance.
///
/// The value adjustment of @p sample_adjustment is normally used for the last voxel in each line. This behaviour may
/// be changed per line, by setting the value of @p GpuKey::voxel[3] (normally unused) to 1. This indicates the line
/// has been clipped.
///
/// The line traversal is also affected by the region_update_flags, which are defined in the enum RayFlag.
/// These modify the line traversal as follows:
/// - kRfEndPointAsFree: use ray_adjustment for the last voxel rather than sample_adjustment.
/// - kRfStopOnFirstOccupied: terminate line walking after touching the first occupied voxel found.
/// - kRfExcludeSample: ignore the voxel containing the sample (the last voxel).
/// - kRfExcludeUnobserved: do not adjust voxels which are (initially) unobserved
/// - kRfExcludeFree: do not adjust voxels which are (initially) free
/// - kRfExcludeOccupied: do not adjust voxels which are (initially) occupied
///
/// @param occupancy Pointer to the dense voxel occupancy maps the currently available regions. Offsets for a
///     specific region are available by looking up a region key in @p occupancy_region_keys_global and using the
///     cooresponding @c occupancy_region_mem_offsets_global byte offset into this array.
/// @param occupancy_region_keys_global Array of voxel region keys identifying regions available in GPU. There are
///     @c region_count elements in this array.
/// @param occupancy_region_mem_offsets_global Array of voxel region memory offsets into @c occupancy. Each element
///     corresponds to a key in occupancy_region_keys_global. The offsets are in bytes.
/// @param line_keys Array of line segment pairs converted into @c GpuKey references to integrate into the map.
/// @param local_lines Array array of line segments which generated the @c line_keys. These are converted from the
///     original, double precision, map frame coordinates into a set of local frames. Each start/end point pair is
///     relative to the centre of the voxel containing the end point. This is to reduce floating point error in
///     double to single precision conversion and assist in voxel mean calculations which are in the same frame.
///     The original coordinates are not recoverable in this code.
/// @param unclipped_lines A companion array to @p local_lines which contains the original, unclipped sensor/sample
///     pairs with @p line_count such pairs. These are the original sensor/sample points used to generate
///     @p local_lines. They may differ from the items in @p local_lines because the lines may be truncated or the
///     original rays may be split into multiple segments (in CPU). We need the corresponding original points
///     to ensure the TSDF distance calculation is correct. Like @p local_lines, the @p samples are provided
///     relative to centre of each @p line_keys end voxels (second voxel key in each pair).
/// @param line_count number of lines in @p line_keys and @p local_lines. These come in pairs, so the number of
/// elements in those arrays is double this value.
/// @param region_dimensions Specifies the size of any one region in voxels.
/// @param voxel_resolution Specifies the size of a voxel cube.
/// @param ray_adjustment Specifies the value adjustment to apply to voxels along the line segment leading up to the
///     final sample voxel. This should be < 0 to re-enforce as free.
/// @param sample_adjustment Specifiest the value adjustment applied to voxels containing the sample point (line end
///     point). Should be > 0 to re-enforce as occupied.
/// @param voxel_value_min Minimum clamping value for voxel adjustments.
/// @param voxel_value_max Maximum clamping value for voxel adjustments.
/// @param region_update_flags Update control values as per @c RayFlag. Only respects @c kRfReverseWalk.
__kernel void tsdfRayUpdate(__global VoxelTsdf *tsdf_voxels, __global ulonglong *tsdf_region_mem_offsets_global,  //
                            __global int3 *tsdf_region_keys_global, uint region_count,                            //
                            __global GpuKey *line_keys, __global float3 *local_lines,                             //
                            __global float3 *unclipped_lines, uint line_count,                                    //
                            int3 region_dimensions, float voxel_resolution, float max_weight,
                            float default_truncation_distance, float dropoff_epsilon,
                            float sparsity_compensation_factor, uint region_update_flags)
{
  // Only process valid lines.
  if (get_global_id(0) >= line_count)
  {
    return;
  }

  TsdfWalkData tsdf_data;
  tsdf_data.tsdf_voxels = tsdf_voxels;
  tsdf_data.tsdf_offsets = tsdf_region_mem_offsets_global;
  tsdf_data.region_keys = tsdf_region_keys_global;
  tsdf_data.region_dimensions = region_dimensions;
  tsdf_data.voxel_resolution = voxel_resolution;
  tsdf_data.max_weight = max_weight;
  tsdf_data.default_truncation_distance = default_truncation_distance;
  tsdf_data.dropoff_epsilon = dropoff_epsilon;
  tsdf_data.sparsity_compensation_factor = sparsity_compensation_factor;
  tsdf_data.region_count = region_count;
  tsdf_data.region_update_flags = region_update_flags;

  regionsInitCurrent(&tsdf_data.current_region, &tsdf_data.current_region_index);

  // Now walk the clipped ray.
  GpuKey start_key, end_key;
  copyKey(&start_key, &line_keys[get_global_id(0) * 2 + 0]);
  copyKey(&end_key, &line_keys[get_global_id(0) * 2 + 1]);

  tsdf_data.sensor = unclipped_lines[get_global_id(0) * 2 + 0];
  tsdf_data.sample = unclipped_lines[get_global_id(0) * 2 + 1];

  const float3 line_start = local_lines[get_global_id(0) * 2 + 0];
  const float3 line_end = local_lines[get_global_id(0) * 2 + 1];

  int walk_flags = 0;
  if (region_update_flags & kRfReverseWalk)
  {
    walk_flags |= kLineWalkFlagReverse;
  }

  walkVoxelsTsdf(&start_key, &end_key, line_start, line_end, region_dimensions, voxel_resolution, walk_flags,
                 &tsdf_data);
}

#ifndef TSDF_UPDATE_BASE_CL
#define TSDF_UPDATE_BASE_CL
#endif  // TSDF_UPDATE_BASE_CL
