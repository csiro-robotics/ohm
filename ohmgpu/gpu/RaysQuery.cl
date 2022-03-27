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
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Debug switches for the compiling code to enable (or just uncomment here).
// Deliberately before incldues to configure those files.
//------------------------------------------------------------------------------

// Report regions we can't resolve via printf().
#define REPORT_MISSING_REGIONS

// Limit the number of cells we can traverse in the line traversal. This is a worst case limit.
//#define LIMIT_LINE_WALK_ITERATIONS
// Limit the number of times we try update a voxel value. Probably best to always have this enabled.
//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "gpu_ext.h"  // Must be first

#include "MapCoord.h"
#include "RayFlag.h"
#include "RaysQueryResult.h"

#include "LineWalkMarkers.cl"
#include "Regions.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

#define WALK_VISIT_VOXEL visitVoxelRayQuery
#define WALK_NAME        RayQuery

// User data for voxel visit callback.
typedef struct LineWalkData_t
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global atomic_float *occupancy;
  // Array of offsets for each regionKey into occupancy. These are byte offsets.
  __global ulonglong *occupancy_offsets;
  // Array of region keys for currently loaded regions.
  __global int3 *region_keys;
  // The region currently being traversed. Also used to reduce searching the region_keys and region_mem_offsets.
  int3 current_region;
  // Size of a region in voxels.
  int3 region_dimensions;
  /// Value threshold for occupied voxels.
  float occupied_threshold;
  // Number of regions in region_keys/region_mem_offsets.
  uint region_count;
  // Index of the @c current_region into region_keys and corresponding xxx_offsets arrays.
  uint current_region_index;
  /// Coefficient for calulating the unobserved_volume.
  float volume_coefficient;

  RaysQueryResult result;
} LineWalkData;

// Implement the voxel traversal function. We update the value of the voxel using atomic instructions.
__device__ bool visitVoxelRayQuery(const GpuKey *voxel_key, const GpuKey *start_key, const GpuKey *end_key,
                                   int voxel_marker, float enter_range, float exit_range, const int *stepped,
                                   void *user_data)
{
  LineWalkData *line_data = (LineWalkData *)user_data;
  __global atomic_float *occupancy = line_data->occupancy;

  // Resolve memory offset for the region of interest.
  bool have_voxel_memory = regionsResolveRegion(voxel_key, &line_data->current_region, &line_data->current_region_index,
                                                line_data->region_keys, line_data->region_count);

#ifdef REPORT_MISSING_REGIONS
  if (!have_voxel_memory)
  {
    // We can fail to resolve regions along the in the line. This can occurs for several reasons:
    // - Floating point error differences between CPU and GPU line walking means that the GPU may walk into the edge
    //   of a region not hit when walking the regions on CPU.
    // - Regions may not be uploaded due to extents limiting on CPU.
    printf("%u region missing: " KEY_F "\n  Voxels: " KEY_F "->" KEY_F "\n", get_global_id(0), KEY_A(*voxel_key),
           KEY_A(*start_key), KEY_A(*end_key));
  }
#endif  // REPORT_MISSING_REGIONS

  // This voxel lies in the region. We will make a value adjustment.
  // Work out which voxel to modify.
  const ulonglong vi_local = voxel_key->voxel[0] + voxel_key->voxel[1] * line_data->region_dimensions.x +
                             voxel_key->voxel[2] * line_data->region_dimensions.x * line_data->region_dimensions.y;
  const ulonglong vi =
    (line_data->occupancy_offsets[line_data->current_region_index] / sizeof(*line_data->occupancy)) + vi_local;

  have_voxel_memory = have_voxel_memory && (voxel_key->voxel[0] < line_data->region_dimensions.x &&
                                            voxel_key->voxel[1] < line_data->region_dimensions.y &&
                                            voxel_key->voxel[2] < line_data->region_dimensions.z);

  __global atomic_float *occupancy_ptr = &occupancy[vi];

  // Lookup the voxel value.
  const float occupancy_value = (have_voxel_memory) ? gputilAtomicLoadF32(occupancy_ptr) : INFINITY;
  const int voxel_type = (occupancy_value == INFINITY) ?
                           RQ_OccUnobserved :
                           ((occupancy_value >= line_data->occupied_threshold) ? RQ_OccOccupied : RQ_OccFree);

  line_data->result.range = (voxel_type != RQ_OccOccupied) ? exit_range : line_data->result.range;
  line_data->result.unobserved_volume += (voxel_type == RQ_OccUnobserved) ?
                                           (line_data->volume_coefficient * (exit_range * exit_range * exit_range -
                                                                             enter_range * enter_range * enter_range)) :
                                           0.0f;
  line_data->result.voxel_type = voxel_type;

  // Stop traversal if occupied.
  return voxel_type != RQ_OccOccupied;
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
/// extracts it's start/end pair and performs a line walking algorithm from start to end key. The lines start end points
/// are also provided, relative to the centre of the starting voxel. These start/end coordinate pairs are in
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
/// be changed per line, by setting the value of @p GpuKey::voxel[3] (normally unused) to 1. This indicates the line has
/// been clipped.
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
/// @param line_count number of lines in @p line_keys and @p local_lines. These come in pairs, so the number of elements
///     in those arrays is double this value.
/// @param region_dimensions Specifies the size of any one region in voxels.
/// @param voxel_resolution Specifies the size of a voxel cube.
/// @param ray_adjustment Specifies the value adjustment to apply to voxels along the line segment leading up to the
///     final sample voxel. This should be < 0 to re-enforce as free.
/// @param sample_adjustment Specifiest the value adjustment applied to voxels containing the sample point (line end
///     point). Should be > 0 to re-enforce as occupied.
/// @param voxel_value_min Minimum clamping value for voxel adjustments.
/// @param voxel_value_max Maximum clamping value for voxel adjustments.
/// @param region_update_flags Update control values as per @c RayFlag.
__kernel void raysQuery(__global atomic_float *occupancy, __global ulonglong *occupancy_region_mem_offsets_global,
                        __global int3 *occupancy_region_keys_global, uint region_count, __global GpuKey *line_keys,
                        __global float3 *local_lines, uint line_count, int3 region_dimensions, float voxel_resolution,
                        float occupied_threshold, float volume_coefficient,
                        // Output buffers.
                        __global RaysQueryResult *results)
{
  // Only process valid lines.
  if (get_global_id(0) >= line_count)
  {
    return;
  }

  LineWalkData line_data;
  line_data.occupancy = occupancy;
  line_data.occupancy_offsets = occupancy_region_mem_offsets_global;
  line_data.region_keys = occupancy_region_keys_global;
  line_data.region_dimensions = region_dimensions;
  line_data.occupied_threshold = occupied_threshold;
  line_data.region_count = region_count;
  line_data.volume_coefficient = volume_coefficient;
  line_data.result.range = 0;
  line_data.result.unobserved_volume = 0;
  line_data.result.voxel_type = RQ_OccNull;

  regionsInitCurrent(&line_data.current_region, &line_data.current_region_index);

  // Now walk the clipped ray.
  GpuKey start_key, end_key;
  copyKey(&start_key, &line_keys[get_global_id(0) * 2 + 0]);
  copyKey(&end_key, &line_keys[get_global_id(0) * 2 + 1]);

  const float3 lineStart = local_lines[get_global_id(0) * 2 + 0];
  const float3 lineEnd = local_lines[get_global_id(0) * 2 + 1];

  walkVoxelsRayQuery(&start_key, &end_key, lineStart, lineEnd, region_dimensions, voxel_resolution, kLineWalkFlagNone,
                     &line_data);

  results[get_global_id(0)] = line_data.result;
}

#ifndef RAY_QUERY_BASE_CL
#define RAY_QUERY_BASE_CL
#endif  // RAY_QUERY_BASE_CL
