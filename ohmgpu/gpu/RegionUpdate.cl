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
//#define REPORT_MISSING_REGIONS

// Limit the number of cells we can traverse in the line traversal. This is a worst case limit.
//#define LIMIT_LINE_WALK_ITERATIONS
// Limit the number of times we try update a voxel value. Probably best to always have this enabled.
#ifndef LIMIT_VOXEL_WRITE_ITERATIONS
#define LIMIT_VOXEL_WRITE_ITERATIONS
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "gpu_ext.h"  // Must be first

#include "MapCoord.h"
#include "RayFlag.h"
#include "Traversal.cl"
#include "VoxelIncident.cl"
#include "VoxelMeanCompute.h"
#ifdef NDT
#include "CovarianceVoxelCompute.h"
#endif  // NDT

#include "LineWalkMarkers.cl"
#include "Regions.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

// This is begging for a refactor.
#ifdef NDT
#define REGION_UPDATE_KERNEL regionRayUpdateNdt
#define REGION_WALK_VOXELS   walkVoxelsNdt
#define REGION_VISIT_VOXEL   visitVoxelNdt
#define WALK_VISIT_VOXEL     visitVoxelNdt
#define WALK_NAME            Ndt

#else  // NDT
#define REGION_UPDATE_KERNEL regionRayUpdateOccupancy
#define REGION_WALK_VOXELS   walkVoxelsOccupancy
#define REGION_VISIT_VOXEL   visitVoxelOccupancy
#define WALK_VISIT_VOXEL     visitVoxelOccupancy
#define WALK_NAME            Occupancy
#endif  // NDT


#ifndef REGION_UPDATE_BASE_CL
// User data for voxel visit callback.
typedef struct LineWalkData_t
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global atomic_float *occupancy;
  // Array of offsets for each regionKey into occupancy. These are byte offsets.
  __global ulonglong *occupancy_offsets;
  __global VoxelMean *means;
  // Array of offsets for each regionKey into means. These are byte offsets.
  __global ulonglong *means_offsets;
  // Traversal voxel memory
  __global atomic_float *traversal;
  // Array of offsets for each regionKey into traversal. These are byte offsets.
  __global ulonglong *traversal_offsets;
  // Touch time voxel memory
  __global atomic_uint *touch_times;
  // Array of offsets for each regionKey into touch_times. These are byte offsets.
  __global ulonglong *touch_times_offsets;
  // Incidents voxel memory
  __global atomic_uint *incidents;
  // Array of offsets for each regionKey into incidents. These are byte offsets.
  __global ulonglong *incidents_offsets;
#ifdef NDT
  __global CovarianceVoxel *cov_voxels;
  // Array of offsets for each regionKey into cov_voxels. These are byte offsets.
  __global ulonglong *cov_offsets;
  // Number of hit/miss counts used in the ndt-tm model
  __global HitMissCount *hit_miss;
  // Array of offsets for each regionKey into hit_miss. These are byte offsets.
  __global ulonglong *hit_miss_offsets;
#endif  // NDT
  // Array of region keys for currently loaded regions.
  __global int3 *region_keys;
  // The region currently being traversed. Also used to reduce searching the region_keys and region_mem_offsets.
  int3 current_region;
  // Size of a region in voxels.
  int3 region_dimensions;
  /// Voxel size
  float voxel_resolution;
  // MapMode/voxel value adjustment for keys along the line segment, but not the sample voxel.
  float ray_adjustment;
  // MapMode/voxel value adjustment for the sample voxel.
  float sample_adjustment;
  /// Value threshold for occupied voxels.
  float occupied_threshold;
  // MapMode/voxel minimum allowed value.
  float voxel_value_min;
  // MapMode/voxel maximum allowed value.
  float voxel_value_max;
  // Number of regions in region_keys/region_mem_offsets.
  uint region_count;
  // Index of the @c current_region into region_keys and corresponding xxx_offsets arrays.
  uint current_region_index;
  uint region_update_flags;
  /// A reference sample position. This is in a frame local to the centre of the voxel containing this sample
  /// coordinate.
  float3 sample;
  /// A reference sensor position. This is in a frame local to the centre of the voxel containing the sample coordinate.
  float3 sensor;
  /// Encoded touch time value for this voxel.
  uint touch_time;
#ifdef NDT
  // Affects how quickly NDT removes voxels: [0, 1].
  float adaptation_rate;
  // An estimate on the sensor range noise error.
  float sensor_noise;
#endif  // NDT
} LineWalkData;
#endif  // REGION_UPDATE_BASE_CL

#include "VoxelMean.cl"

#ifdef NDT
#include "AdjustNdt.cl"
#else  // NDT
#include "AdjustOccupancy.cl"
#endif  // NDT

__device__ bool REGION_VISIT_VOXEL(const GpuKey *voxel_key, const GpuKey *start_key, const GpuKey *end_key,
                                   int voxel_marker, float enter_range, float exit_range, const int *stepped,
                                   void *user_data);

// Must be included after WALK_NAME and WALK_VISIT_VOXEL function is define
#include "LineWalk.cl"

// Implement the voxel traversal function. We update the value of the voxel using atomic instructions.
__device__ bool REGION_VISIT_VOXEL(const GpuKey *voxel_key, const GpuKey *start_key, const GpuKey *end_key,
                                   int voxel_marker, float enter_range, float exit_range, const int *stepped,
                                   void *user_data)
{
  float old_value, new_value;

  LineWalkData *line_data = (LineWalkData *)user_data;
  __global atomic_float *occupancy = line_data->occupancy;

  // Abort if this is the sample voxel and we are to exclude the sample. The sample voxel is detected when voxel_marker
  // is kLineWalkMarkerEnd is true and voxel[3] is zero. A value of 1 indicates a clipped ray and the end voxel does not
  // contain the sample.
  const bool reverse_walk = line_data->region_update_flags & kRfReverseWalk;
  const bool is_sample_candidate =
    (!reverse_walk && voxel_marker == kLineWalkMarkerEnd) || (reverse_walk && voxel_marker == kLineWalkMarkerStart);
  const bool is_sample_voxel = is_sample_candidate && voxel_key->voxel[3] == 0;
  if (is_sample_voxel && (line_data->region_update_flags & kRfExcludeSample))
  {
    return true;
  }

  if (!is_sample_voxel && (line_data->region_update_flags & kRfExcludeRay))
  {
    return true;
  }

  // Resolve memory offset for the region of interest.
  if (!regionsResolveRegion(voxel_key, &line_data->current_region, &line_data->current_region_index,
                            line_data->region_keys, line_data->region_count))
  {
    // We can fail to resolve regions along the in the line. This can occurs for several reasons:
    // - Floating point error differences between CPU and GPU line walking means that the GPU may walk into the edge
    //   of a region not hit when walking the regions on CPU.
    // - Regions may not be uploaded due to extents limiting on CPU.
#ifdef REPORT_MISSING_REGIONS
    printf("%u region missing: " KEY_F "\n  Voxels: " KEY_F "->" KEY_F "\n", get_global_id(0), KEY_A(*voxel_key),
           KEY_A(*start_key), KEY_A(*end_key));
#endif
    return true;
  }

  // Adjust value by ray_adjustment unless this is the sample voxel.
  float adjustment = calculateOccupancyAdjustment(voxel_key, (!reverse_walk) ? end_key : start_key, is_sample_candidate,
                                                  is_sample_voxel, line_data->voxel_resolution, line_data);

  // This voxel lies in the region. We will make a value adjustment.
  // Work out which voxel to modify.
  const ulonglong vi_local = voxel_key->voxel[0] + voxel_key->voxel[1] * line_data->region_dimensions.x +
                             voxel_key->voxel[2] * line_data->region_dimensions.x * line_data->region_dimensions.y;
  ulonglong vi =
    (line_data->occupancy_offsets[line_data->current_region_index] / sizeof(*line_data->occupancy)) + vi_local;

  if (voxel_key->voxel[0] < line_data->region_dimensions.x && voxel_key->voxel[1] < line_data->region_dimensions.y &&
      voxel_key->voxel[2] < line_data->region_dimensions.z)
  {
    __global atomic_float *occupancy_ptr = &occupancy[vi];

    bool was_occupied_voxel = false;

#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
    // Under high contension we can end up repeatedly failing to write the voxel value.
    // The primary concern is not deadlocking the GPU, so we put a hard limit on the numebr of
    // attempts made.
    const int iteration_limit = 20;
    int iterations = 0;
#endif
    do
    {
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
      if (iterations++ > iteration_limit)
      {
        break;
      }
#endif
      // Calculate a new value for the voxel.
      old_value = new_value = gputilAtomicLoadF32(occupancy_ptr);

      const bool initially_unobserved = old_value == INFINITY;
      const bool initially_free = !initially_unobserved && old_value < line_data->occupied_threshold;
      const bool initially_occupied = !initially_unobserved && old_value >= line_data->occupied_threshold;
      was_occupied_voxel = initially_occupied;

      // Check exclusion flags and skip this voxel if excluded.
      // We skip by a 'break' statement which will break out of the compare and swap loop. Could return true.
      // Check skipping unobserved.
      // We also check for zero adjustment here.
      if (initially_unobserved && (line_data->region_update_flags & kRfExcludeUnobserved) || adjustment == 0)
      {
        adjustment = 0;  // Flag null adjustment. Prevents mean update.
        break;
      }

      // Check skipping free.
      if (initially_free && (line_data->region_update_flags & kRfExcludeFree))
      {
        adjustment = 0;  // Flag null adjustment. Prevents mean update.
        break;
      }

      // Check skipping occupied.
      if (initially_occupied && (line_data->region_update_flags & kRfExcludeOccupied))
      {
        adjustment = 0;  // Flag null adjustment. Prevents mean update.
        break;
      }

      // Uninitialised voxels start at INFINITY.
      new_value = (new_value != INFINITY) ? new_value + adjustment : adjustment;
      // Clamp the value.
      new_value = clamp(new_value, line_data->voxel_value_min, line_data->voxel_value_max);

      // Now try write the value, looping if we fail to write the new value.
      // mem_fence(CLK_GLOBAL_MEM_FENCE);
    } while (new_value != old_value && !gputilAtomicCasF32(occupancy_ptr, old_value, new_value));

    if (adjustment > 0)
    {
      uint sample_count = 0;
      if (line_data->means_offsets)
      {
        ulonglong vi =
          vi_local + (line_data->means_offsets[line_data->current_region_index] / sizeof(*line_data->means));
        sample_count = updateVoxelMeanPosition(&line_data->means[vi], line_data->sample, line_data->voxel_resolution);
      }

      if (line_data->touch_times)
      {
        ulonglong vi = vi_local + (line_data->touch_times_offsets[line_data->current_region_index] /
                                   sizeof(*line_data->touch_times));
        // Cast from atomic_uint to uint for OpenCL 2.0 compatibility. The atomic_max has not atomic_<type> signatures.
        gputilAtomicMax((__global uint *)&line_data->touch_times[vi], line_data->touch_time);
      }

      if (line_data->incidents)
      {
        ulonglong vi =
          vi_local + (line_data->incidents_offsets[line_data->current_region_index] / sizeof(*line_data->incidents));
        updateVoxelIncident(&line_data->incidents[vi], line_data->sensor - line_data->sample, sample_count);
      }
    }

    // Update traversal. There is no floating based atomic arithmetic, so we must do the same CAS style update.
    if (line_data->traversal_offsets)
    {
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
      iterations = 0;
#endif
      // Work out which voxel to modify.
      vi = (line_data->traversal_offsets[line_data->current_region_index] / sizeof(*line_data->traversal)) + vi_local;
      __global atomic_float *traversal = &line_data->traversal[vi];
      old_value = new_value = gputilAtomicLoadF32(traversal);
      do
      {
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
        if (iterations++ > iteration_limit)
        {
          break;
        }
#endif
        new_value += exit_range - enter_range;
      } while (new_value != old_value && !gputilAtomicCasF32(traversal, old_value, new_value));
    }

    if (was_occupied_voxel && (line_data->region_update_flags & kRfStopOnFirstOccupied))
    {
      // Found first occupied voxel and request is to stop on the first occupied voxel. Abort traversal.
      return false;
    }
  }
  else
  {
    // printf("%u Out of bounds: %u " KEY_F "\n", get_global_id(0), vi, KEY_A(*voxel_key));
    // Abort traversal
    return false;
  }

  // Continue traversal
  return true;
}

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
__kernel void REGION_UPDATE_KERNEL(
  __global atomic_float *occupancy, __global ulonglong *occupancy_region_mem_offsets_global,  //
  __global VoxelMean *means, __global ulonglong *means_region_mem_offsets_global,             //
#ifdef NDT
  __global CovarianceVoxel *cov_voxels, __global ulonglong *cov_region_mem_offsets_global,         //
  __global HitMissCount *hit_miss_voxels, __global ulonglong *hit_miss_region_mem_offsets_global,  //
#endif
  __global atomic_float *traversal_voxels, __global ulonglong *traversal_region_mem_offsets_global,       //
  __global atomic_uint *touch_time_voxels, __global ulonglong *touch_times_region_mem_offsets_global,     //
  __global atomic_uint *incident_voxels, __global ulonglong *incidents_region_mem_offsets_global,         //
  __global int3 *occupancy_region_keys_global, uint region_count,                                         //
  __global GpuKey *line_keys, __global float3 *local_lines, uint line_count, __global uint *touch_times,  //
  int3 region_dimensions, float voxel_resolution, float ray_adjustment, float sample_adjustment,
  float occupied_threshold, float voxel_value_min, float voxel_value_max, uint region_update_flags
#ifdef NDT
  ,
  float adaptation_rate, float sensor_noise
#endif  // NDT
)
{
  // Only process valid lines.
  if (get_global_id(0) >= line_count)
  {
    return;
  }

  LineWalkData line_data;
  line_data.occupancy = occupancy;
  line_data.occupancy_offsets = occupancy_region_mem_offsets_global;
  line_data.means = means;
  line_data.means_offsets = means_region_mem_offsets_global;
#ifdef NDT
  line_data.cov_voxels = cov_voxels;
  line_data.cov_offsets = cov_region_mem_offsets_global;
  line_data.adaptation_rate = adaptation_rate;
  line_data.sensor_noise = sensor_noise;
  line_data.hit_miss = hit_miss_voxels;
  line_data.hit_miss_offsets = hit_miss_region_mem_offsets_global;
#endif  // NDT
  line_data.traversal = traversal_voxels;
  line_data.traversal_offsets = traversal_region_mem_offsets_global;
  line_data.touch_times = touch_time_voxels;
  line_data.touch_times_offsets = touch_times_region_mem_offsets_global;
  line_data.incidents = incident_voxels;
  line_data.incidents_offsets = incidents_region_mem_offsets_global;
  line_data.region_keys = occupancy_region_keys_global;
  line_data.region_dimensions = region_dimensions;
  line_data.voxel_resolution = voxel_resolution;
  line_data.ray_adjustment = ray_adjustment;
  line_data.sample_adjustment = sample_adjustment;
  line_data.occupied_threshold = occupied_threshold;
  line_data.voxel_value_min = voxel_value_min;
  line_data.voxel_value_max = voxel_value_max;
  line_data.region_count = region_count;
  line_data.region_update_flags = region_update_flags;

  regionsInitCurrent(&line_data.current_region, &line_data.current_region_index);

  // Now walk the clipped ray.
  GpuKey start_key, end_key;
  copyKey(&start_key, &line_keys[get_global_id(0) * 2 + 0]);
  copyKey(&end_key, &line_keys[get_global_id(0) * 2 + 1]);

  const float3 line_start = local_lines[get_global_id(0) * 2 + 0];
  const float3 line_end = local_lines[get_global_id(0) * 2 + 1];

  line_data.sample = line_end;
  line_data.sensor = line_start;
  line_data.touch_time = (touch_times) ? touch_times[get_global_id(0)] : 0;

  // For reverse line walk, the start voxel centre is always (0, 0, 0).
  float3 start_voxel_centre = make_float3(0.0f, 0.0f, 0.0f);
  int walk_flags = 0;
  if (region_update_flags & kRfReverseWalk)
  {
    walk_flags |= kLineWalkFlagReverse;
#ifndef NDT
    // For non-NDT (pure occupancy) updates, force reporting the sample last. This yields better occupancy behaviour
    // and less erosion.
    walk_flags |= kLineWalkFlagForReportEndLast;
#endif  // !NDT
  }

  // Call the line walking function.
  REGION_WALK_VOXELS(&start_key, &end_key, line_start, line_end, region_dimensions, voxel_resolution, walk_flags,
                     &line_data);
}

#undef REGION_UPDATE_KERNEL
#undef REGION_WALK_VOXELS
#undef REGION_VISIT_VOXEL
#undef VOXEL_TYPE

#ifndef REGION_UPDATE_BASE_CL
#define REGION_UPDATE_BASE_CL
#endif  // REGION_UPDATE_BASE_CL
