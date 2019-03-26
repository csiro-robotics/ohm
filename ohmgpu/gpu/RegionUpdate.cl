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
// - SUB_VOXEL : defined if compiling for sub-voxels
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

#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
// Store additional debug information in LineWalkData for error reporting.
//#define STORE_DEBUG_INFO
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "gpu_ext.h"

// Explicitly include MapCoord.h first. It's included from each of the subsequent includes, but leaving it to SubVoxel.h
// has issues with the resource generation. Essentially it causes MapCoord.h to be only included within the SUB_VOXEL
// define.
#include "MapCoord.h"
#include "SubVoxel.h"
#include "RayFlag.h"

#include "Regions.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

#ifndef REGION_UPDATE_BASE_CL
typedef struct VoxelSubVox_
{
  atomic_float occupancy;
  atomic_uint sub_voxel;
} VoxelSubVox;

typedef struct VoxelSimple_
{
  atomic_float occupancy;
} VoxelSimple;
#endif  // REGION_UPDATE_BASE_CL

#ifdef SUB_VOXEL
#define REGION_UPDATE_KERNEL regionRayUpdateSubVox
#define VISIT_LINE_VOXEL visitVoxelRegionUpdateSubVox
#define WALK_LINE_VOXELS walkRegionLineSubVox
#define VOXEL_TYPE VoxelSubVox

#else  // SUB_VOXEL
#define REGION_UPDATE_KERNEL regionRayUpdate
#define VISIT_LINE_VOXEL visitVoxelRegionUpdate
#define WALK_LINE_VOXELS walkRegionLine
#define VOXEL_TYPE VoxelSimple

#endif  // SUB_VOXEL

#ifndef REGION_UPDATE_BASE_CL
// User data for voxel visit callback.
struct LineWalkData
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global void *voxels;
  // Array of region keys for currently loaded regions.
  __global int3 *region_keys;
  // Array of offsets for each regionKey into voxels. These are byte offsets.
  __global ulonglong *region_mem_offsets;
  // The region currently being traversed. Also used to reduce searching the region_keys and region_mem_offsets.
  int3 current_region;
  // Size of a region in voxels.
  int3 region_dimensions;
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
  // Weighting given to new position when integrating sub-voxel positions.
  float sub_voxel_weighting;
  // Number of regions in region_keys/region_mem_offsets.
  uint region_count;
  // The region_mem_offsets value corresponding to the current_region. This is an index offset into voxels, not
  // a byte offset.
  uint region_voxel_offset;
  uint region_update_flags;
  // Local coordinate within the end voxel.
  float3 sub_voxel_coord;
#ifdef STORE_DEBUG_INFO
  const struct GpuKey *start_key;
  const struct GpuKey *end_key;
#endif  // STORE_DEBUG_INFO
};
#endif  // REGION_UPDATE_BASE_CL

#ifdef SUB_VOXEL
/// Update the sub-voxel pattern at @p target_address by including the bit(s) from @p pattern_to_add.
/// This is done using atomic operations.
///
/// Each bit in the pattern indicates occupancy at a particular sub-voxel location.
__device__ void updateSubVoxelPosition(__global atomic_uint *target_address, float3 sub_voxel_pos,
                                       float voxel_resolution, float sub_voxel_weigthing);
#endif  // SUB_VOXEL

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// Psuedo header guard to prevent function implementation duplication.
#ifdef SUB_VOXEL
__device__ void updateSubVoxelPosition(__global atomic_uint *target_address, float3 sub_voxel_pos,
                                       float voxel_resolution, float sub_voxel_weigthing)
{
  uint old_value;
  uint new_value;

  // Few iterations as it's less important to get this right.
  const int iteration_limit = 10;
  int iteration_count = 0;
  do
  {
    old_value = gputilAtomicLoadU32(target_address);
    new_value = subVoxelUpdate(old_value, sub_voxel_pos, voxel_resolution, sub_voxel_weigthing);
    ++iteration_count;
  } while (!gputilAtomicCasU32(target_address, old_value, new_value) && iteration_limit < iteration_count);
}
#endif  // SUB_VOXEL


// Implement the voxel traversal function. We update the value of the voxel using atomic instructions.
__device__ bool VISIT_LINE_VOXEL(const struct GpuKey *voxelKey, bool isEndVoxel, float voxel_resolution, void *userData)
{
  float old_value, new_value;

  struct LineWalkData *line_data = (struct LineWalkData *)userData;
  __global VOXEL_TYPE *voxels = (__global VOXEL_TYPE *)line_data->voxels;

  // Adjust value by ray_adjustment unless this is the sample voxel.
  const float adjustment = (!isEndVoxel || line_data->region_update_flags & kRfEndPointAsFree) ?
            line_data->ray_adjustment : line_data->sample_adjustment;

  // Resolve memory offset for the region of interest.
  if (!regionsResolveRegion(voxelKey, &line_data->current_region, &line_data->region_voxel_offset, line_data->region_keys,
                            line_data->region_mem_offsets, line_data->region_count, sizeof(VOXEL_TYPE)))
  {
    // We can fail to resolve regions along the in the line. This can occurs for several reasons:
    // - Floating point error differences between CPU and GPU line walking means that the GPU may walk into the edge
    //   of a region not hit when walking the regions on CPU.
    // - Regions may not be uploaded due to extents limiting on CPU.
#ifdef REPORT_MISSING_REGIONS
    printf("%u region missing: " KEY_F "\n"
#ifdef STORE_DEBUG_INFO
           "  Voxels: " KEY_F "->" KEY_F "\n"
#endif
           ,
           get_global_id(0), KEY_A(*voxelKey)
#ifdef STORE_DEBUG_INFO
                               ,
           KEY_A(*line_data->start_key), KEY_A(*line_data->end_key)
#endif
    );
#endif
    return true;
  }

  // This voxel lies in the region. We will make a value adjustment.
  // Work out which voxel to modify.
  ulonglong vi = voxelKey->voxel[0] + voxelKey->voxel[1] * line_data->region_dimensions.x +
             voxelKey->voxel[2] * line_data->region_dimensions.x * line_data->region_dimensions.y +
             line_data->region_voxel_offset;

  if (voxelKey->voxel[0] < line_data->region_dimensions.x && voxelKey->voxel[1] < line_data->region_dimensions.y &&
      voxelKey->voxel[2] < line_data->region_dimensions.z)
  {
    __global atomic_float *occupancy_ptr = &voxels[vi].occupancy;

    bool was_occupied_voxel = false;

    // i

#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
    // Under high contension we can end up repeatedly failing to write the voxel value.
    // The primary concern is not deadlocking the GPU, so we put a hard limit on the numebr of
    // attempts made.
    const int iterationLimit = 20;
    int iterations = 0;
#endif
    do
    {
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
      if (iterations++ > iterationLimit)
      {
        break;
      }
#endif
      // Calculate a new value for the voxel.
      old_value = new_value = gputilAtomicLoadF32(occupancy_ptr);

      if (old_value < 0 && (line_data->region_update_flags & kRfClearOnly))
      {
        // kRfClearOnly flag set => only affect occupied voxels.
        // This is not an occupied voxel: skip.
        break;
      }

      was_occupied_voxel = line_data->occupied_threshold <= old_value && old_value < INFINITY;

      // Uninitialised voxels start at INFINITY.
      new_value = (new_value != INFINITY) ? new_value + adjustment : adjustment;
      // Clamp the value.
      new_value = clamp(new_value, line_data->voxel_value_min, line_data->voxel_value_max);

      // Now try write the value, looping if we fail to write the new value.
      // mem_fence(CLK_GLOBAL_MEM_FENCE);
    } while (new_value != old_value && !gputilAtomicCasF32(occupancy_ptr, old_value, new_value));

#ifdef SUB_VOXEL
    if (adjustment > 0)
    {
      updateSubVoxelPosition(&voxels[vi].sub_voxel, line_data->sub_voxel_coord, voxel_resolution,
                             line_data->sub_voxel_weighting);
    }
#endif  // SUB_VOXEL

    if (was_occupied_voxel && (line_data->region_update_flags & kRfStopOnFirstOccupied))
    {
      // Found first occupied voxel and request is to stop on the first occupied voxel. Abort traversal.
      return false;
    }
  }
 else
  {
    // printf("%u Out of bounds: %u " KEY_F "\n", get_global_id(0), vi, KEY_A(*voxelKey));
    // Abort traversal
    return false;
  }

  // Continue traversal
  return true;
}

// Must be included after WALK_LINE_VOXELS and VISIT_LINE_VOXEL and the VISIT_LINE_VOXEL function is defined
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
/// For each voxel key along the line, we resolve a voxel in @p voxels_mem by cross referencing in
/// @p occupancy_region_keys_global, @p occupancy_region_mem_offsets_global and @p region_count. Voxels are split into
/// regions in contiguous chunks in @p voxels_mem. The @c GpuKey::region for a voxel is matched in lookup @p
/// occupancy_region_keys_global and the index into @p occupancy_region_keys_global recorded. This index is used to
/// lookup @p occupancy_region_mem_offsets_global, which provides a byte offset (not elements) from @p voxels_mem at
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
/// - kRfClearOnly: only adjust the probability of occupied voxels.
__kernel void REGION_UPDATE_KERNEL(__global VOXEL_TYPE *voxels_mem, __global int3 *occupancy_region_keys_global,
                                   __global ulonglong *occupancy_region_mem_offsets_global, uint region_count,
                                   __global struct GpuKey *line_keys, __global float3 *local_lines, uint line_count,
                                   int3 region_dimensions, float voxel_resolution, float ray_adjustment,
                                   float sample_adjustment, float occupied_threshold,
                                   float voxel_value_min, float voxel_value_max,
                                   float sub_voxel_weighting, uint region_update_flags)
{
  // Only process valid lines.
  if (get_global_id(0) >= line_count)
  {
    return;
  }

  struct LineWalkData line_data;
  line_data.voxels = voxels_mem;
  line_data.region_keys = occupancy_region_keys_global;
  line_data.region_mem_offsets = occupancy_region_mem_offsets_global;
  line_data.region_dimensions = region_dimensions;
  line_data.ray_adjustment = ray_adjustment;
  line_data.sample_adjustment = sample_adjustment;
  line_data.occupied_threshold = occupied_threshold;
  line_data.voxel_value_min = voxel_value_min;
  line_data.voxel_value_max = voxel_value_max;
  line_data.sub_voxel_weighting = sub_voxel_weighting;
  line_data.region_count = region_count;
  line_data.region_update_flags = region_update_flags;

  regionsInitCurrent(&line_data.current_region, &line_data.region_voxel_offset);

  // Now walk the clipped ray.
  struct GpuKey start_key, end_key;
  copyKey(&start_key, &line_keys[get_global_id(0) * 2 + 0]);
  copyKey(&end_key, &line_keys[get_global_id(0) * 2 + 1]);

  const float3 lineStart = local_lines[get_global_id(0) * 2 + 0];
  const float3 lineEnd = local_lines[get_global_id(0) * 2 + 1];

#ifdef SUB_VOXEL
  // We don't need a precise conversion to a voxel key here. We simply need to logically quantise in order to work out
  // the sub-voxel offset. Essentially we have:
  //   s = E - R floor(E/R + 0.5)
  // where:
  //   s: sub-voxel position
  //   E: ray End point
  //   R: voxel resolution
  // The addition of 0.5 applies the same half voxel offset used elsewhere. We use pointToRegionCoord() to do this
  line_data.sub_voxel_coord.x = lineEnd.x - pointToRegionCoord(lineEnd.x, voxel_resolution) * voxel_resolution;
  line_data.sub_voxel_coord.y = lineEnd.y - pointToRegionCoord(lineEnd.y, voxel_resolution) * voxel_resolution;
  line_data.sub_voxel_coord.z = lineEnd.z - pointToRegionCoord(lineEnd.z, voxel_resolution) * voxel_resolution;

  // Validate sub-voxel coordinate calculation.
  // We use 0.5001 * resolution rather than 0.5 to allow for floating point error when clipping to exact voxel bounds.
  if (line_data.sub_voxel_coord.x < -0.5001 * voxel_resolution || line_data.sub_voxel_coord.x > 0.5001 * voxel_resolution ||
      line_data.sub_voxel_coord.y < -0.5001 * voxel_resolution || line_data.sub_voxel_coord.y > 0.5001 * voxel_resolution ||
      line_data.sub_voxel_coord.z < -0.5001 * voxel_resolution || line_data.sub_voxel_coord.z > 0.5001 * voxel_resolution)
  {
    printf("sub-voxel-out [%f, %f]: (%f %f %f) -> (%f %f %f)\n", -0.5 * voxel_resolution, 0.5 * voxel_resolution,
           lineEnd.x, lineEnd.y, lineEnd.z, line_data.sub_voxel_coord.x, line_data.sub_voxel_coord.y,
           line_data.sub_voxel_coord.z);
  }
#else  // SUB_VOXEL
  line_data.sub_voxel_coord = make_float3(0, 0, 0);
#endif // SUB_VOXEL

#ifdef STORE_DEBUG_INFO
  line_data.start_key = &start_key;
  line_data.end_key = &end_key;
#endif

  WALK_LINE_VOXELS(&start_key, &end_key, &lineStart, &lineEnd, &region_dimensions, voxel_resolution, &line_data);
}

#undef REGION_UPDATE_KERNEL
#undef VISIT_LINE_VOXEL
#undef WALK_LINE_VOXELS
#undef VOXEL_TYPE

#ifndef REGION_UPDATE_BASE_CL
#define REGION_UPDATE_BASE_CL
#endif  // REGION_UPDATE_BASE_CL
