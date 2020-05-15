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
// - VOXEL_MEAN : defined if compiling for voxel means
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
#include "gpu_ext.h"

// Explicitly include MapCoord.h first. It's included from each of the subsequent includes, but leaving it to
// VoxelMean.h has issues with the resource generation. Essentially it causes MapCoord.h to be only included within the
// VOXEL_MEAN define.
#include "MapCoord.h"
#if defined(VOXEL_MEAN) || defined(NDT)
#include "VoxelMean.h"
#endif  // VOXEL_MEAN || NDT
#include "RayFlag.h"
#ifdef NDT
#include "NdtVoxel.h"
#endif  // NDT

#include "Regions.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

#ifdef NDT
#define REGION_UPDATE_KERNEL regionRayUpdateNdt
#define VISIT_LINE_VOXEL visitVoxelRegionUpdateNdt
#define WALK_LINE_VOXELS walkRegionLineNdt

#elif defined(VOXEL_MEAN)
#define REGION_UPDATE_KERNEL regionRayUpdateSubVox
#define VISIT_LINE_VOXEL visitVoxelRegionUpdateSubVox
#define WALK_LINE_VOXELS walkRegionLineSubVox

#else  // VOXEL_MEAN
#define REGION_UPDATE_KERNEL regionRayUpdate
#define VISIT_LINE_VOXEL visitVoxelRegionUpdate
#define WALK_LINE_VOXELS walkRegionLine

#endif

#ifndef REGION_UPDATE_BASE_CL
// User data for voxel visit callback.
struct LineWalkData
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global atomic_float *occupancy;
  // Array of offsets for each regionKey into occupancy. These are byte offsets.
  __global ulonglong *occupancy_offsets;
#if defined(VOXEL_MEAN) || defined(NDT)
  __global VoxelMean *means;
  // Array of offsets for each regionKey into means. These are byte offsets.
  __global ulonglong *means_offsets;
#endif // VOXEL_MEAN || NDT
#ifdef NDT
  __global NdtVoxel *ndt_voxels;
  // Array of offsets for each regionKey into ndt_voxels. These are byte offsets.
  __global ulonglong *ndt_offsets;
#endif  // NDT
  // Array of region keys for currently loaded regions.
  __global int3 *region_keys;
  // // Array of offsets for each regionKey into voxels. These are byte offsets.
  // __global ulonglong *region_mem_offsets;
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
  // Number of regions in region_keys/region_mem_offsets.
  uint region_count;
  // Index of the @c current_region into region_keys and corresponding xxx_offsets arrays.
  uint current_region_index;
  uint region_update_flags;
  // Local coordinate within the end voxel.
  float3 sub_voxel_coord;
#ifdef NDT
  /// Modified sensor position. TODO: clarify "modified"
  float3 sensor;
  /// Modified sample position. TODO: clarify "modified"
  float3 sample;
  // An estimate on the sensor range noise error.
  float sensor_noise;
#endif  // NDT
};
#endif  // REGION_UPDATE_BASE_CL

#if defined(VOXEL_MEAN) || defined(NDT)
#include "VoxelMean.cl"
#endif  // VOXEL_MEAN || NDT

#ifdef NDT
#include "AdjustNdt.cl"
#else  // NDT
#include "AdjustOccupancy.cl"
#endif  // NDT

// Implement the voxel traversal function. We update the value of the voxel using atomic instructions.
__device__ bool VISIT_LINE_VOXEL(const struct GpuKey *voxelKey, bool isEndVoxel,
                                 const struct GpuKey *startKey, const struct GpuKey *endKey,
                                 float voxel_resolution, void *userData)
{
  float old_value, new_value;

  struct LineWalkData *line_data = (struct LineWalkData *)userData;
  __global atomic_float *occupancy = line_data->occupancy;

  if (isEndVoxel && (line_data->region_update_flags & kRfExcludeSample))
  {
    return true;
  }

  // Resolve memory offset for the region of interest.
  if (!regionsResolveRegion(voxelKey, &line_data->current_region, &line_data->current_region_index,
                            line_data->region_keys, line_data->region_count))
  {
    // We can fail to resolve regions along the in the line. This can occurs for several reasons:
    // - Floating point error differences between CPU and GPU line walking means that the GPU may walk into the edge
    //   of a region not hit when walking the regions on CPU.
    // - Regions may not be uploaded due to extents limiting on CPU.
#ifdef REPORT_MISSING_REGIONS
    printf("%u region missing: " KEY_F "\n  Voxels: " KEY_F "->" KEY_F "\n",
           get_global_id(0), KEY_A(*voxelKey), KEY_A(*startKey), KEY_A(*endKey)
    );
#endif
    return true;
  }

  // Adjust value by ray_adjustment unless this is the sample voxel.
  const float adjustment =
    calculateOccupancyAdjustment(voxelKey, isEndVoxel, startKey, endKey, voxel_resolution, line_data);

  // This voxel lies in the region. We will make a value adjustment.
  // Work out which voxel to modify.
  const ulonglong vi_local = voxelKey->voxel[0] + voxelKey->voxel[1] * line_data->region_dimensions.x +
                             voxelKey->voxel[2] * line_data->region_dimensions.x * line_data->region_dimensions.y;
  ulonglong vi =
    (line_data->occupancy_offsets[line_data->current_region_index] / sizeof(*line_data->occupancy)) + vi_local;

  if (voxelKey->voxel[0] < line_data->region_dimensions.x && voxelKey->voxel[1] < line_data->region_dimensions.y &&
      voxelKey->voxel[2] < line_data->region_dimensions.z)
  {
    __global atomic_float *occupancy_ptr = &occupancy[vi];

    bool was_occupied_voxel = false;

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

#if defined(VOXEL_MEAN) || defined(NDT)
    if (adjustment > 0)
    {
      ulonglong vi =
        vi_local + (line_data->means_offsets[line_data->current_region_index] / sizeof(*line_data->means));
      updateVoxelMeanPosition(&line_data->means[vi], line_data->sub_voxel_coord, voxel_resolution);
    }
#endif // VOXEL_MEAN || NDT

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
/// - kRfClearOnly: only adjust the probability of occupied voxels.
/// - kRfExcludeSample: ignore the voxel containing the sample (the last voxel).
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
///     relative to the centre of the voxel containing the start point. This is to reduce floating point error in
///     double to single precision conversion. The original coordinates are not recoverable in this code.
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
__kernel void REGION_UPDATE_KERNEL(__global atomic_float *occupancy, __global ulonglong *occupancy_region_mem_offsets_global,
#if defined(VOXEL_MEAN) || defined(NDT)
                                   __global VoxelMean *means, __global ulonglong *means_region_mem_offsets_global,
#endif // VOXEL_MEAN || NDT
#ifdef NDT
                                   __global NdtVoxel *ndt_voxels, __global ulonglong *ndt_region_mem_offsets_global,
#endif  // NDT
                                   __global int3 *occupancy_region_keys_global, uint region_count,
                                   __global struct GpuKey *line_keys, __global float3 *local_lines, uint line_count,
                                   int3 region_dimensions, float voxel_resolution, float ray_adjustment,
                                   float sample_adjustment, float occupied_threshold, float voxel_value_min,
                                   float voxel_value_max, uint region_update_flags)
{
  // Only process valid lines.
  if (get_global_id(0) >= line_count)
  {
    return;
  }

  struct LineWalkData line_data;
  line_data.occupancy = occupancy;
  line_data.occupancy_offsets = occupancy_region_mem_offsets_global;
#if defined(VOXEL_MEAN) || defined(NDT)
  line_data.means = means;
  line_data.means_offsets = means_region_mem_offsets_global;
#endif // VOXEL_MEAN || NDT
#ifdef NDT
  line_data.ndt_voxels = ndt_voxels;
  line_data.ndt_offsets = ndt_region_mem_offsets_global;
#endif  // NDT
  line_data.region_keys = occupancy_region_keys_global;
  line_data.region_dimensions = region_dimensions;
  line_data.ray_adjustment = ray_adjustment;
  line_data.sample_adjustment = sample_adjustment;
  line_data.occupied_threshold = occupied_threshold;
  line_data.voxel_value_min = voxel_value_min;
  line_data.voxel_value_max = voxel_value_max;
  line_data.region_count = region_count;
  line_data.region_update_flags = region_update_flags;

  regionsInitCurrent(&line_data.current_region, &line_data.current_region_index);

  // Now walk the clipped ray.
  struct GpuKey start_key, end_key;
  copyKey(&start_key, &line_keys[get_global_id(0) * 2 + 0]);
  copyKey(&end_key, &line_keys[get_global_id(0) * 2 + 1]);

  const float3 lineStart = local_lines[get_global_id(0) * 2 + 0];
  const float3 lineEnd = local_lines[get_global_id(0) * 2 + 1];

#if defined(VOXEL_MEAN) || defined(NDT)
  // We don't need a precise conversion to a voxel key here. We simply need to logically quantise in order to work out
  // the voxel mean offset. Essentially we have:
  //   s = E - R floor(E/R + 0.5)
  // where:
  //   s: voxel mean position
  //   E: ray End point
  //   R: voxel resolution
  // The addition of 0.5 applies the same half voxel offset used elsewhere. We use pointToRegionCoord() to do this
  line_data.sub_voxel_coord.x = lineEnd.x - pointToRegionCoord(lineEnd.x, voxel_resolution) * voxel_resolution;
  line_data.sub_voxel_coord.y = lineEnd.y - pointToRegionCoord(lineEnd.y, voxel_resolution) * voxel_resolution;
  line_data.sub_voxel_coord.z = lineEnd.z - pointToRegionCoord(lineEnd.z, voxel_resolution) * voxel_resolution;

  // Validate voxel mean coordinate calculation.
  // We use 0.5001 * resolution rather than 0.5 to allow for floating point error when clipping to exact voxel bounds.
  if (line_data.sub_voxel_coord.x < -0.5001f * voxel_resolution ||
      line_data.sub_voxel_coord.x > 0.5001f * voxel_resolution ||
      line_data.sub_voxel_coord.y < -0.5001f * voxel_resolution ||
      line_data.sub_voxel_coord.y > 0.5001f * voxel_resolution ||
      line_data.sub_voxel_coord.z < -0.5001f * voxel_resolution ||
      line_data.sub_voxel_coord.z > 0.5001f * voxel_resolution)
  {
    printf("voxel mean-out [%f, %f]: (%f %f %f) -> (%f %f %f)\n", -0.5f * voxel_resolution, 0.5f * voxel_resolution,
           lineEnd.x, lineEnd.y, lineEnd.z, line_data.sub_voxel_coord.x, line_data.sub_voxel_coord.y,
           line_data.sub_voxel_coord.z);
  }
#else   // VOXEL_MEAN || NDT
  line_data.sub_voxel_coord = make_float3(0, 0, 0);
#endif  // VOXEL_MEAN || NDT

#ifdef NDT
  line_data.sensor = lineStart;
  line_data.sample = lineEnd;
#endif //  NDT

  WALK_LINE_VOXELS(&start_key, &end_key, &lineStart, &lineEnd, &region_dimensions, voxel_resolution, &line_data);
}

#undef REGION_UPDATE_KERNEL
#undef VISIT_LINE_VOXEL
#undef WALK_LINE_VOXELS
#undef VOXEL_TYPE

#ifndef REGION_UPDATE_BASE_CL
#define REGION_UPDATE_BASE_CL
#endif  // REGION_UPDATE_BASE_CL
