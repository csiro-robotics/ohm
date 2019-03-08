// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpu_ext.h"

#ifdef SUB_VOXEL
#define isOccupied isOccupiedSubVox
#define seedRegionVoxels seedRegionVoxelsSubVox
#define seedFromOuterRegions seedFromOuterRegionsSubVox
#define propagateObstacles propagateObstaclesSubVox
#define migrateResults migrateResultsSubVox
#endif  // SUB_VOXEL

// Explicitly include MapCoord.h first. It's included from each of the subsequent includes, but leaving it to SubVoxel.h
// has issues with the resource generation. Essentially it causes MapCoord.h to be only included within the SUB_VOXEL
// define.
#include "MapCoord.h"

#include "Regions.cl"

#ifdef SUB_VOXEL
#include "SubVoxel.h"
#endif  // SUB_VOXEL

/// @defgroup voxelClearanceGpu Voxel Clearance GPU
/// @{
/// @brief Documentation on GPU code used to calculate the clearance to the nearest obstacle for each voxel.
///
/// This program has each voxel calculate the range to the nearest obstructed voxel, including itself. An obstructed
/// voxel is an occupied voxel, or an uncertain voxel if @c unknownAsOccupied is set. Clearance is only calculated to
/// the requested range, reporting -1 for any voxel with no obstruction within that range.
///
/// The calculation works using a flood fill algorithm, as follows:
/// -# Seed voxels using @c seedObstacleVoxels() to contain an @c char4 with the following information:
///   - @c xyz defines the voxel coordinates relative to the 3D voxel block being processed.
///     The lower, bottom, left corner is (0, 0, 0).
///   - @c w set to 1 for an occupied voxel and zero for a free voxel. Uncertain voxels attain either depending @c
///   unknownAsOccupied
/// -# Iteratively call @c propagateObstacles to propagate obstacle flood fill. Each voxel does the following:
///   - Inspect immediate neighbours (8) and select the closest obstacle from these and its current value.
///     - If voxel does not contain an obstacle (w = 0) select an occupied neighbour with closest @c xyz values.
///     - If voxel contains an obstacle (w = 1) select an occupied neighbour closer than the current @c xyz value.
///       - Maintain current @c xyz no neighbour tracks a closer voxel.
/// -# Repeat 2. as many times as required to ensure we have propagated the wave fronts to the required range.
///
/// For efficiency, voxels for all required regions are maintained in a single GPU buffer cache (see @c GpuLayerCache).
/// This buffer is given as the @c voxelsMem parameter. A region may at any fixed interface within this buffer
/// (interval determined by the region memory size). The @c regionKeysGlobal and @c regionMemOffsetsGlobal parameters
/// are required to map from a region key (in the @c regionKeysGlobal array) to its corresponding memory offset from
/// @c regionMemOffsetGlobal. This offset is used to find the region memory block in @c voxelsMem.
///
/// An optional axis scaling may be applied using @p axisScaling affecting how ranges is calculated along each
/// primary axis. This vector effectively distorts the space by the following matrix:
/// @code{.unparsed}
///   axisScaling.x   0               0
///   0               axisScaling.y   0
///   0               0               axisScaling.z
/// @endcode
///
/// Note the scaling may optionally be omitted from the reported range by using scaling vector other than (1, 1, 1)
/// during propagation, but using the vector (1, 1, 1) for @c migrateResults().

#ifndef ROI_RANGE_FILL_BASE_CL
//-----------------------------------------------------------------------------
// Compile switches
//-----------------------------------------------------------------------------
// Validate we touch all local memory required.
#define VALIDATE_LOCAL_MEM_LOAD 0

#define DBG_X 34
#define DBG_Y 28
#define DBG_Z 28

// TODO: include flags header instead of repeating definitions here.
#define kUnknownAsOccupied (1 << 0)
#define kSubVoxelFiltering (1 << 1)

// Limit the number of cells we can traverse in the line traversal. This is a worst case limit.
//#define LIMIT_LINE_WALK_ITERATIONS
// Limit the number of times we try update a voxel value. Probably best to always have this enabled.
#define LIMIT_VOXEL_WRITE_ITERATIONS
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
// Store additional debug information in LineWalkData for error reporting.
//#define STORE_DEBUG_INFO
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS

//-----------------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------------
typedef atomic_uint voxel_type;
typedef float OccupancyType;
typedef uint SubVoxelPatternType;

#endif  // ROI_RANGE_FILL_BASE_CL

#ifndef ROI_RANGE_FILL_BASE_CL
typedef struct VoxelSubVox_
{
  OccupancyType occupancy;
  SubVoxelPatternType sub_voxel;
} VoxelSubVox;

typedef struct VoxelSimple_
{
  OccupancyType occupancy;
} VoxelSimple;
#endif  // ROI_RANGE_FILL_BASE_CL

#ifdef SUB_VOXEL
#define VOXEL_TYPE VoxelSubVox
#else  // SUB_VOXEL
#define VOXEL_TYPE VoxelSimple
#endif  // SUB_VOXEL

//-----------------------------------------------------------------------------
// Function prototypes.
//-----------------------------------------------------------------------------
bool __device__ isOccupied(const VOXEL_TYPE *value, float threshold, unsigned flags,
                                      float sub_voxel_filter_scale);
/// Get the index of a voxel within the provided extents.
int __device__ getGlobalVoxelIndex(int3 coord, int3 voxelExtents);
int3 __device__ linearIndexToCoord(unsigned linearIndex, int3 expanse);
bool __device__ isInExpanse(int3 coord, int3 expanseMin, int3 expanseMax);
bool __device__ onROIEdge(int3 coord, int3 roiExtents);
/// Calculate the volume of @p expanse: <tt>x * y * z</tt>.
unsigned __device__ volumeOf(int3 expanse);

/// Select the best obstruction considering the neighbour of the current voxel offset
/// at <tt>(nx, ny, nz)</tt>. The current voxel coordinate is determined by the
/// get_local_id(N) indices.
///
/// @param nx Neighbour offset in x.
/// @param ny Neighbour offset in y.
/// @param nz Neighbour offset in z.
/// @param currentClosest Coordinates of the current best obstruction with w=1 for obstacle, w=0 otherwise.
/// @param localVoxel Cache of voxels with their current best results.
/// @param axisScaling Scaling applied to each axis when calculating the nearest obstruction. See comments above.
/// @return The closest obstructing voxel selected from the neighbour and @p currentClosest.
char4 __device__ selectObstructionForNeighbour(int nx, int ny, int nz, char4 currentClosest, float *currentDistSqr,
                                               __local char4 *localVoxels, float3 axisScaling);

/// Precalculate parameters for @c resolvePaddingVoxelIndex().
void __device__ calculateFacePadding(int3 workingVoxelExtents, int3 outerRegionPadding, __local uint *faceVolumeSizes,
                                     __local int3 *minFaceExtents, __local int3 *maxFaceExtents,
                                     __local int3 *faceVolumes);

/// Convert an index into the padding region into a 3D index local to the @p workingVoxelExtents.
///
/// The @p workingVoxelExtents defines a target region of voxels of interest with 3D indices in the range
/// [0, workingVoxelExtents). Meanwhile the region [-outerRegionPadding, workingVoxelExtents + outerRegionPadding)
/// defines a padded region of interest (to read from). This function assumes we have a 1D @p index into the
/// padding region, skipping over the region [0, workingVoxelExtents). This @p index is converted into a 3D
/// index in the range [-outerRegionPadding, workingVoxelExtents + outerRegionPadding), excluding indices
/// [0, workingVoxelExtents).
///
/// @param index The 1D index into the padding.
/// @param workingVoxelExtents Defines the void region to skip.
/// @param outerRegionPadding Defines the padding added to @p workingVoxelExtents in which we do index.
/// @return A 3D index in the range [-outerRegionPadding, workingVoxelExtents + outerRegionPadding), excluding the
///   range [0, workingVoxelExtents). Returns (0, 0, 0) on failure.
int3 __device__ resolvePaddingVoxelIndex(uint index, int3 workingVoxelExtents, int3 outerRegionPadding,
                                         __local uint *faceVolumeSizes, __local int3 *minFaceExtents,
                                         __local int3 *maxFaceExtents, __local int3 *faceVolumes);

/// Find the voxel index within the @p voxelExtents closest to @p voxelIndex3 (outside the region).
///
/// @param voxelIndex3 Index to find a valid voxel in @p voxelExtents. General usage expects this is
///     outside the @p voxelExtents.
/// @param voxelExtents Defines the voxel region of interest. A valid index lies within [0, voxelExtents).
/// @return The closest voxel in [0, voxelExtents) to @p voxelIndex3, or @p voxelIndex on failure.
int3 __device__ findClosestRegionVoxel(int3 voxelIndex3, int3 voxelExtents);

/// Update a working voxel obstruction using compare and swap semantics.
///
/// The new value, @p newObstruction, is only written if it reresents a voxel closer than the existing value at
/// @p voxel. This method supports write contention on @p voxel.
///
/// @param voxelIndex The voxel coordiantes in the region of interest.
/// @param voxel A pointer to the @p voxel data.
/// @param newObstruction The new obstruction to write. XYZ are relative offsets from @p voxelIndex, while W must be 1.
/// @param axisScaling Per axis scaling applied to distance calculations.
bool __device__ updateVoxelObstructionCas(int3 voxelIndex, __global voxel_type *voxel, char4 newObstruction,
                                          float3 axisScaling);

// Load voxel data into local memory.
void __device__ loadPropagationLocalVoxels(__global char4 *srcVoxels, __local char4 *localVoxels, int3 voxelExtents,
                                           int3 globalWorkItem, int3 localWorkItem, int3 localExpance);

/// Convert from obstruction value to uint for use in atomic operations in @c seedFromOuterRegions()
/// @param obstruction Offset to nearest current obstruction, with w = 1 if there is an obstruction, w = 0 otherwise.
/// @return A 32-bit uint representation of obstruction.
inline uint __device__ obstructionToVoxel(char4 obstruction);
/// Convert from 32-bit uint voxel representation to a char4 obstruction representation for use in atomic operations in
/// @c seedFromOuterRegions()
/// @param voxel The voxel representation to convert.
/// @retrun Offset to nearest current obstruction, with w = 1 if there is an obstruction, w = 0 otherwise.
inline char4 __device__ voxelToObstruction(uint voxel);

//-----------------------------------------------------------------------------
// Implementation
//-----------------------------------------------------------------------------
bool __device__ isOccupied(const VOXEL_TYPE *voxel, float threshold, unsigned flags,
                                      float sub_voxel_filter_scale)
{
  return (voxel->occupancy == INFINITY && (flags & kUnknownAsOccupied)) ||
         (voxel->occupancy != INFINITY && voxel->occupancy >= threshold
          // #ifdef SUB_VOXEL
          //           && (!(flags & kSubVoxelFiltering) || subVoxelOccupancyFilter2(voxel->sub_voxel,
          //           sub_voxel_filter_scale))
          // #endif  // SUB_VOXEL
         );
}

#ifndef ROI_RANGE_FILL_BASE_CL
int __device__ getGlobalVoxelIndex(int3 coord, int3 voxelExtents)
{
  if (isInExpanse(coord, make_int3(0, 0, 0), voxelExtents))
  {
    return coord.x + coord.y * voxelExtents.x + coord.z * voxelExtents.x * voxelExtents.y;
  }

  return -1;
}


int3 __device__ linearIndexToCoord(unsigned linearIndex, int3 expanse)
{
  int3 coord;
  coord.x = (int)(linearIndex % expanse.x);
  linearIndex /= expanse.x;
  coord.y = (int)(linearIndex % expanse.y);
  linearIndex /= expanse.y;
  coord.z = (int)linearIndex;
  return coord;
}

bool __device__ isInExpanse(int3 coord, int3 expanseMin, int3 expanseMax)
{
  return expanseMin.x <= coord.x && coord.x < expanseMax.x && expanseMin.y <= coord.y && coord.y < expanseMax.y &&
         expanseMin.z <= coord.z && coord.z < expanseMax.z;
}

bool __device__ onROIEdge(int3 coord, int3 roiExtents)
{
  return
    // in region
    0 <= coord.x && coord.x < roiExtents.x && 0 <= coord.y && coord.y < roiExtents.y && 0 <= coord.z &&
    coord.z < roiExtents.z &&
    // on edge
    (coord.x == 0 || coord.y == 0 || coord.z == 0 || coord.x == roiExtents.x - 1 || coord.y == roiExtents.y - 1 ||
     coord.z == roiExtents.z - 1);
}

unsigned __device__ volumeOf(int3 expanse)
{
  return (unsigned)(expanse.x) * (unsigned)(expanse.y) * (unsigned)(expanse.z);
}



char4 __device__ selectObstructionForNeighbour(int nx, int ny, int nz, char4 currentClosest, float *currentDistSqr,
                                               __local char4 *localVoxels, float3 axisScaling)
{
  // We need to add 2 to the local size lookups to cater for the 1 voxel padding either side
  // of the local voxels. Similarly, we add one to the get_local_id() calls for the same
  // reason.
  const int linearIndexN = (get_local_id(0) + 1 + nx) + (get_local_id(1) + 1 + ny) * (get_local_size(0) + 2) +
                           (get_local_id(2) + 1 + nz) * (get_local_size(0) + 2) * (get_local_size(1) + 2);
  float3 scaledSeparation;

  // Get the neighbour's closest obstacle voxel coordinate.
  char4 neighbourObstacle = localVoxels[linearIndexN];

  // Localise the obstacle from the neighbour frame.
  neighbourObstacle = neighbourObstacle + make_char4(nx, ny, nz, 0);

  // Use neighbourObstacle if:
  // 1. It is an obstacle (w == 1)
  // 2. It is closer than currentClosest or currentClosest is not an obstacle (w == 0).
  scaledSeparation = make_float3(neighbourObstacle.x, neighbourObstacle.y, neighbourObstacle.z) * axisScaling;
  float neighbourObstacleRangeSqr = dot(scaledSeparation, scaledSeparation);
  // Set up for item 2 above. We set neighbourObstacleRangeSqr to currentDistSqr when neighbourObstacleRangeSqr is
  // out of range.
  const bool replaceCurrent =
    neighbourObstacle.w && (currentClosest.w == 0 || neighbourObstacleRangeSqr < *currentDistSqr);

  if (replaceCurrent)
  {
    *currentDistSqr = neighbourObstacleRangeSqr;
    // Neighbour has a closer, in range obstacle than currentClosest.
    return neighbourObstacle;
  }

  return currentClosest;
}


void __device__ calculateFacePadding(int3 workingVoxelExtents, int3 outerRegionPadding, __local uint *faceVolumeSizes,
                                     __local int3 *minFaceExtents, __local int3 *maxFaceExtents,
                                     __local int3 *faceVolumes)
{
  // Indexing here is tricky. We essentially have a working volume in the range:
  // [workingVoxelExtents - outerRegionPadding, workingVoxelExtents + outerRegionPadding)
  // However, we need to skip voxels in the range [0, workingVoxelExtents).
  //
  // We take the following approach:
  // - Define the padding region as the voxels outside of workingVoxelExtents.
  // - Split the padding region into 6 faces.
  //  - Bottom:
  //    [ -outerRegionPadding.x, workingVoxelExtents.x + outerRegionPadding.x )
  //    [ -outerRegionPadding.y, workingVoxelExtents.y + outerRegionPadding.y )
  //    [ -outerRegionPadding.z, 0                                            )
  //  - Top:
  //    [ -outerRegionPadding.x, workingVoxelExtents.x + outerRegionPadding.x )
  //    [ -outerRegionPadding.y, workingVoxelExtents.y + outerRegionPadding.y )
  //    [ workingVoxelExtents.z, workingVoxelExtents.z + outerRegionPadding.z )
  //  - Back:
  //    [ -outerRegionPadding.x, workingVoxelExtents.x + outerRegionPadding.x )
  //    [ -outerRegionPadding.y, 0                                            )
  //    [ 0,                     workingVoxelExtents.z                        )
  //  - Front:
  //    [ -outerRegionPadding.x, workingVoxelExtents.x + outerRegionPadding.x )
  //    [ workingVoxelExtents.y, workingVoxelExtents.y + outerRegionPadding.y )
  //    [ 0,                     workingVoxelExtents.z                        )
  //  - Left:
  //    [ -outerRegionPadding.x, 0                     )
  //    [ 0,                     workingVoxelExtents.y )
  //    [ 0,                     workingVoxelExtents.z )
  //  - Right:
  //    [ workingVoxelExtents.x, workingVoxelExtents.x + outerRegionPadding.x )
  //    [ 0,                     workingVoxelExtents.y                        )
  //    [ 0,                     workingVoxelExtents.z                        )
  //
  // Other ideas to consider:
  // https://en.wikipedia.org/wiki/Net_(polyhedron)

  enum Face
  {
    FaceBottom,
    FaceTop,
    FaceBack,
    FaceFront,
    FaceLeft,
    FaceRight,
  };

  // Cacluate which face the index falls into. For this we first calculate all the face volumes
  // Calculate volumes into local memory to share in the work group.
  if (get_local_id(0) < 6)
  {
    switch (get_local_id(0))
    {
    case FaceBottom:
      minFaceExtents[FaceBottom] = -outerRegionPadding;
      maxFaceExtents[FaceBottom] =
        make_int3(workingVoxelExtents.x + outerRegionPadding.x, workingVoxelExtents.y + outerRegionPadding.y, 0);
      break;
    case FaceTop:
      minFaceExtents[FaceTop] = make_int3(-outerRegionPadding.x, -outerRegionPadding.y, workingVoxelExtents.z);
      maxFaceExtents[FaceTop] = workingVoxelExtents + outerRegionPadding;
      break;
    case FaceBack:
      minFaceExtents[FaceBack] = make_int3(-outerRegionPadding.x, -outerRegionPadding.y, 0);
      maxFaceExtents[FaceBack] = make_int3(workingVoxelExtents.x + outerRegionPadding.x, 0, workingVoxelExtents.z);
      break;
    case FaceFront:
      minFaceExtents[FaceFront] = make_int3(-outerRegionPadding.x, workingVoxelExtents.y, 0);
      maxFaceExtents[FaceFront] = make_int3(workingVoxelExtents.x + outerRegionPadding.x,
                                            workingVoxelExtents.y + outerRegionPadding.y, workingVoxelExtents.z);
      break;
    case FaceLeft:
      minFaceExtents[FaceLeft] = make_int3(-outerRegionPadding.x, 0, 0);
      maxFaceExtents[FaceLeft] = make_int3(0, workingVoxelExtents.y, workingVoxelExtents.z);
      break;
    case FaceRight:
      minFaceExtents[FaceRight] = make_int3(workingVoxelExtents.x, 0, 0);
      maxFaceExtents[FaceRight] =
        make_int3(workingVoxelExtents.x + outerRegionPadding.x, workingVoxelExtents.y, workingVoxelExtents.z);
      break;
    default:
      break;
    }

    faceVolumes[get_local_id(0)] = maxFaceExtents[get_local_id(0)] - minFaceExtents[get_local_id(0)];
    faceVolumeSizes[get_local_id(0)] = volumeOf(faceVolumes[get_local_id(0)]);
    // printf("[%d] %d %d %d to %d %d %d\t=>(%d %d %d)\n", get_local_id(0),
    //   minFaceExtents[get_local_id(0)].x, minFaceExtents[get_local_id(0)].y, minFaceExtents[get_local_id(0)].z,
    //   maxFaceExtents[get_local_id(0)].x, maxFaceExtents[get_local_id(0)].y, maxFaceExtents[get_local_id(0)].z,
    //   faceVolumes[get_local_id(0)].x, faceVolumes[get_local_id(0)].y, faceVolumes[get_local_id(0)].z);
  }

  barrier(CLK_LOCAL_MEM_FENCE);
}


int3 __device__ resolvePaddingVoxelIndex(uint index, int3 workingVoxelExtents, int3 outerRegionPadding,
                                         __local uint *faceVolumeSizes, __local int3 *minFaceExtents,
                                         __local int3 *maxFaceExtents, __local int3 *faceVolumes)
{
  int face = -1;
  uint faceIndex1D = index;
  for (int i = 0; i < 6; ++i)
  {
    if (face == -1)
    {
      if (faceIndex1D < faceVolumeSizes[i])
      {
        face = i;
      }
      else
      {
        faceIndex1D -= faceVolumeSizes[i];
      }
    }
  }

  if (face < 0)
  {
    // Out of range. Should have already been culled.
    printf("No face found on thread %u\n", index);
    return make_int3(0, 0, 0);
  }

  // Now generate an index into the face region.
  // maxFaceExtents is exclusive, so use >=
  if (faceIndex1D >= volumeOf(maxFaceExtents[face] - minFaceExtents[face]))
  {
    printf("Face index out of range on thread %u\n", index);
    return make_int3(0, 0, 0);
  }

  // Convert the 1D face index into a 3D index in the face volume.
  int3 faceIndex3;
  faceIndex3.z = faceIndex1D / (faceVolumes[face].x * faceVolumes[face].y);
  const uint faceIndex1D2 = faceIndex1D - faceIndex3.z * faceVolumes[face].x * faceVolumes[face].y;
  faceIndex3.y = faceIndex1D2 / faceVolumes[face].x;
  faceIndex3.x = faceIndex1D2 % faceVolumes[face].x;

  // if (index == 144)
  // {
  //   printf("[%u]: F[%d]: if[%u], if3[%d %d %d] => [%d %d %d]\n", index, face, faceIndex1D,
  //       faceIndex3.x, faceIndex3.y, faceIndex3.z,
  //       faceIndex3.x + minFaceExtents[face].x,
  //       faceIndex3.y + minFaceExtents[face].y,
  //       faceIndex3.z + minFaceExtents[face].z);
  // }

  // Offset by the minFaceExtents for the face.
  return faceIndex3 + minFaceExtents[face];
}


inline float __device__ calcTimeVal(float limit, float origin, float direction)
{
  return (limit - origin) * direction;
}


inline bool __device__ rangesOverlap(float2 range1, float2 range2, float2 *overlap)
{
  overlap->x = max(range1.x, range2.x);
  overlap->y = min(range1.y, range2.y);
  return range1.x <= range2.y && range2.x <= range1.y;
}


int3 __device__ findClosestRegionVoxel(int3 voxelIndex3, int3 voxelExtents)
{
  // Convert to a ray/box interection test.

  // Define the box. Technically the box ranges from (0,0,0) to voxelExtents. However, when converting
  // to integer results, we'll get results at the voxel extents. We avoid this by bringing in the box edges
  // by half a voxel.
  const float3 box[2] = { make_float3(0.5f, 0.5f, 0.5f), convert_float3(voxelExtents) - 0.5f };
  // Define the ray as starting at the centre of the source voxel.
  const float3 origin = convert_float3(voxelIndex3) + make_float3(0.5f, 0.5f, 0.5f);
  // Tagret the centre of the box.
  const float3 end = 0.5f * convert_float3(voxelExtents);
  const float3 direction = end - origin;
  const float3 inv_direction = make_float3(1.0f, 1.0f, 1.0f) / direction;
  const int3 sign = make_int3(inv_direction.x < 0 ? 1 : 0, inv_direction.y < 0 ? 1 : 0, inv_direction.z < 0 ? 1 : 0);

  float2 hitTimes;
  float2 tx, ty, tz;
  bool validIntersection = true;

  // Perform various segment overlap calculations.
  tx.x = calcTimeVal(box[sign.x].x, origin.x, inv_direction.x);
  tx.y = calcTimeVal(box[1 - sign.x].x, origin.x, inv_direction.x);
  ty.x = calcTimeVal(box[sign.y].y, origin.y, inv_direction.y);
  ty.y = calcTimeVal(box[1 - sign.y].y, origin.y, inv_direction.y);
  tz.x = calcTimeVal(box[sign.z].z, origin.z, inv_direction.z);
  tz.y = calcTimeVal(box[1 - sign.z].z, origin.z, inv_direction.z);

  validIntersection = validIntersection && rangesOverlap(tx, ty, &hitTimes);
  validIntersection = validIntersection && rangesOverlap(hitTimes, tz, &hitTimes);

  const float hitTime = hitTimes.x >= 0 ? hitTimes.x : hitTimes.y;
  return validIntersection ? convert_int3(origin + hitTime * direction) : voxelIndex3;
}


inline __device__ uint obstructionToVoxel(char4 obstruction)
{
#if __ENDIAN_LITTLE__
  return (uint)((uchar)obstruction.x) | ((uchar)obstruction.y << 8) | ((uchar)obstruction.z << 16) |
         ((uchar)obstruction.w << 24);
#else   // __ENDIAN_LITTLE__
  return (uint)((uchar)obstruction.x << 24) | ((uchar)obstruction.y << 16) | ((uchar)obstruction.z << 8) |
         ((uchar)obstruction.w);
#endif  // __ENDIAN_LITTLE__
}


inline __device__ char4 voxelToObstruction(uint voxel)
{
#if __ENDIAN_LITTLE__
  return make_char4((char)((voxel)&0xFFu), (char)((voxel >> 8) & 0xFFu), (char)((voxel >> 16) & 0xFFu),
                    (char)((voxel >> 24) & 0xFFu));
#else   // __ENDIAN_LITTLE__
  return make_char4((char)((voxel >> 24) & 0xFFu), (char)((voxel >> 16) & 0xFFu), (char)((voxel >> 8) & 0xFFu),
                    (char)((voxel)&0xFFu));
#endif  // __ENDIAN_LITTLE__
}



bool __device__ updateVoxelObstructionCas(int3 voxelIndex, __global voxel_type *voxel, char4 newObstruction,
                                          float3 axisScaling)
{
  uint reference_value, new_value;
  __global voxel_type *voxel_ptr;

  // Prepare the new value we may write.
  new_value = obstructionToVoxel(newObstruction);
  voxel_ptr = voxel;

  float3 offset = make_float3(newObstruction.x, newObstruction.y, newObstruction.z) * axisScaling;
  const float distanceToObstacleSqr = dot(offset, offset);

  const int iterationLimit = 500;
  int iteration = 0;
  bool needsUpdate = false, updated = false;
  // Begin the contended write loop:
  do
  {
    // Cache the current value as a reference value.
    reference_value = gputilAtomicLoadU32(voxel_ptr);

    // Evaluate the range to the voxel currently considered the closest obstruction for the target voxel.
    // No need to apply voxelResolution as we are making relative comparisons.
    const char4 ref = voxelToObstruction(reference_value);
    offset = make_float3(ref.x, ref.y, ref.z) * axisScaling;

    const bool existing_obstruction = ref.w != 0;

    // Apply axis scaling to the distance calculation.
    const float currentDistSqr = dot(offset, offset);

    // Only important when the new obstruction is closer than the current one or the current is not an obstruction.
    if (distanceToObstacleSqr < currentDistSqr || !existing_obstruction)
    {
      // Attempt to write to the target location. This is done with contention, so we use
      // atomics to test for success. On failure we'll iterate again until we hit the iteration
      // limit.
      needsUpdate = !gputilAtomicCasU32(voxel_ptr, reference_value, new_value);
      updated = !needsUpdate;
    }
    else
    {
      // No need to touch the voxel. Existing value is closer.
      needsUpdate = false;
    }
  } while (needsUpdate && ++iteration < iterationLimit);

#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
  if (iteration == iterationLimit)
  {
    printf("%u excessive voxel update iterations (%d %d %d).\n", get_global_id(0), voxelIndex.x, voxelIndex.y,
           voxelIndex.z);
  }
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS

  return updated;
}


/// Load voxels into local memory for propagation kernels.
void __device__ loadPropagationLocalVoxels(__global char4 *srcVoxels, __local char4 *localVoxels, int3 voxelExtents,
                                           int3 globalWorkItem, int3 localWorkItem, int3 localExpanse)
{
  const int3 localGroupExpanse = localExpanse;
  // We pad the local memory by one voxel around the region expanse. This is to fetch data from neighbouring regions.
  const int3 localMemExpanse = localGroupExpanse + make_int3(2, 2, 2);
  const uint totalLoad = volumeOf(localMemExpanse);
  const char4 unobstructedVoxel = make_char4(0, 0, 0, 0);

#if VALIDATE_LOCAL_MEM_LOAD
  const char4 localMemClearValue = make_char4(1, 2, 3, 4);
  const int3 debugGroup = make_int3(0, 0, 0);
  const int3 debugThread = make_int3(0, 0, 0);
  // if (isLocalThread(0, 0, 0) && isInGroup(debugGroup.x, debugGroup.y, debugGroup.z))
  if (isGlobalThread(debugThread.x, debugThread.y, debugThread.z))
  {
    // Validate we touch all local mem required.
    for (uint i = 0; i < volumeOf(localMemExpanse); ++i)
    {
      localVoxels[i] = localMemClearValue;
    }
  }
  barrier(CLK_LOCAL_MEM_FENCE);
#endif  // VALIDATE_LOCAL_MEM_LOAD

  // if (isGlobalThread(0, 0, 0))
  // {
  //   printf("voxelExt: %d %d %d, localExp %d %d %d, localMem %d %d %d\n",
  //     voxelExtents.x, voxelExtents.y, voxelExtents.z,
  //     localExpanse.x, localExpanse.y, localExpanse.z,
  //     localMemExpanse.x, localMemExpanse.y, localMemExpanse.z);
  // }

  for (uint i = 0; i < totalLoad; i += volumeOf(localGroupExpanse))
  {
    // Cache voxels into local memory.
    // Work out the local source coordinate.
    // Note the linear index will normally be out of range in the last iteration as the
    // padded 3D expanse doesn't nicely match the local size.
    // There is also an oddity in the linearisation. We use the localGroupExpanse to generate a linear
    // index (matching the work group size) then convert that to a 3D index in the localMemExpanse,
    // which includes padding. We then iterate and get additional voxels, essentially accounting for
    // padding. Odd, but it works out.
    uint localLinearIndex = i + localWorkItem.x + localWorkItem.y * localGroupExpanse.x +
                            localWorkItem.z * localGroupExpanse.x * localGroupExpanse.y;

    // Convert the linear index into a 3D index into the padded local expanse.
    int3 localSrcCoord = linearIndexToCoord(localLinearIndex, localMemExpanse);
    // Convert the localSrcCoord into the unpadded coordinates. Remember, we only need pad one voxel from neighbouring
    // regions as we only propagate one voxel at a time.
    localSrcCoord -= make_int3(1, 1, 1);

    // Convert the local coordinate to a global voxel coordinate.
    // This may under of over flow.
    int3 globalSrcCoord =
      make_int3(localSrcCoord.x + get_group_id(0) * localExpanse.x, localSrcCoord.y + get_group_id(1) * localExpanse.y,
                localSrcCoord.z + get_group_id(2) * localExpanse.z);

    // Convert the global 3D voxel coordinate into a linear value. It will come out as -1
    // when the global coordinate is out of range.
    int globalLinearSrcIndex = getGlobalVoxelIndex(globalSrcCoord, voxelExtents);

    // Now make the load into local memory when the target index is in range.
    if (localLinearIndex < volumeOf(localMemExpanse))
    {
      // Cache the range of the global voxel, but assign -1 when the global voxel is out of range.
      localVoxels[localLinearIndex] = (globalLinearSrcIndex >= 0) ? srcVoxels[globalLinearSrcIndex] : unobstructedVoxel;
    }

    barrier(CLK_LOCAL_MEM_FENCE);
  }

#if VALIDATE_LOCAL_MEM_LOAD
  if (isGlobalThread(debugThread.x, debugThread.y, debugThread.z))
  {
    // Validate we touch all local mem required.
    bool touchedAllMem = true;
    for (uint i = 0; i < volumeOf(localMemExpanse); ++i)
    {
      if (localVoxels[i].x == localMemClearValue.x && localVoxels[i].y == localMemClearValue.y &&
          localVoxels[i].z == localMemClearValue.z && localVoxels[i].w == localMemClearValue.w)
      {
        printf("Did not touch %u\n", i);
        touchedAllMem = false;
      }
    }

    if (!touchedAllMem)
    {
      printf("Did not touch all local memory\n");
    }
  }
  barrier(CLK_LOCAL_MEM_FENCE);
#endif  // VALIDATE_LOCAL_MEM_LOAD
}

#endif // ROI_RANGE_FILL_BASE_CL

//-----------------------------------------------------------------------------
// Kernels
//-----------------------------------------------------------------------------

/// Build obstacle information into workingVoxels. We store the voxel coordinates in the GPU global group and
/// w = 1 for occupied voxels, zero for free. Each voxel in workingVoxels directly matches the dimensions of the
/// space we are to calculate clearance values for. This will always match directly with a number of OccupancyMap
/// regions in CPU.
///
/// This kernel does not consider any voxels outside the regions we are calculating clearance values for. That
/// occurs in the @c seedFromOuterRegions() kernel below.
///
/// @par Invocation
/// In layer batches, one thread per X/Z coordinate, processing zbatch items in Z.
///
/// @param cornerVoxelKey Key for the lower extents corner of the global work group. All other GPU threads can resolve
/// their key by
///   adjusting this key using their 3D global ID.
/// @param workingVoxels Identifies the offset to the nearest obstructed voxel in voxel units.
///   The w coordinate is zero if there is no obstructed in range and the voxel itself isn't and obstructed.
///   The w coordinate is 1 if there is an obstructed to consider.
__kernel void seedRegionVoxels(__global struct GpuKey *cornerVoxelKey, __global VOXEL_TYPE *voxelOccupancy,
                                          __global char4 *workingVoxels, __global int3 *regionKeysGlobal,
                                          __global ulong *regionMemOffsetsGlobal, uint regionCount,
                                          int3 regionVoxelDimensions, int3 workingVoxelExtents,
                                          float occupancyThresholdValue, uint flags, int zbatch,
                                          float sub_voxel_filter_scale)
{
  int3 effectiveGlobalId = make_int3(get_global_id(0), get_global_id(1), get_global_id(2) * zbatch);
  int3 voxelRegion;
  uint regionVoxelOffset;
  regionsInitCurrent(&voxelRegion, &regionVoxelOffset);

  struct GpuKey voxelKey = *cornerVoxelKey;

  // Offset the voxel key.
  moveKeyAlongAxis(&voxelKey, 0, effectiveGlobalId.x, &regionVoxelDimensions);
  moveKeyAlongAxis(&voxelKey, 1, effectiveGlobalId.y, &regionVoxelDimensions);
  moveKeyAlongAxis(&voxelKey, 2, effectiveGlobalId.z, &regionVoxelDimensions);

  for (int z = 0; z < zbatch; ++z)
  {
    if (effectiveGlobalId.x < (uint)workingVoxelExtents.x && effectiveGlobalId.y < (uint)workingVoxelExtents.y &&
        effectiveGlobalId.z < (uint)workingVoxelExtents.z)
    {
      // Find the region for voxelKey
      if (regionsResolveRegion(&voxelKey, &voxelRegion, &regionVoxelOffset, regionKeysGlobal, regionMemOffsetsGlobal,
                               regionCount, sizeof(*voxelOccupancy)))
      {
        // Found the region memory.
        // Index into voxels
        const uint vidx = regionVoxelOffset + voxelKey.voxel[0] + voxelKey.voxel[1] * regionVoxelDimensions.x +
                          voxelKey.voxel[2] * regionVoxelDimensions.x * regionVoxelDimensions.y;
        // Index into workingVoxels.
        const uint widx = effectiveGlobalId.x + effectiveGlobalId.y * workingVoxelExtents.x +
                          effectiveGlobalId.z * workingVoxelExtents.x * workingVoxelExtents.y;
        const VOXEL_TYPE occupancy = voxelOccupancy[vidx];
        const bool isObstruction = isOccupied(&occupancy, occupancyThresholdValue, flags, sub_voxel_filter_scale);
        workingVoxels[widx] = make_char4(0, 0, 0, (isObstruction) ? 1 : 0);
      }
    }

    // Move to the next Z item.
    ++effectiveGlobalId.z;
    moveKeyAlongAxis(&voxelKey, 2, 1, &regionVoxelDimensions);
  }
}


/// Seeds obstacle information taken from voxels outside of the region of interest. Each obstructing voxels
/// finds the voxel inside the working region closest to itself and attempts to push propagate into this voxel.
/// An obstacle voxel can only propagate into the working region if:
/// - It is within the search range of the closest working voxel.
/// - It is closer than any previously recorded voxel in the working voxel.
///
/// This process raises memory contention so obstucting voxels are popagated using compare and swap semantics.
///
/// Each element in @p workingVoxels stores the 3D coordinates to the nearest obstructing voxel in XYZ and either
/// 1 (obstacle) or 0 (no obstacles) in W. The @p workingVoxels array has dimensions of
/// <tt>(regionVoxelDimensions.x, regionVoxelDimensions.y, regionVoxelDimensions.z) and the first entry 3D index is
/// (0, 0, 0). Note that the @c seedFromOuterRegions() kernel will incorporate data from oustide the @p workingVoxels
/// so indexing on each axis may be out of range, either negative or greater than @p regionVoxelDimensions.
///
/// Source data from CPU are held in @p voxelOccupancy (occupancy values) and referenced via @p regionKeysGlobal and
/// @p regionMemOffsetsGlobal using each voxel's @c GpuKey. Each voxel can map to CPU coordinates by offsetting
/// @p cornerVoxel by its local coordiantes in @p workingVoxels. The region part of @c GpuKey is matched against
/// an element of @p regionkeysGlobal. This index is then used to lookup @p regionMemOffsetGlobal from which we
/// get an offset into @p voxelOccupancy. Finally, we convert the voxel part of @c GpuKey into a 1D index which is
/// added to the offset into @p voxelOccupancy. (Needs diagram)
///
/// @param cornerVoxelKey Key for the lower extents corner of the global work group. All other GPU threads can resolve
///   their key by adjusting this key using their 3D global ID.
/// @param voxelOccupancy Voxel occupancy data from the CPU. Indexing is done in conjunction with @p regionKeysGlobal
///    and @p regionMemOffsetGlobal.
/// @param workingVoxels Represents the region of interest and the region(s) for which we will export calculated
///   clearance values.
/// @param regionKeysGlobal Array of region keys corresponding to entries in @p regionMemOffsetsGlobal. See above.
/// @param regionMemOffsetsGlobal Array of index offsets into @c voxelOccupancy marking the start of each region.
/// @param regionCount Number of elements in both @p regionKeysGlobal and @p regionMemOffsetsGlobal.
/// @param regionVoxelDimensions The voxel dimensions of a single region in CPU.
/// @param workingVoxelExtents The dimensions of @p workingVoxels.
/// @param outerRegionPadding The amount of padding required around @c regionVoxelDimensions to correctly consider
///   external obstacles.
/// @param occupancyThresholdValue Requires @p voxelOccupancy threshold value to consider a voxel occupied. Any larger
///   value is considered occupied except for @c INFINITY, which is considered unknown.
/// @param flags Flags modifying behaviour. See @c QF_ flags above.
/// @param zbatch Number of items each thread should process.
__kernel void seedFromOuterRegions(__global struct GpuKey *cornerVoxelKey,
                                              __global VOXEL_TYPE *voxelOccupancy, __global voxel_type *workingVoxels,
                                              __global int3 *regionKeysGlobal, __global ulong *regionMemOffsetsGlobal,
                                              uint regionCount, int3 regionVoxelDimensions, int3 workingVoxelExtents,
                                              int3 outerRegionPadding, float3 axisScaling,
                                              float occupancyThresholdValue, uint flags, int batch,
                                              float sub_voxel_filter_scale)
{
  local uint faceVolumeSizes[6];
  local int3 minFaceExtents[6];
  local int3 maxFaceExtents[6];
  local int3 faceVolumes[6];

  // Work out whether this thread indexes a valid padding voxel.
  const unsigned voxelCount = volumeOf(workingVoxelExtents + 2 * outerRegionPadding) - volumeOf(workingVoxelExtents);

  int3 voxelRegion;
  uint regionVoxelOffset;
  regionsInitCurrent(&voxelRegion, &regionVoxelOffset);
  calculateFacePadding(workingVoxelExtents, outerRegionPadding, faceVolumeSizes, minFaceExtents, maxFaceExtents,
                       faceVolumes);

  for (int i = 0; i < batch; ++i)
  {
    uint voxelIndex = get_global_id(0) * batch + i;
    if (voxelIndex >= voxelCount)
    {
      // printf("v out of range: %u < %u\n", voxelIndex, voxelCount);
      // Out of range.
      return;
    }

    // Resolve the padding voxel index relative to the workingVoxels region.
    const int3 workingIndex3 = resolvePaddingVoxelIndex(voxelIndex, workingVoxelExtents, outerRegionPadding,
                                                        faceVolumeSizes, minFaceExtents, maxFaceExtents, faceVolumes);

    // Validate the indexing. Failure shows a logic error.
    if (workingIndex3.x < -outerRegionPadding.x || workingIndex3.y < -outerRegionPadding.y ||
        workingIndex3.z < -outerRegionPadding.z || workingIndex3.x >= workingVoxelExtents.x + outerRegionPadding.x ||
        workingIndex3.y >= workingVoxelExtents.y + outerRegionPadding.y ||
        workingIndex3.z >= workingVoxelExtents.z + outerRegionPadding.z ||
        (0 <= workingIndex3.x && workingIndex3.x < workingVoxelExtents.x && 0 <= workingIndex3.y &&
         workingIndex3.y < workingVoxelExtents.y && 0 <= workingIndex3.z && workingIndex3.z < workingVoxelExtents.z))
    {
      printf("Bad working index: %u => %d %d %d: region %d %d %d, padding %d %d %d\n", voxelIndex, workingIndex3.x,
             workingIndex3.y, workingIndex3.z, workingVoxelExtents.x, workingVoxelExtents.y, workingVoxelExtents.z,
             outerRegionPadding.x, outerRegionPadding.y, outerRegionPadding.z);
      return;
    }

    // Offset the voxel key.
    struct GpuKey voxelKey = *cornerVoxelKey;
    moveKeyAlongAxis(&voxelKey, 0, workingIndex3.x, &regionVoxelDimensions);
    moveKeyAlongAxis(&voxelKey, 1, workingIndex3.y, &regionVoxelDimensions);
    moveKeyAlongAxis(&voxelKey, 2, workingIndex3.z, &regionVoxelDimensions);

    // Find the region for voxelKey
    if (regionsResolveRegion(&voxelKey, &voxelRegion, &regionVoxelOffset, regionKeysGlobal, regionMemOffsetsGlobal,
                             regionCount, sizeof(*voxelOccupancy)))
    {
      // Found the region memory.
      // Index into voxels
      const uint vidx = regionVoxelOffset + voxelKey.voxel[0] + voxelKey.voxel[1] * regionVoxelDimensions.x +
                        voxelKey.voxel[2] * regionVoxelDimensions.x * regionVoxelDimensions.y;
      const VOXEL_TYPE occupancy = voxelOccupancy[vidx];
      const bool isObstruction =
        isOccupied(&occupancy, occupancyThresholdValue, flags, sub_voxel_filter_scale);

      if (isObstruction)
      {
        // printf("outer obstruction from key " KEY_F " @: %d %d %d\n",
        //   KEY_A(voxelKey),
        //   workingIndex3.x, workingIndex3.y, workingIndex3.z);

        // Find the edge voxel in the ROI closest to the source voxel.
        // Note: There are rounding errors on this, there are different behaviours depending on the ray direction.
        // This was most notable when testing a single region ROI with obstacles being sourced from the voxel layers
        // directly above and below the ROI. The pattern on the bottom was such that the nearest voxel was generally
        // offset by exactly 1 voxel along Z. The pattern on the top tended to have diagonal offsets.
        // This results in an effective down sampling and a loss of information. We cater for this below by targeting
        // neighbours as well as the edgeVoxel3.
        const int3 edgeVoxel3 = findClosestRegionVoxel(workingIndex3, workingVoxelExtents);

        // Exhaustive neighbours.
        for (int z = -1; z <= 1; ++z)
        {
          for (int y = -1; y <= 1; ++y)
          {
            for (int x = -1; x <= 1; ++x)
            {
              const int3 regionVoxel3 = edgeVoxel3 + make_int3(x, y, z);
              if (onROIEdge(regionVoxel3, workingVoxelExtents))
              {
                // Index into workingVoxels.
                const uint ridx = regionVoxel3.x + regionVoxel3.y * workingVoxelExtents.x +
                                  regionVoxel3.z * workingVoxelExtents.x * workingVoxelExtents.y;


                // Value to write; relative offset from regionVoxel3 to workingIndex3.
                char4 obstructionValue = make_char4(workingIndex3.x - regionVoxel3.x, workingIndex3.y - regionVoxel3.y,
                                                    workingIndex3.z - regionVoxel3.z,
                                                    1);  // Must be 1 for an obstruction.

                // printf("obstruction at %d %d %d => %d %d %d : %d %d %d %d\n", workingIndex3.x, workingIndex3.y,
                // workingIndex3.z,
                //   regionVoxel3.x, regionVoxel3.y, regionVoxel3.z,
                //   obstructionValue.x, obstructionValue.y, obstructionValue.z, obstructionValue.w);

                // Write into the voxel with contention.
                updateVoxelObstructionCas(workingIndex3, &workingVoxels[ridx], obstructionValue, axisScaling);
              }
            }
          }
        }
      }
    }
  }
}


/// Propagate obstacles range value from neighbouring voxels.
///
/// Invoking this kernel once will ensure every voxel looks at its immediate face, edge
/// and corner neighbours, and minimises its range value (where -1/negative is an irrelevant
/// range value). The range value of a neighbour has the range from the neighbour to the
/// voxel of interest added to it before being considered for update.
///
/// When called iteratively, this effectively calculated the range to the closest obstacle
/// voxel. The number of times this is invoked determines the maximum obstacle range.
///
/// The seedObstacleVoxels() kernel should be invoked before this kernel.
///
/// Local memory requirement:
/// - @p localVoxels group volume expanded by 2 around each axis yielding a 1 voxel padding.
///
/// @par Invocation
/// In layer batches, one thread per X/Z coordinate, processing zbatch items in Z.
__kernel void propagateObstacles(__global char4 *srcVoxels, __global char4 *dstVoxels, int3 voxelExtents,
                                 float searchRange, float3 axisScaling LOCAL_ARG(char4 *, localVoxels))
{
  LOCAL_MEM_ENABLE();
  LOCAL_VAR(char4 *, localVoxels,
            0);  // FIXME(KS): match size to CPU size. Won't actually have effect unless more locals are used.


  int3 effectiveGlobalId = make_int3(get_global_id(0), get_global_id(1), get_global_id(2));
  int3 effectiveLocalId = make_int3(get_local_id(0), get_local_id(1), get_local_id(2));
  const int3 effectiveLocalSize = make_int3(get_local_size(0), get_local_size(1), get_local_size(2));

  const uint voxelIndex =
    get_global_id(0) + get_global_id(1) * voxelExtents.x + get_global_id(2) * voxelExtents.x * voxelExtents.y;
  const bool validVoxel = voxelIndex < volumeOf(voxelExtents) && effectiveGlobalId.x < voxelExtents.x &&
                          effectiveGlobalId.y < voxelExtents.y && effectiveGlobalId.z < voxelExtents.z;

  // if (isGlobalThread(0, 0, 0))
  // {
  //   printf("axisScaling: %f %f %f\n", axisScaling.x, axisScaling.y, axisScaling.z);
  // }
  loadPropagationLocalVoxels(srcVoxels, localVoxels, voxelExtents, effectiveGlobalId, effectiveLocalId,
                             effectiveLocalSize);

  if (!validVoxel)
  {
    // Stop invalid voxel references here.
    return;
  }

  // Set range for an unobstructed voxel at the maximum range.
  char4 closestObstruction = srcVoxels[voxelIndex];
  const float3 currentScaledSeparation =
    make_float3(closestObstruction.x, closestObstruction.y, closestObstruction.z) * axisScaling;
  float currentClosestDistSqr = dot(currentScaledSeparation, currentScaledSeparation);

  // 2. Consider each neighbour range value and select the best result.
  for (int z = -1; z <= 1; ++z)
  {
    for (int y = -1; y <= 1; ++y)
    {
      for (int x = -1; x <= 1; ++x)
      {
        if (x || y || z)  // Only do neighbours.
        {
          closestObstruction = selectObstructionForNeighbour(x, y, z, closestObstruction, &currentClosestDistSqr,
                                                             localVoxels, axisScaling);
        }
      }
    }
  }

  // 3. Update the voxel range.
  if (validVoxel)
  {
    dstVoxels[voxelIndex] = closestObstruction;
  }
}


/// Migrate from the working voxel data to the voxel clearance (distance) memory data.
///
/// @par Invocation
/// One thread per region voxel.
///
/// @param cornerVoxelKey Key for the lower extents corner of the global work group. All other GPU threads can resolve
/// their key by
///   adjusting this key using their 3D global ID.
/// @param voxelsMem Memory pointer to the memory block holding voxels. See notes in @ref voxelClearanceGpu
/// @param workingVoxels The flood fill map from @c propagateObstacles()
///   The w coordinate is zero if there is no obstructed in range and the voxel itself isn't and obstructed.
///   The w coordinate is 1 if there is an obstructed to consider.
/// @param regionKeysGlobal Array of region keys matched to elements in @p regionMemOffsetsGlobal (see @ref
/// voxelClearanceGpu).
/// @param regionMemOffsetsGlobal Array of memory offsets into @p voxelsMem where to find each region (see @ref
/// voxelClearanceGpu).
/// @param regionCount Number of regions in @p regionKesyGlobal and @p regionMemOffsetsGlobal.
/// @param regionVoxelDim Number of voxels along each axis in a single region.
/// @param workingVoxelExtents Extents within @p workingVoxels for which we actually want to generate results. This is
///   the non-padded region.
/// @param searchRange The maximum search range for the overall query. Obstacles beyond this range are ignored.
/// @param voxelResolution Physical dimensions along each edge of each voxel.
__kernel void migrateResults(__global float *clearanceVoxels, __global char4 *workingVoxels,
                                        int3 regionVoxelDimensions, int3 workingVoxelExtents, float searchRange,
                                        float voxelResolution, float3 axisScaling, uint flags)
{
  // NOTE: Experimentation with invoking smaller global sizes was attempted, but to no performance benefit.
  // Tried:
  //  - Each worker thread processed a number of Z layers.
  //  - Each worker thread processed a whole X/Y layer.

  const int3 regionIndex3 = make_int3(get_global_id(0), get_global_id(1), get_global_id(2));
  if (0 <= regionIndex3.x && regionIndex3.x < regionVoxelDimensions.x && 0 <= regionIndex3.y &&
      regionIndex3.y < regionVoxelDimensions.y && 0 <= regionIndex3.z && regionIndex3.z < regionVoxelDimensions.z)
  {
    const int3 workingIndex3 = make_int3(regionIndex3.x, regionIndex3.y, regionIndex3.z);

    // if (isGlobalThread(0, 0, 0))
    // {
    //   // printf("axisScaling: %f %f %f\n", axisScaling.x, axisScaling.y, axisScaling.z);
    //   printf("regionIndex: %d %d %d, of %d %d %d\n", regionIndex3.x, regionIndex3.y, regionIndex3.z,
    //     regionVoxelDimensions.x, regionVoxelDimensions.y, regionVoxelDimensions.z);
    //   printf("workingIndex: %d %d %d of %d %d %d\n", workingIndex3.x, workingIndex3.y, workingIndex3.z,
    //     workingVoxelExtents.x, workingVoxelExtents.y, workingVoxelExtents.z);
    // }

    // Find working voxel index to read from.
    const uint widx = workingIndex3.x + workingIndex3.y * workingVoxelExtents.x +
                      workingIndex3.z * workingVoxelExtents.x * workingVoxelExtents.y;
    const char4 nearestObstacleOffset = workingVoxels[widx];
    const float3 nearestObstacle = make_float3(nearestObstacleOffset.x * voxelResolution * axisScaling.x,
                                               nearestObstacleOffset.y * voxelResolution * axisScaling.y,
                                               nearestObstacleOffset.z * voxelResolution * axisScaling.z);
    float clearance = (nearestObstacleOffset.w) ? length(nearestObstacle) : -1.0f;
    clearance = (clearance <= searchRange) ? clearance : -1.0f;

    // if (isGlobalThread(0, 0, 31))
    // {
    //   printf("[%u,%u,%u] : %d %d %d %d => %f\n",
    //     get_global_id(0), get_global_id(1), get_global_id(2),
    //     nearestObstacleOffset.x, nearestObstacleOffset.y, nearestObstacleOffset.z, nearestObstacleOffset.w,
    //     clearance);
    // }

    // Index to write to. Assume writing only to regionVoxelDimensions
    const uint vidx = regionIndex3.x + regionIndex3.y * regionVoxelDimensions.x +
                      regionIndex3.z * regionVoxelDimensions.x * regionVoxelDimensions.y;

    clearanceVoxels[vidx] = clearance;
  }
}

#ifndef ROI_RANGE_FILL_BASE_CL
#define ROI_RANGE_FILL_BASE_CL
#endif  // ROI_RANGE_FILL_BASE_CL

#ifdef SUB_VOXEL
#undef isOccupied
#undef seedRegionVoxels
#undef seedFromOuterRegions
#undef propagateObstacles
#undef migrateResults
#endif  // SUB_VOXEL

#undef VOXEL_TYPE

/// @}
