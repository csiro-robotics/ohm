// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELMEANCOMPUTE_H
#define VOXELMEANCOMPUTE_H

// Note: Must include MapCoord.h before this header.

// Note: this header is included in GPU code.
// Because of this "OhmConfig.h" and <cmath> cannot be included here and you may need to include those first.

#if !GPUTIL_DEVICE
#ifndef __device__
#define __device__
#endif  // __device__
#ifndef __host__
#define __host__
#endif  // __host__

#define SUB_VOX_FUNC_PREFACE template <typename vec3, typename coord_real>
using vec3 = glm::vec3;

/// @ingroup voxelmean
/// The data structure used to hold the voxel mean coordinate and sample count. See @ref voxelmean for details on usage
/// and the mean quantisation in @c coord .
/// @todo Rename this and associated code as VoxelMean
struct VoxelMean
{
  uint32_t coord;  ///< Quantised voxel mean coordinate.
  uint32_t count;  ///< Number of samples used to generate the current @c coord value.
};

#else  // GPUTIL_DEVICE

#if !defined(COORD_REAL)
#define COORD_REAL
typedef float coord_real;
#endif  // !defined(COORD_REAL)

#if !defined(vec3)
typedef float3 vec3;
#endif  // !defined(vec3)

#define SUB_VOX_FUNC_PREFACE


// Voxel mean structure used on GPU. Must match the CPU structure, but the types are modified to support atomic
// operations.
typedef struct VoxelMean_t
{
  atomic_uint coord;
  atomic_uint count;
} VoxelMean;

#endif  // GPUTIL_DEVICE


/// @ingroup voxelmean
/// Convert @p voxel_local_coord into a voxel mean positioning pattern.
/// @param voxel_local_coord The new coordinate to store. This position should be within the voxel bounds relative
///   to the centre of the voxel; i.e., [-0.5 * resolution, 0.5 * resolution]. The position is clamped to the voxel
///   bounds.
/// @param resolution The length of each voxel cube edge.
/// @return The voxel mean pattern approximating @p voxel_local_coord.
SUB_VOX_FUNC_PREFACE
inline __device__ __host__ unsigned subVoxelCoord(vec3 voxel_local_coord, coord_real resolution)
{
  // We divide the voxel into a voxel_local_coord, 3D grid, then assign 1 bit per cell.
  const unsigned bits_per_axis = 10;
  const unsigned mean_positions = (1 << bits_per_axis) - 1;
  const unsigned used_bit = (1u << 31);
  const coord_real mean_resolution = resolution / (coord_real)mean_positions;  // NOLINT
  const coord_real offset = (coord_real)0.5 * resolution;                      // NOLINT

  int pos_x = pointToRegionCoord(voxel_local_coord.x + offset, mean_resolution);
  int pos_y = pointToRegionCoord(voxel_local_coord.y + offset, mean_resolution);
  int pos_z = pointToRegionCoord(voxel_local_coord.z + offset, mean_resolution);

  pos_x = (pos_x >= 0 ? (pos_x < (1 << bits_per_axis) ? pos_x : mean_positions) : 0);
  pos_y = (pos_y >= 0 ? (pos_y < (1 << bits_per_axis) ? pos_y : mean_positions) : 0);
  pos_z = (pos_z >= 0 ? (pos_z < (1 << bits_per_axis) ? pos_z : mean_positions) : 0);

  unsigned pattern = 0;
  pattern |= (unsigned)pos_x;                           // NOLINT
  pattern |= ((unsigned)pos_y << bits_per_axis);        // NOLINT
  pattern |= ((unsigned)pos_z << (2 * bits_per_axis));  // NOLINT
  pattern |= used_bit;
  return pattern;
}


/// @ingroup voxelmean
/// Convert a voxel mean pattern into a coordinate, relative to the voxel centre.
/// @param pattern The voxel mean pattern.
/// @param resolution The length of each voxel cube edge.
/// @return The unpacked coordinate, relative to the voxel centre, in the range [-0.5 * resolution, 0.5 resolution].
///   The result is (0, 0, 0) if @p pattern has yet to be used (bit-31 not set).
SUB_VOX_FUNC_PREFACE
inline __device__ __host__ vec3 subVoxelToLocalCoord(unsigned pattern, coord_real resolution)
{
  const unsigned bits_per_axis = 10;
  const unsigned mean_positions = (1 << bits_per_axis) - 1;
  const unsigned used_bit = (1u << 31);
  const coord_real mean_resolution = resolution / (coord_real)mean_positions;  // NOLINT
  const coord_real offset = (coord_real)0.5 * resolution;                      // NOLINT

  vec3 coord;  // NOLINT
  // NOLINTNEXTLINE
  coord.x = (used_bit) ? regionCentreCoord((int)(pattern & mean_positions), mean_resolution) - offset : 0;
  coord.y = (used_bit) ?
              // NOLINTNEXTLINE
              regionCentreCoord((int)((pattern >> bits_per_axis) & mean_positions), mean_resolution) - offset :
              0;
  coord.z = (used_bit) ?
              // NOLINTNEXTLINE
              regionCentreCoord((int)((pattern >> (2 * bits_per_axis)) & mean_positions), mean_resolution) - offset :
              0;
  return coord;
}


/// @ingroup voxelmean
/// Update the @c VoxelMean for a voxel adding @c voxel_local_coord to the coordinate. The @c voxel_local_coord
/// coordiate is sample coordinate to add relative to the voxel centre.
///
/// @param voxel The @c VoxelMean structure to update.
/// @param resolution The length of each voxel cube edge.
SUB_VOX_FUNC_PREFACE
inline __device__ __host__ unsigned subVoxelUpdate(unsigned coord, unsigned point_count, vec3 voxel_local_coord,
                                                   coord_real resolution)
{
  vec3 mean =
#if !GPUTIL_DEVICE
    subVoxelToLocalCoord<vec3>(coord, resolution)
#else   //  GPUTIL_DEVICE
    subVoxelToLocalCoord(coord, resolution)
#endif  //  GPUTIL_DEVICE
    ;

  const coord_real one_on_count_plus_one = (coord_real)1 / (coord_real)(point_count + 1);
  mean.x += (voxel_local_coord.x - mean.x) * one_on_count_plus_one;
  mean.y += (voxel_local_coord.y - mean.y) * one_on_count_plus_one;
  mean.z += (voxel_local_coord.z - mean.z) * one_on_count_plus_one;
  return subVoxelCoord(mean, resolution);
}

#endif  // VOXELMEANCOMPUTE_H
