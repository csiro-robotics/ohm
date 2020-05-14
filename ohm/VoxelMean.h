// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELMEAN_H
#define VOXELMEAN_H

#include "MapCoord.h"

// Note: this header is included in GPU code.
// Because of this "OhmConfig.h" and <cmath> cannot be included here and you may need to include those first.

/// @defgroup voxelmean Voxel Mean Position
/// These functions are used to manipulate the voxel mean positioning voxel fields. @c VoxelMean allows a voxel to store
/// a position field offset from the centre of the voxel, thereby refining the precision of the voxel position.
///
/// To support voxel mean positioning, an @c OccupancyMap must first enable this feature by the constructor flag, or
/// by calling @c OccupancyMap::setVoxelMeanEnabled(true). This ensures a voxel layer is added to the map containing
/// to hold the @c VoxelMean data. This structure contains a quantised voxel mean coordinate pattern and a point count.
/// The voxel mean coordinate pattern is defined as follows:
/// - bits [0, 9]: voxel mean x coord
/// - bits [10, 19]: voxel mean y coord
/// - bits [20, 29]: voxel mean z coord
/// - bit 30: unused
/// - bit 31: marks voxel mean data as present
///
/// XYZ coordinates are assigned 10 bits each to store a quantised voxel mean coordinate. This coordinate is quantised
/// accross the range of 10-bits allowing for 1024 possible positions along each axis. If no voxel mean pattern is
/// present, then the voxel centre is given as the voxel position.
///
/// When samples are integrated into a voxel, the voxel mean position is calculated via a progressive average
/// combining the existing coordinate and the new sample value.
/// This is given as:
/// @code{.unparsed}
///   new_mean_coord = (point_count * current_mean_coord + sample) / (point_count + 1)
/// @endcode

#if !GPUTIL_DEVICE
#ifndef __device__
#define __device__
#endif  // __device__
#ifndef __host__
#define __host__
#endif  // __host__

namespace ohm
{
#define SUB_VOX_FUNC_PREFACE template <typename vec3, typename coord_real>
  using vec3 = glm::vec3;

  /// @ingroup voxelmean
  /// The data structure is used to hold the voxel mean mean coordinate. See @ref voxelmean for details on usage and the
  /// mean quantisation in @c coord .
  /// @todo Rename this and assodiated code as VoxelMean
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

// Voxel mean structure used on GPU. Must match the CPU structure, but the types are modified to support atomic operations.
struct VoxelMean
{
  atomic_uint coord;
  atomic_uint count;
};

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
  inline __device__ __host__ unsigned subVoxelUpdate(uint coord, uint point_count, vec3 voxel_local_coord,
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
    mean.x = (point_count * mean.x + voxel_local_coord.x) * one_on_count_plus_one;
    mean.y = (point_count * mean.y + voxel_local_coord.y) * one_on_count_plus_one;
    mean.z = (point_count * mean.z + voxel_local_coord.z) * one_on_count_plus_one;
    return subVoxelCoord(mean, resolution);
  }
#if !GPUTIL_DEVICE
}  // namespace ohm
#endif  // !GPUTIL_DEVICE

#endif  // VOXELMEAN_H
