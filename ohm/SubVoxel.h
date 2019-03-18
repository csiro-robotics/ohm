// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SUBVOXEL_H
#define SUBVOXEL_H

#include "MapCoord.h"

// Note: this header is included in GPU code.
// Because of this "OhmConfig.h" and <cmath> cannot be included here and you may need to include those first.

/// @defgroup subvoxel Sub-Voxel Positioning
/// These functions are used to manipulate the sub-voxel positioning voxel fields. Sub-voxel allows a voxel to store
/// a position field offset from the centre of the voxel, thereby refining the precision of the voxel position.
///
/// To support sub-voxel positioning, an @c OccupancyMap must first enable this feature by the constructor flag, or
/// by calling @c OccupancyMap::setSubVoxelsEnabled(true). This modifies the map's occupancy layer from
/// containing a single float value (occupancy value) to contain a float occupancy value and a 32-bit, unsigned integer
/// sub-voxel pattern. The sub-voxel pattern is defined as follows:
/// - bits [0, 9]: sub-voxel x coord
/// - bits [10, 19]: sub-voxel y coord
/// - bits [20, 29]: sub-voxel z coord
/// - bit 30: unused
/// - bit 31: marks sub-voxel data as present
///
/// XYZ coordinates are assigned 10 bits each to store a quantised sub-voxel coordinate. This coordinate is quantised
/// accross the range of 10-bits allowing for 1024 possible positions along each axis. If no sub-voxel pattern is
/// present, then the voxel centre is given as the voxel position.
///
/// When samples are integrated into a voxel, the sub-voxel position is calculated via a weighted sub of thenew sample
/// position and the existing sub-voxel position (or the voxel centre if no sub-voxel position has yet been recorded).
/// This is given as:
/// @code{.unparsed}
///   new_sub_voxel_coord = weighting * new_sample + (1.0 - weighting) * current_sub_voxel_position
/// @endcode
///
/// The weighting is given by @c OccupancyMap::subVoxelWeighting(), but is expected to be low (e.g., 0.1).

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

#else  // GPUTIL_DEVICE

#if !defined(COORD_REAL)
#define COORD_REAL
typedef float coord_real;
#endif  // !defined(COORD_REAL)

#if !defined(vec3)
typedef float3 vec3;
#endif  // !defined(vec3)

#define SUB_VOX_FUNC_PREFACE

#endif  // GPUTIL_DEVICE

  /// @ingroup subvoxel
  /// Convert @p voxel_local_coord into a sub-voxel positioning pattern.
  /// @param voxel_local_coord The new coordinate to store. This position should be within the voxel bounds relative
  ///   to the centre of the voxel; i.e., [-0.5 * resolution, 0.5 * resolution]. The position is clamped to the voxel
  ///   bounds.
  /// @param resolution The length of each voxel cube edge.
  /// @return The sub-voxel pattern approximating @p voxel_local_coord.
  SUB_VOX_FUNC_PREFACE
  inline __device__ __host__ unsigned subVoxelCoord(vec3 voxel_local_coord, coord_real resolution)
  {
    // We divide the voxel into a voxel_local_coord, 3D grid, then assign 1 bit per cell.
    const unsigned bits_per_axis = 10;
    const unsigned sub_voxel_positions = (1 << bits_per_axis) - 1;
    const unsigned used_bit = (1u << 31);
    const coord_real sub_voxel_resolution = resolution / (coord_real)sub_voxel_positions;
    const coord_real offset = (coord_real)0.5 * resolution;

    int pos_x = pointToRegionCoord(voxel_local_coord.x + offset, sub_voxel_resolution);
    int pos_y = pointToRegionCoord(voxel_local_coord.y + offset, sub_voxel_resolution);
    int pos_z = pointToRegionCoord(voxel_local_coord.z + offset, sub_voxel_resolution);

    pos_x = (pos_x >= 0 ? (pos_x < (1 << bits_per_axis) ? pos_x : sub_voxel_positions) : 0);
    pos_y = (pos_y >= 0 ? (pos_y < (1 << bits_per_axis) ? pos_y : sub_voxel_positions) : 0);
    pos_z = (pos_z >= 0 ? (pos_z < (1 << bits_per_axis) ? pos_z : sub_voxel_positions) : 0);

    unsigned pattern = 0;
    pattern |= (unsigned)pos_x;
    pattern |= ((unsigned)pos_y << bits_per_axis);
    pattern |= ((unsigned)pos_z << (2 * bits_per_axis));
    pattern |= used_bit;
    return pattern;
  }


  /// @ingroup subvoxel
  /// Convert a sub-voxel pattern into a coordinate, relative to the voxel centre.
  /// @param pattern The sub-voxel pattern.
  /// @param resolution The length of each voxel cube edge.
  /// @return The unpacked coordinate, relative to the voxel centre, in the range [-0.5 * resolution, 0.5 resolution].
  ///   The result is (0, 0, 0) if @p pattern has yet to be used (bit-31 not set).
  SUB_VOX_FUNC_PREFACE
  inline __device__ __host__ vec3 subVoxelToLocalCoord(unsigned pattern, coord_real resolution)
  {
    const unsigned bits_per_axis = 10;
    const unsigned sub_voxel_positions = (1 << bits_per_axis) - 1;
    const unsigned used_bit = (1u << 31);
    const coord_real sub_voxel_resolution = resolution / (coord_real)sub_voxel_positions;
    const coord_real offset = (coord_real)0.5 * resolution;

    vec3 coord;
    coord.x = (used_bit) ? regionCentreCoord((int)(pattern & sub_voxel_positions), sub_voxel_resolution) - offset : 0;
    coord.y =
      (used_bit) ?
        regionCentreCoord((int)((pattern >> bits_per_axis) & sub_voxel_positions), sub_voxel_resolution) - offset :
        0;
    coord.z = (used_bit) ?
                regionCentreCoord((int)((pattern >> (2 * bits_per_axis)) & sub_voxel_positions), sub_voxel_resolution) -
                  offset :
                0;
    return coord;
  }


  /// @ingroup subvoxel
  /// Update a sub-voxel pattern using a weighted sum of the existing sub-voxel coordinate an the new
  /// @p voxel_local_coord.
  ///
  /// @param pattern The existing sub-voxel pattern.
  /// @param resolution The length of each voxel cube edge.
  /// @param weighting The weighting for the new @p voxel_coord_local [0, 1].
  SUB_VOX_FUNC_PREFACE
  inline __device__ __host__ unsigned subVoxelUpdate(unsigned initial_pattern, vec3 voxel_local_coord,
                                                     coord_real resolution, coord_real weighting)
  {
    vec3 old_local =
#if !GPUTIL_DEVICE
      subVoxelToLocalCoord<vec3>(initial_pattern, resolution)
#else   //  GPUTIL_DEVICE
    subVoxelToLocalCoord(initial_pattern, resolution)
#endif  //  GPUTIL_DEVICE
      ;
    old_local.x *= (1.0f - weighting);
    old_local.y *= (1.0f - weighting);
    old_local.z *= (1.0f - weighting);
    voxel_local_coord.x *= weighting;
    voxel_local_coord.y *= weighting;
    voxel_local_coord.z *= weighting;
    return subVoxelCoord(old_local + voxel_local_coord, resolution);
  }

  /// @ingroup subvoxel
  /// Sub-voxel occupancy enhancement. This function checks the sub-voxel positioning in order to augment the occupancy
  /// test for a single voxel adding a level of noise and outlier tolerance.
  ///
  /// This function is used on voxels which are indicated to be occupied by their occupancy probability value as a
  /// second step occupancy validation. The sub-voxel occupancy test can override the result and mark the voxel as
  /// free instead.
  ///
  /// An occupied voxel with a sub-voxel position is considered free instead of occupied when the sub-voxel position
  /// falls outside of the sphere scaled within the inside the voxel. This is shown below in ASCII art. The sphere size
  /// depends on @p sphere_scale scaled [0, 1] such at a scale of 1 is the largest sphere which fits within the
  /// voxel cube.
  ///
  /// @code{.unparsed}
  /// Sub-voxel occupancy filter:
  /// +-----------+
  /// |x  _____   |
  /// |  /     \  |
  /// | /       \ |
  /// ||         ||
  /// ||         ||
  /// | \       / |
  /// |  \_____/  |
  /// +-----------+
  ///   - Outer box defines the voxel
  ///   - Inner "circle" represents the filtering sphere.
  ///   - 'x' shows a sub-voxel position which would fail the sub-voxel occupancy filter, converting the voxel to free.
  /// @endcode
  ///
  /// The assumption here is that such voxel positions are generated by noise and measurement errors and a truly
  /// occupied voxel will generate multiple sample positions in a voxel, which will average out to fall inside the
  /// aforementioned sphere. This technique also helps deal with quantisation issues from smooth surfaces represented
  /// in the voxel cubes.
  ///
  /// @param sub_voxel_pattern The sub-voxel positioning pattern of the voxel of interest.
  /// @param sphere_scale Scale factor for the sphere [0, 1]. A scale of 1 fills the cube without over flow.
  /// @return True if the sub-voxel position lies within the filtering sphere confirming occupancy. False if the
  ///   position lies outside the sphere implying a rejection of occupancy (convert to free voxel).
  inline __device__ __host__ bool subVoxelOccupancyFilter2(unsigned sub_voxel_pattern, float sphere_scale)
  {
    // We work in a voxel space of a unit cube as only relative positions are relevant.
#if !GPUTIL_DEVICE
    const vec3 v = subVoxelToLocalCoord<vec3>(sub_voxel_pattern, 1.0f);
#else   //  GPUTIL_DEVICE
  const vec3 v = subVoxelToLocalCoord(sub_voxel_pattern, 1.0f);
#endif  //  GPUTIL_DEVICE
    return sub_voxel_pattern == 0 ||
           (v.x * v.x + v.y * v.y + v.z * v.z) < ((0.5f * sphere_scale) * (0.5f * sphere_scale));
  }

  /// @ingroup subvoxel
  /// This function is effectively an overload of @c subVoxelOccupancyFilter2() which uses the fully scaled sphere.
  /// @param sub_voxel_pattern The sub-voxel positioning pattern of the voxel of interest.
  /// @return True if the sub-voxel position lies within the filtering sphere confirming occupancy. False if the
  ///   position lies outside the sphere implying a rejection of occupancy (convert to free voxel).
  inline __device__ __host__ bool subVoxelOccupancyFilter(unsigned sub_voxel_pattern)
  {
    return subVoxelOccupancyFilter2(sub_voxel_pattern, 1.0f);
  }
#if !GPUTIL_DEVICE
}
#endif  // !GPUTIL_DEVICE

#endif  // SUBVOXEL_H
