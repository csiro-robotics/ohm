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

#ifndef __OPENCL_C_VERSION__
namespace ohm
{
#define SUB_VOX_FUNC_PREFACE template <typename vec3, typename coord_real>

#else  // !__OPENCL_C_VERSION__

#if !defined(coord_real)
typedef float coord_real;
#endif  // !defined(coord_real)

#if !defined(vec3)
typedef float3 vec3;
#endif  // !defined(vec3)

#define SUB_VOX_FUNC_PREFACE

#endif  // !__OPENCL_C_VERSION__

  /// @ingroup subvoxel
  /// Convert @p voxel_local_coord into a sub-voxel positioning pattern.
  /// @param voxel_local_coord The new coordinate to store. This position should be within the voxel bounds relative
  ///   to the centre of the voxel; i.e., [-0.5 * resolution, 0.5 * resolution]. The position is clamped to the voxel
  ///   bounds.
  /// @param resolution The length of each voxel cube edge.
  /// @return The sub-voxel pattern approximating @p voxel_local_coord.
  SUB_VOX_FUNC_PREFACE
  unsigned subVoxelCoord(vec3 voxel_local_coord, coord_real resolution)
  {
    // We divide the voxel into a voxel_local_coord, 3D grid, then assign 1 bit per cell.
    const unsigned bits_per_axis = 10;
    const unsigned sub_voxel_positions = (1 << bits_per_axis) - 1;
    const unsigned used_bit = (1 << 31);
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
  vec3 subVoxelToLocalCoord(unsigned pattern, coord_real resolution)
  {
    const unsigned bits_per_axis = 10;
    const unsigned sub_voxel_positions = (1 << bits_per_axis) - 1;
    const unsigned used_bit = (1 << 31);
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
  unsigned subVoxelUpdate(unsigned initial_pattern, vec3 voxel_local_coord, coord_real resolution, coord_real weighting)
  {
    vec3 old_local =
#ifndef __OPENCL_C_VERSION__
      subVoxelToLocalCoord<vec3>(initial_pattern, resolution)
#else   //  __OPENCL_C_VERSION__
    subVoxelToLocalCoord(initial_pattern, resolution)
#endif  //  __OPENCL_C_VERSION__
      ;
    old_local.x *= (1.0 - weighting);
    old_local.y *= (1.0 - weighting);
    old_local.z *= (1.0 - weighting);
    voxel_local_coord.x *= weighting;
    voxel_local_coord.y *= weighting;
    voxel_local_coord.z *= weighting;
    return subVoxelCoord(old_local + voxel_local_coord, resolution);
  }
#ifndef __OPENCL_C_VERSION__
}
#endif  // !__OPENCL_C_VERSION__

#endif  // SUBVOXEL_H
