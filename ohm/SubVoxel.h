// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SUBVOXEL_H
#define SUBVOXEL_H

#ifndef __OPENCL_C_VERSION__
#include <OhmConfig.h>
#endif // !__OPENCL_C_VERSION__


#include "MapCoord.h"

// Note: this header is included in GPU code.
// Because of this <cmath> cannot be included here and you may need to include that first.

// Sub voxel positioning is done as follows:
// - 4 byte value, split into bit sets:
// - bits [0, 9]: sub-voxel x coord
// - bits [10, 19]: sub-voxel y coord
// - bits [20, 29]: sub-voxel z coord
// - bit 30: unused
// - bit 31: marks sub-voxel data as present

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

    pos_x = (pos_x >= 0 ? (pos_x < (1 << bits_per_axis) ? pos_x : sub_voxel_positions) : 0) ;
    pos_y = (pos_y >= 0 ? (pos_y < (1 << bits_per_axis) ? pos_y : sub_voxel_positions) : 0) ;
    pos_z = (pos_z >= 0 ? (pos_z < (1 << bits_per_axis) ? pos_z : sub_voxel_positions) : 0) ;

    unsigned pattern = 0;
    pattern |= (unsigned)pos_x;
    pattern |= ((unsigned)pos_y << bits_per_axis);
    pattern |= ((unsigned)pos_z << (2 * bits_per_axis));
    pattern |= used_bit;
    return pattern;
  }


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
    coord.y = (used_bit) ? regionCentreCoord((int)((pattern >> bits_per_axis) & sub_voxel_positions), sub_voxel_resolution) - offset : 0;
    coord.z = (used_bit) ? regionCentreCoord((int)((pattern >> (2 * bits_per_axis)) & sub_voxel_positions), sub_voxel_resolution) - offset : 0;
    return coord;
  }


  SUB_VOX_FUNC_PREFACE
  unsigned subVoxelUpdate(unsigned initial, vec3 voxel_local_coord, coord_real resolution)
  {
#ifndef __OPENCL_C_VERSION__
    const vec3 half_scale(0.5, 0.5, 0.5);
#else  // __OPENCL_C_VERSION__
    const vec3 half_scale = (vec3)(0.5, 0.5, 0.5);
#endif // __OPENCL_C_VERSION__
    voxel_local_coord =
      (initial) ? half_scale * (voxel_local_coord + subVoxelToLocalCoord(initial, resolution)) : voxel_local_coord;
    return subVoxelCoord(voxel_local_coord, resolution);
  }
#ifndef __OPENCL_C_VERSION__
}
#endif  // !__OPENCL_C_VERSION__

#endif  // SUBVOXEL_H
