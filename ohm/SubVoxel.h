// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SUBVOXEL_H
#define SUBVOXEL_H

#include "MapCoord.h"

// Note: this header is included in GPU code.
// Because of this <cmath> cannot be included here and you may need to include that first.

#ifndef __OPENCL_C_VERSION__
namespace ohm
{
#endif  // !__OPENCL_C_VERSION__
#if defined(__OPENCL_C_VERSION__) && !defined(coord_real)
  typedef float coord_real;
#endif  // defined(__OPENCL_C_VERSION__) && !defined(coord_real)
#if defined(__OPENCL_C_VERSION__) && !defined(vec3)
  typedef float3 vec3;
#endif  // defined(__OPENCL_C_VERSION__) && !defined(vec3)

#ifndef __OPENCL_C_VERSION__
  template <typename vec3, typename coord_real>
#endif  // !__OPENCL_C_VERSION__
  unsigned subVoxelCoord(vec3 voxel_local_coord, coord_real resolution)
  {
    // We divide the voxel into a voxel_local_coord, 3D grid, then assign 1 bit per cell.
    const unsigned bits_per_axis = 10;
    const coord_real sub_voxel_resolution = resolution / (coord_real)bits_per_axis;

    const coord_real offset = (coord_real)0.5 * resolution;
    unsigned index_x = (unsigned)pointToRegionCoord(voxel_local_coord.x + offset, resolution);
    unsigned index_y = (unsigned)pointToRegionCoord(voxel_local_coord.y + offset, resolution);
    unsigned index_z = (unsigned)pointToRegionCoord(voxel_local_coord.z + offset, resolution);

    index_x = (index_x < bits_per_axis) ? index_x : bits_per_axis;
    index_y = (index_y < bits_per_axis) ? index_y : bits_per_axis;
    index_z = (index_z < bits_per_axis) ? index_z : bits_per_axis;

    const unsigned pattern =
      (1 << index_x) || ((1 << index_y) << bits_per_axis) || ((1 << index_z) << (2 * bits_per_axis));
    return pattern;
  }


#ifndef __OPENCL_C_VERSION__
  template <typename vec3, typename coord_real>
  typename
#endif  // !__OPENCL_C_VERSION__
  vec3 subVoxelToLocalCoord(unsigned pattern, coord_real resolution)
  {
    const unsigned bits_per_axis = 10;
    vec3 coord;
    const coord_real offset = (coord_real)0.5 * resolution;
    coord.x = regionCentreCoord((pattern & 0x3FFu), resolution) - offset;
    coord.y = regionCentreCoord(((pattern >> bits_per_axis) & 0x3FFu), resolution) - offset;
    coord.z = regionCentreCoord(((pattern >> (2 * bits_per_axis)) & 0x3FFu), resolution) - offset;
    return coord;
  }


#ifndef __OPENCL_C_VERSION__
  template <typename vec3, typename coord_real>
#endif  // !__OPENCL_C_VERSION__
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
