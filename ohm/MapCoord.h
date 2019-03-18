// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// Copyright (c) 2017
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPCOORD_H
#define OHM_MAPCOORD_H

// Note: this header is included in GPU code.
// Because of this <cmath> cannot be included here and you may need to include that first.

#if !GPUTIL_DEVICE
#ifndef __device__
#define __device__
#endif  // __device__
#ifndef __host__
#define __host__
#endif  // __host__

namespace ohm
{
#endif  // !GPUTIL_DEVICE

  // Define real coordinate for functions below. Normally double, float for GPU.
#if GPUTIL_DEVICE == 1 && !defined(COORD_REAL)
#define COORD_REAL
  typedef float coord_real;
#endif  // GPUTIL_DEVICE && !defined(COORD_REAL)

  /// Calculate the map local centre coordinate for a region along a single axis.
  /// @param regionCoord The coordinate of the region in the map along the axis of interest.
  /// @param regionDimension The global size of each region along the axis of interest.
  /// @return The centre of the region along this axis local to the map centre.
#if GPUTIL_DEVICE != 1
  template <typename coord_real>
#endif  // GPUTIL_DEVICE != 1
  inline __device__ __host__ coord_real regionCentreCoord(int region_coord, const coord_real region_dimesion)
  {
    return region_coord * region_dimesion;
  }

#if GPUTIL_DEVICE != 1
  template <typename coord_real>
#endif  // GPUTIL_DEVICE != 1
  inline __device__ __host__ int pointToRegionVoxel(coord_real coord, coord_real voxel_resolution,
                                                    coord_real region_resolution)
  {
    // Due to precision error, we can end up with coordinates just outside the region spatial boundary.
    // That is, either just below zero, or just above regionResolution. Since by now we should have determined
    // the correct region for coord, we need to handle this imprecision. To do so, we simply adjust such
    // values by a small epsilon.
    //
    // In testing, the largest double precision delta I've seen has been on the order of 1e-15, while
    // the largest single precision delta should be around 5e-7. We use an epsilon of 1e-6 to support
    // single precision. Anything larger is treated as an invalid input.
    const coord_real epsilon = (coord_real)1e-6;
    if (-epsilon <= coord && coord < 0)
    {
      // Note: TEST_EPSILON_MAGNITUDE is not explicitly defined anywhere. The code is for testing only.
#ifdef TEST_EPSILON_MAGNITUDE
      static coord_real largestDelta = 0;
      if (-coord > -largestDelta)
      {
        largestDelta = coord;
        fprintf(stderr, "W: pointToRegionVoxel(%lg, %lg) : negative coord\n", coord, voxelResolution);
      }
#endif  // TEST_EPSILON_MAGNITUDE
      coord = 0;
    }
    else if (coord >= region_resolution && coord - epsilon < region_resolution)
    {
      coord -= epsilon;
    }
    return (int)floor((coord / voxel_resolution));
  }

#if GPUTIL_DEVICE != 1
  template <typename coord_real>
#endif  // GPUTIL_DEVICE != 1
  inline __device__ __host__ int pointToRegionCoord(coord_real coord, coord_real resolution)
  {
    return (int)floor(coord / resolution + (coord_real)0.5);
  }
#if !GPUTIL_DEVICE
}
#endif  // !GPUTIL_DEVICE

#endif  // OHM_MAPCOORD_H
