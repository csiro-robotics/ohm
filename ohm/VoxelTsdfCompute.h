// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELTSDFCOMPUTE_H
#define VOXELTSDFCOMPUTE_H

#if !GPUTIL_DEVICE
#ifndef __device__
#define __device__
#endif  // __device__
#ifndef __host__
#define __host__
#endif  // __host__

#define TSDF_VOX_FUNC_PREFACE template <typename Vec3>

/// Voxel data for maintaining truncated signed distance field.
struct VoxelTsdf
{
  float weight;    ///< TSDF weight.
  float distance;  ///< TSDF distance.
};

#else  // !GPUTIL_DEVICE

#if !defined(VEC3)
typedef float3 Vec3;
#endif  // !defined(VEC3)

#if !defined(Int3)
typedef int3 Int3;
#endif  // !defined(Int3)

#define TSDF_VOX_FUNC_PREFACE

/// Voxel data for maintaining truncated signed distance field.
typedef struct VoxelTsdf_t
{
  atomic_float weight;    ///< TSDF weight.
  atomic_float distance;  ///< TSDF distance.
} VoxelTsdf;

#endif  // !GPUTIL_DEVICE

/// Check if @p voxel has valid data in it.
/// @return True if @p voxel is non-null and has valid data, false when null or unused.
inline __device__ __host__ bool isValidTsdf(const VoxelTsdf *voxel)
{
  return voxel && (voxel->weight != 0 || voxel->distance != 0);
}


TSDF_VOX_FUNC_PREFACE
inline __device__ __host__ float computeDistance(const Vec3 sensor, const Vec3 sample, const Vec3 voxel_centre)
{
  const Vec3 sensor_to_voxel = voxel_centre - sensor;
  const Vec3 sensor_to_sample = sample - sensor;

  const float distance_g = (float)sqrt(dot(sensor_to_sample, sensor_to_sample));
  // Project sensor to voxel onto the sample ray.
  const float distance_g_v = (float)dot(sensor_to_voxel, sensor_to_sample) / distance_g;

  const float sdf = distance_g - distance_g_v;
  return sdf;
}


/// Calculate the truncated signed distance fields (TSDF) update for a voxel.
///
/// This code is based on Voxblox fast TSDF update and is licensed under BSD 3-clause license.
///
/// @param sensor Sensor position/the start of the sample ray.
/// @param sample The sample position and the end of the sample ray.
/// @param voxel_centre The center of the voxel being updated. The voxel lies along the ray.
/// @param max_weight The maximum weight allowed for a TSDF voxel.
/// @param dropoff_epsilon Used to calculate weight dropoff. The recommended value is to match the voxel size.
///   See voxblox for details. Ignored if zero or less.
/// @param use_sparsity_compensation_factor Use compensation for sparse point clouds?
/// @param sparsity_compensation_factor The sparsity compensation factor to apply. Ignored if zero or less.
/// @param[in,out] voxel_weight Initial TSDF voxel weight which is then modified with the TSDF calculation for this ray.
/// @param[in,out] voxel_distance Initial TSDF voxel distance, then modified with the TSDF calculation for this ray.
/// @return True if the calculation modified the @p voxel_weight and/or @p voxel_distance.
TSDF_VOX_FUNC_PREFACE
inline __device__ __host__ bool calculateTsdf(const Vec3 sensor, const Vec3 sample, const Vec3 voxel_centre,
                                              float default_truncation_distance, float max_weight,
                                              float dropoff_epsilon, float sparsity_compensation_factor,
                                              float *voxel_weight, float *voxel_distance)
{
  // Calculate the TSDF adjustment.
#if !GPUTIL_DEVICE
  using namespace std;
#else   // !GPUTIL_DEVICE
#endif  // !GPUTIL_DEVICE
  const float sdf = computeDistance(sensor, sample, voxel_centre);

  const float initial_weight = *voxel_weight;
  float updated_weight = 1.0f;  // Equivalent to voxblox use_constant_weight

  // Comment from voxblox
  // Compute updated weight in case we use weight dropoff. It's easier here
  // that in getVoxelWeight as here we have the actual SDF for the voxel
  // already computed.
  updated_weight *= (dropoff_epsilon > 0) ?
                      ((default_truncation_distance + sdf) / (default_truncation_distance - dropoff_epsilon)) :
                      1.0f;
  updated_weight = max(updated_weight, 0.0f);

  // Comment from voxblox
  // Compute the updated weight in case we compensate for sparsity. By
  // multiplicating the weight of occupied areas (|sdf| < truncation distance)
  // by a factor, we prevent to easily fade out these areas with the free
  // space parts of other rays which pass through the corresponding voxels.
  // This can be useful for creating a TSDF map from sparse sensor data (e.g.
  // visual features from a SLAM system). By default, this option is disabled.
  updated_weight *=
    (sparsity_compensation_factor > 0 && fabs(sdf) < default_truncation_distance) ? sparsity_compensation_factor : 1.0f;

  const float new_weight = initial_weight + updated_weight;

  // Comment from voxblox
  // it is possible to have weights very close to zero, due to the limited
  // precision of floating points dividing by this small value can cause nans
  const float abs_new_weight = fabs(new_weight);
  const bool near_zero_weight = abs_new_weight < 0.00001f;
  const float new_sdf =
    (!near_zero_weight) ? (sdf * updated_weight + *voxel_distance * initial_weight) / new_weight : 0;

  *voxel_distance = (!near_zero_weight) ? ((new_sdf > 0.0) ? min(default_truncation_distance, new_sdf) :
                                                             max(-default_truncation_distance, new_sdf)) :
                                          *voxel_distance;
  *voxel_weight = (!near_zero_weight) ? min(new_weight, max_weight) : initial_weight;
  return !near_zero_weight;
}

#endif  // VOXELTSDFCOMPUTE_H
