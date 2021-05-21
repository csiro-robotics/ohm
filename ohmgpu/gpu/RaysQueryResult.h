// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPU_RAYQUERY_RESULT_H
#define OHMGPU_GPU_RAYQUERY_RESULT_H

#ifndef RQ_OccNull
// TODO(KS): include OccupancyType.h or formalise the types.
/// Invalid/null voxel.
#define RQ_OccNull (-2)
/// Unobserved: no data recorded or available for the voxel.
#define RQ_OccUnobserved (-1)
/// Know to be empty or free (traversable).
#define RQ_OccFree (0)
/// Occupied voxel.
#define RQ_OccOccupied (1)
#endif  // RQ_OccNull

/// Structure used to write results for a GpuRayQuery.
typedef struct RaysQueryResult_t  // NOLINT(readability-identifier-naming, modernize-use-using)
{
  /// Range the ray successfully traverses.
  float range;
  /// Unobserved volume metric.
  float unobserved_volume;
  /// Occupancy type of the final voxel.
  int voxel_type;
} RaysQueryResult;

#endif  // OHMGPU_GPU_RAYQUERY_RESULT_H
