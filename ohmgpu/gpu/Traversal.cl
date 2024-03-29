// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef TRAVERSAL_CL_H
#define TRAVERSAL_CL_H

/// Calculates the traversal for the line [start, end]. The reference frame of @p start and @p end is assumed to be
/// relative to the centre of the voxel containing @p end (the sample point). This means we can calculate the distance
/// traversed through this voxel by constructing six planes axis aligned around the origin at a distance of
/// @p 0.5*voxel_resolution . We find the distance travelled by intersecting the ray from @p end to @p start with these
/// planes and finding the first hit. The distance to this hit point marks the distance travelled through the voxel.
///
/// @param start The ray start point. This lies outside the end voxel and it's position is relative to the end voxel
///   centre.
/// @param end The ray end point. This lies in the end voxel and it's position is relative to the end voxel centre.
__device__ float calculateTraversal(float3 start, float3 end, float voxel_resolution);

inline __device__ float calculateTraversal(float3 start, float3 end, float voxel_resolution)
{
  // Define the planes which mark the edge of the voxel. The normals out of the voxel.
  // The distance part of the plane equation is implied below as 0.5*voxel_resolution
  const float3 voxel_plane_normals[6] = { make_float3(-1, 0, 0), make_float3(1, 0, 0),  make_float3(0, -1, 0),
                                          make_float3(0, 1, 0),  make_float3(0, 0, -1), make_float3(0, 0, 1) };

  // Seed with the length of the line segment. Doing so handles the case where both points lie within the voxel as we
  // minimise this value in our plane checks.
  float first_hit_time = length(start - end);
  // We trace a ray out of the voxel from end/sample point to start/origin.
  float3 dir = (start - end) / (first_hit_time > 0 ? first_hit_time : 1.0f);

  for (int i = 0; i < 6; ++i)
  {
    const float ray_dot = dot(voxel_plane_normals[i], dir);
    // Generate some point on the plane. It's the voxel centre plus the n*resolution/2 . We know the rays are passed to
    // GPU such that the start and end points are relative to the end voxel centre, so it's (0, 0, 0)
    // Modify p so that is the vector from end to some point on the plane.
    const float3 p = voxel_plane_normals[i] * 0.5f * voxel_resolution - end;
    const float plane_hit_time = (ray_dot > 1e-6f) ? dot(p, voxel_plane_normals[i]) / ray_dot : INFINITY;
    first_hit_time = min(first_hit_time, plane_hit_time);
  }

  return first_hit_time;
}

#endif  // TRAVERSAL_CL_H
