// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef OHM_VOXEL_INCIDENT_COMPUTE_H
#define OHM_VOXEL_INCIDENT_COMPUTE_H

#if !GPUTIL_DEVICE
using Vec3 = glm::vec3;
#define OHM_NORMAL_STD std::
#define OHM_DEVICE_HOST
#else  // GPUTIL_DEVICE
typedef float3 Vec3;
#define OHM_NORMAL_STD
#define OHM_DEVICE_HOST __device__ __host__
#endif  // GPUTIL_DEVICE

#define OHM_NORMAL_QUAT       16383.0f
#define OHM_NORMAL_MASK       0x3FFF
#define OHM_NORMAL_SHIFT_X    0
#define OHM_NORMAL_SHIFT_Y    15
#define OHM_NORMAL_SET_BIT_Z  30
#define OHM_NORMAL_SIGN_BIT_Z 31

inline Vec3 OHM_DEVICE_HOST decodeNormal(unsigned packed_normal)
{
  Vec3 n;

  n.x = (2.0f * ((packed_normal >> OHM_NORMAL_SHIFT_X) & OHM_NORMAL_MASK) / OHM_NORMAL_QUAT) - 1.0f;
  n.y = (2.0f * ((packed_normal >> OHM_NORMAL_SHIFT_Y) & OHM_NORMAL_MASK) / OHM_NORMAL_QUAT) - 1.0f;

  n.x = OHM_NORMAL_STD max(-1.0f, OHM_NORMAL_STD min(n.x, 1.0f));
  n.y = OHM_NORMAL_STD max(-1.0f, OHM_NORMAL_STD min(n.y, 1.0f));
  n.z = OHM_NORMAL_STD max(-1.0f, OHM_NORMAL_STD min(1.0f - (n.x * n.x + n.y * n.y), 1.0f));

  n.x = (packed_normal & (1u << OHM_NORMAL_SET_BIT_Z)) ? n.x : 0.0f;
  n.y = (packed_normal & (1u << OHM_NORMAL_SET_BIT_Z)) ? n.y : 0.0f;
  n.z = (packed_normal & (1u << OHM_NORMAL_SET_BIT_Z)) ? sqrt(n.z) : 0.0f;

  return n;
}

/// Encode a normalised vector into a 32-bit floating point value.
///
/// We use 15-bits each to encode X and Y channels. We use the most significant bit (31) to encode the sign of Z.
/// Bit 30 is used to incidate if there is a valid normal stored.
inline unsigned OHM_DEVICE_HOST encodeNormal(Vec3 normal)
{
  unsigned n = 0;

  // Adjust normal range from [-1, 1] -> [0, 2] -> [0, 1]
  normal.x = 0.5f * (OHM_NORMAL_STD max(-1.0f, OHM_NORMAL_STD min(normal.x, 1.0f)) + 1.0f);
  normal.y = 0.5f * (OHM_NORMAL_STD max(-1.0f, OHM_NORMAL_STD min(normal.y, 1.0f)) + 1.0f);

  unsigned i = (unsigned)(normal.x * OHM_NORMAL_QUAT);
  n |= (i & OHM_NORMAL_MASK) << OHM_NORMAL_SHIFT_X;
  i = (unsigned)(normal.y * OHM_NORMAL_QUAT);
  n |= (i & OHM_NORMAL_MASK) << OHM_NORMAL_SHIFT_Y;

  n |= (normal.z < 0) ? (1 << OHM_NORMAL_SIGN_BIT_Z) : 0;
  // mark as set if the input normal is non-zero.
  n |= (normal.x || normal.y || normal.z) ? (1u << OHM_NORMAL_SET_BIT_Z) : 0;

  return n;
}

inline Vec3 OHM_DEVICE_HOST updateIncidentNormalV3(Vec3 normal, Vec3 incident_ray, unsigned point_count)
{
  // Handle having a zero normal as an initialiastion pass regardless of point count.
  point_count = ((normal.x != 0 || normal.y != 0 || normal.z != 0) && point_count) ? point_count : 0;
  const float one_on_count_plus_one = 1.0f / (float)(point_count + 1);
  float normal_length_2 =
    incident_ray.x * incident_ray.x + incident_ray.y * incident_ray.y + incident_ray.z * incident_ray.z;
  incident_ray *= (normal_length_2 > 1e-6f) ? 1.0f / sqrt(normal_length_2) : 0.0f;
  normal.x += (incident_ray.x - normal.x) * one_on_count_plus_one;
  normal.y += (incident_ray.y - normal.y) * one_on_count_plus_one;
  normal.z += (incident_ray.z - normal.z) * one_on_count_plus_one;
  normal_length_2 = normal.x * normal.x + normal.y * normal.y + normal.z * normal.z;
  normal *= (normal_length_2 > 1e-6f) ? 1.0f / sqrt(normal_length_2) : 0.0f;
  return normal;
}

inline unsigned OHM_DEVICE_HOST updateIncidentNormal(unsigned packed_normal, Vec3 incident_ray, unsigned point_count)
{
  Vec3 normal = decodeNormal(packed_normal);
  normal = updateIncidentNormalV3(normal, incident_ray, point_count);
  return encodeNormal(normal);
}

#undef OHM_DEVICE_HOST

#endif  // OHM_VOXEL_INCIDENT_COMPUTE_H
