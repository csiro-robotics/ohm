// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_POINTS_H_
#define SLAMIO_POINTS_H_

#include "SlamIOConfig.h"

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cstring>

namespace slamio
{
/// Extracted point cloud sample.
struct CloudPoint
{
  double timestamp;     ///< Point timestamp (if available)
  glm::dvec3 position;  ///< Sample position (if available)
  glm::dvec3 normal;    ///< Sample normal value. May be a ray when loading rayclouds or with trajectory (if available)
  glm::vec4 colour;     ///< Point colour (if available)
  float intensity;      ///< Point intensity value (if available)
  uint8_t return_number;  ///< Zero based, sample return number (if available). Primary return is 0, second 1, etc.
};

/// SLAM sample point. This is very similar to a point cloud sample, but has an @p origin point rather than a point
/// cloud @p normal . In practice, clouds loaded by this library expect that point cloud normals will already be origin
/// points rather than normals.
struct SamplePoint
{
  double timestamp;       ///< Sample timestamp (if available)
  glm::dvec3 sample;      ///< Sample position
  glm::dvec3 origin;      ///< Point from which the same was generated (sensor position, if available)
  glm::vec4 colour;       ///< Sample colour (if available)
  float intensity;        ///< Sample intensity (if available)
  uint8_t return_number;  ///< Zero based, sample return number (if available). Primary return is 0, second 1, etc.
};

inline void c2sPt(SamplePoint &sample, const CloudPoint &cloud)
{
  memcpy(&sample, &cloud, sizeof(sample));
}

inline SamplePoint c2sPt(const CloudPoint &cloud)
{
  SamplePoint sample;
  c2sPt(sample, cloud);
  return sample;
}

inline void s2cPt(CloudPoint &cloud, const SamplePoint &sample)
{
  static_assert(sizeof(cloud) == sizeof(sample), "Cloud/sample point size mismatch. Cannot use memcpy().");
  memcpy(&cloud, &sample, sizeof(sample));
}

inline CloudPoint s2cPt(const SamplePoint &sample)
{
  CloudPoint cloud;
  s2cPt(cloud, sample);
  return cloud;
}
}  // namespace slamio

#endif  // SLAMIO_POINTS_H_
