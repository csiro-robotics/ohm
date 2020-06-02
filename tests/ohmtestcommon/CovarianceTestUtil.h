// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CovarianceTestUtil_H
#define CovarianceTestUtil_H

#include <ohm/OhmConfig.h>

// Required before CovarianceVoxel.h
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtx/norm.hpp>

#include <ohm/CovarianceVoxel.h>

namespace ohmtestutil
{
  /// Ndt reference data.
  struct CovTestVoxel : public ohm::CovarianceVoxel
  {
    double mean[3];
    unsigned point_count;
  };

  void initialiseTestVoxel(CovTestVoxel *ref_voxel, float sensor_noise);
  double packed_dot(const double *A, const size_t j, const size_t k);
  void updateHit(CovTestVoxel *cov, const glm::dvec3 &sample);
  bool validate(const glm::dvec3 &mean, unsigned point_count, const ohm::CovarianceVoxel &cov, const CovTestVoxel &ref);
}

#endif // CovarianceTestUtil_H
