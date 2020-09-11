// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef COVARIANCEVOXEL_H
#define COVARIANCEVOXEL_H

#include "OhmConfig.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtx/norm.hpp>

/// Allows additional debug information on @c covarianceEigenDecomposition()
#define OHM_COV_DEBUG 0


/// @defgroup voxelcovariance Voxel Covariance Functions
///
/// These functions support updating @c VoxelCovariance in CPU and GPU. This update implements a normal distributions
/// transform logic for occupancy voxels.
///
/// For reference, see:
///
/// Saarinen, Jari & Andreasson, Henrik & Stoyanov, Todor & Lilienthal, Achim. (2013). 3D Normal Distributions Transform
/// Occupancy Maps: An Efficient Representation for Mapping in Dynamic Environments. The International Journal of
/// Robotics Research. 32. 1627-1644. 10.1177/0278364913499415.

namespace ohm
{
#include "CovarianceVoxelCompute.h"

  class Key;
  class NdtMap;

  /// @ingroup voxelcovariance
  /// Perform an eigen decomposition on the covariance data in @p cov.
  ///
  /// This currently uses the QR algorithm. This is an iterative solution, which is not recommended. Therefore this
  /// function is not recommended for high performance code.
  ///
  /// @param cov The covariance voxel to operate on.
  void ohm_API covarianceEigenDecomposition(const CovarianceVoxel *cov, glm::dmat3 *eigenvectors,
                                            glm::dvec3 *eigenvalues);

  /// @ingroup voxelcovariance
  /// Estimate a primary normal from the given covariance. This selects the eivenvector with the smallest eigenvalue.
  /// This may be ambiguous.
  /// @param cov The covariance data for the voxel.
  /// @param normal The normal value is written here.
  /// @param peferred_axis Optionally specify which axis to prefer when the covariance is ambiguous. E.g., for voxels
  /// with insufficient data.
  void ohm_API covarianceEstimatePrimaryNormal(const CovarianceVoxel *cov, glm::dvec3 *normal, int preferred_axis = 0);

  /// @ingroup voxelcovariance
  /// Convert @p cov into a rotation and scale factors to deform a unit sphere to approximate the covariance cluster.
  ///
  /// This calls @c covarianceEigenDecomposition() and suffers the same performance constraints.
  ///
  /// @param cov The covariance voxel to operate on.
  /// @param[out] rotation The quaternion rotation to apply to the unit sphere after applying @p scale .
  /// @param[out] scale The scaling to apply to the unit sphere before @p rotation .
  bool ohm_API covarianceUnitSphereTransformation(const CovarianceVoxel *cov, glm::dquat *rotation, glm::dvec3 *scale);

  /// @ingroup voxelcovariance
  /// Unpack @c cov.trianglar_covariance into a 3x3 covariance matrix.
  inline glm::dmat3 covarianceSqrtMatrix(const CovarianceVoxel *cov)
  {
    glm::dmat3 cov_sqrt_mat;

    glm::dvec3 *col = &cov_sqrt_mat[0];
    (*col)[0] = cov->trianglar_covariance[0];
    (*col)[1] = cov->trianglar_covariance[1];
    (*col)[2] = cov->trianglar_covariance[3];

    col = &cov_sqrt_mat[1];
    (*col)[0] = 0;
    (*col)[1] = cov->trianglar_covariance[2];
    (*col)[2] = cov->trianglar_covariance[4];

    col = &cov_sqrt_mat[2];
    (*col)[0] = 0;
    (*col)[1] = 0;
    (*col)[2] = cov->trianglar_covariance[5];

    return cov_sqrt_mat;
  }

  /// @ingroup voxelcovariance
  /// Unpack @c cov.trianglar_covariance into a 3x3 covariance matrix.
  inline glm::dmat3 covarianceMatrix(const CovarianceVoxel *cov)
  {
    const glm::dmat3 cov_sqrt_mat = covarianceSqrtMatrix(cov);
    return cov_sqrt_mat * glm::inverse(cov_sqrt_mat);
  }

#if OHM_COV_DEBUG
  void covDebugStats();
#endif  // OHM_COV_DEBUG

  /// Integrate a hit result for a single voxel of @p map with NDT support.
  /// @param map The @c NdtMap to integrate the hit for.
  /// @param key The key for the voxel to modify. This is the voxel containing @p sample
  /// @param sample The gobal sample coordinate.
  void ohm_API integrateNdtHit(NdtMap &map, const Key &key, const glm::dvec3 &sample);

  /// Integrate a miss result for a single voxel of @p map with NDT support.
  /// @param map The @c NdtMap to integrate the hit for.
  /// @param key The key for the voxel to modify. This is a voxel along the line segment @p sensor to @p sample, but
  /// not the voxel containing @p sample .
  /// @param sensor The sensor location from which the @p sample was attained.
  /// @param sample The gobal sample coordinate.
  void ohm_API integrateNdtMiss(NdtMap &map, const Key &key, const glm::dvec3 &sensor, const glm::dvec3 &sample);
}  // namespace ohm

#endif  // COVARIANCEVOXEL_H
