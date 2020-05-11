// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef NDTVOXEL_H
#define NDTVOXEL_H

#include "OhmConfig.h"

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>

#define OHM_NDT_UNPACKED_MEAN 0

namespace ohm
{
  struct NdtVoxel
  {
    /// The lower diagonal of the covariance matrix for the voxel.
    float cov_sqrt_diag[6];
#if OHM_NDT_UNPACKED_MEAN
    float mean[3];
#endif  // OHM_NDT_UNPACKED_MEAN
    // TODO: check packing. Should be tightly packed.
    uint32_t point_count;
#if !OHM_NDT_UNPACKED_MEAN
    // Padding: voxel alignment is 8 bytes
    uint32_t padding;
#endif  // !OHM_NDT_UNPACKED_MEAN
  };

  /// Perform an eigen decomposition on the covariance dat ain @p ndt.
  /// Requires compilation with the Eigen maths library or it always returns false.
  bool ohm_API eigenDecomposition(const NdtVoxel &ndt, glm::dvec3 *eigenvalues, glm::dmat3 *eigenvectors);

  /// Unpack the covariance matrix storage.
  ///
  /// The unpacked matrix represents a sparse 3,4 matrix of the following form:
  ///
  /// |   |   |   |
  /// | - | - | - |
  /// | 0 | 1 | 3 |
  /// | . | 2 | 4 |
  /// | . | . | 5 |
  /// | 6 | 7 | 8 |
  ///
  /// Items marked '.' are not represented in the martix and are treated as zero.
  ///
  /// @param A The matrix to unpack to.
  /// @param sample_to_mean The difference between the new sample point and the voxel mean.
  /// @return An unpacked 3x3 covariance martrix (approximate).
  inline double *unpackedA(const NdtVoxel &ndt, double A[9], const glm::dvec3 sample_to_mean)
  {
    if (ndt.point_count == 0)
    {
      for (int i = 0; i < 9; ++i)
      {
        A[i] = 0;
      }
      return A;
    }

    const double one_on_num_pt_plus_one = 1.0 / (double(ndt.point_count) + 1);
    const double sc_1 = std::sqrt(double(ndt.point_count) * one_on_num_pt_plus_one);
    const double sc_2 = one_on_num_pt_plus_one * std::sqrt(double(ndt.point_count));

    for (int i = 0; i < 6; ++i)
    {
      A[i] = sc_1 * double(ndt.cov_sqrt_diag[i]);
    }

    for (int i = 0; i < 3; ++i)
    {
      A[i + 6] = sc_2 * sample_to_mean[i];
    }

    return A;
  }

  // Find x for Mx = y, given lower triangular M where M is @c cov_sqrt_diag
  // Storage order for M:
  // 0 z z
  // 1 2 z
  // 3 4 5
  inline glm::dvec3 solveTriangular(const NdtVoxel &ndt, const glm::dvec3 &y)
  {
    // FIXME: if we generate the voxel with point on a perfect plane, say (0, 0, 1, 0), then do this operation,
    // we get a divide by zero. The suggestion has been to seed the covariance matrix with an identity matrix scaled
    // by the sensor noise.
    const static int row_first_el[] = { 0, 1, 3 };
    glm::dvec3 x;
    for (int i = 0; i < 3; ++i)
    {
      double d = y[i];
      for (int j = 0; j < i; ++j)
      {
        d -= ndt.cov_sqrt_diag[row_first_el[i] + j] * x[j];
      }
      x[i] = d / ndt.cov_sqrt_diag[row_first_el[i] + i];
    }
    return x;
  }

  /// Unpack @c ndt.cov_sqrt_diag into a 3x3 covariance matrix.
  inline glm::dmat3 covarianceMatrix(const NdtVoxel &ndt)
  {
    glm::dmat3 cov;

    glm::dvec3 *col = &cov[0];
    (*col)[0] = ndt.cov_sqrt_diag[0];
    (*col)[1] = ndt.cov_sqrt_diag[1];
    (*col)[2] = ndt.cov_sqrt_diag[3];

    col = &cov[1];
    (*col)[0] = 0;
    (*col)[1] = ndt.cov_sqrt_diag[2];
    (*col)[2] = ndt.cov_sqrt_diag[4];

    col = &cov[2];
    (*col)[0] = 0;
    (*col)[1] = 0;
    (*col)[2] = ndt.cov_sqrt_diag[5];

    return cov;
  }


  inline void initialise(NdtVoxel *ndt, float sensor_noise)
  {
    ndt->point_count = 0;
    // Initialise the covariance matrix to a scaled identity matrix based on the sensor noise.
    ndt->cov_sqrt_diag[0] = ndt->cov_sqrt_diag[2] = ndt->cov_sqrt_diag[5] = sensor_noise * sensor_noise;
    ndt->cov_sqrt_diag[1] = ndt->cov_sqrt_diag[3] = ndt->cov_sqrt_diag[4] = 0;
#if OHM_NDT_UNPACKED_MEAN
    ndt->mean[0] = ndt->mean[1] = ndt->mean[2] = 0;
#else   // OHM_NDT_UNPACKED_MEAN
    ndt->padding = 0;
#endif  // OHM_NDT_UNPACKED_MEAN
  }
}  // namespace ohm

#endif  // NDTVOXEL_H
