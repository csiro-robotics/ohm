// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "CovarianceTestUtil.h"

#include <algorithm>
#include <vector>

  // The reference code below was provided by Jason Williams as a guide to implementing the NDT map epsecially with
  // consideration to using a packed covariance matrix. We use this code to validate the NDT results.
  // Note: Jason's code was written using Eigen for mathematical operations which we've converted to glm. The ohm
  // library prefers glm as it is a much ligher weight library and we do not require Eigen's advanced functionality.

namespace ohmtestutil
{
  void initialiseTestVoxel(CovTestVoxel *ref_voxel, float sensor_noise)
  {
    initialiseCovariance(ref_voxel, sensor_noise);
    ref_voxel->mean[0] = ref_voxel->mean[1] = ref_voxel->mean[2] = 0;
    ref_voxel->point_count = 0;
  }

  // dot product of j-th and k-th columns of A
  // A is (4,3), assumed to be packed as follows, where z is non-represented zero
  // 0 1 3
  // z 2 4
  // z z 5
  // 6 7 8
  double packed_dot(const double *A, const size_t j, const size_t k)
  {
    const static size_t col_first_el[] = { 0, 1, 3 };
    double d = A[6 + k] * A[6 + j];
    const size_t indj = col_first_el[j], indk = col_first_el[k], m = std::min(j, k);
    for (size_t i = 0; i <= m; ++i)
    {
      d += A[indj + i] * A[indk + i];
    }
    return d;
  }

  void updateHit(CovTestVoxel *cov, const glm::dvec3 &sample)
  {
    const double num_pt = double(cov->point_count);
    const double one_on_num_pt_plus_one = 1.0 / (num_pt + 1.0);
    glm::dvec3 mean(cov->mean[0], cov->mean[1], cov->mean[2]);
    const glm::dvec3 diff = sample - mean;
    const double sc_1 = num_pt ? std::sqrt(num_pt * one_on_num_pt_plus_one) : 1;
    const double sc_2 = one_on_num_pt_plus_one * std::sqrt(num_pt);
    std::vector<double> A(9);
    for (unsigned i = 0; i < 6; ++i)
    {
      A[i] = sc_1 * cov->trianglar_covariance[i];
    }
    for (unsigned i = 0; i < 3; ++i)
    {
      A[i + 6] = sc_2 * diff[i];
    }
    for (unsigned k = 0; k < 3; ++k)
    {
      const unsigned ind1 = (k * (k + 3)) >> 1,  // packed index of (k,k) term
        indk = ind1 - k;                       // packed index of (1,k)
      const double ak = std::sqrt(packed_dot(&A[0], k, k));
      cov->trianglar_covariance[ind1] = float(ak);
      if (ak > 0.0)
      {
        const double aki = 1.0 / ak;
        for (unsigned j = k + 1; j < 3; ++j)
        {
          const unsigned indj = (j * (j + 1)) >> 1, indkj = indj + k;
          double c = packed_dot(&A[0], j, k) * aki;
          cov->trianglar_covariance[indkj] = float(c);
          c *= aki;
          A[j + 6] -= c * A[k + 6];
          for (unsigned l = 0; l <= k; ++l)
          {
            A[indj + l] -= c * A[indk + l];
          }
        }
      }
    }

    mean = (num_pt * mean + sample) * one_on_num_pt_plus_one;
    cov->mean[0] = mean[0];
    cov->mean[1] = mean[1];
    cov->mean[2] = mean[2];
    ++cov->point_count;
  }

  bool validate(const glm::dvec3 &mean, unsigned point_count, const ohm::CovarianceVoxel &cov, const CovTestVoxel &ref)
  {
    // Quantisation in the mean storage create more signficant absolute errors in the covariance and mean.
    const double epsilon_cov = 1e-2;
    const double epsilon_mean = 1e-1;
    for (int i = 0; i < 6; ++i)
    {
      if (std::abs(cov.trianglar_covariance[i] - ref.trianglar_covariance[i]) > epsilon_cov)
      {
        return false;
      }
    }

    if (point_count != ref.point_count)
    {
      return false;
    }

    const glm::dvec3 ref_mean(ref.mean[0], ref.mean[1], ref.mean[2]);
    const double mean_diff = glm::length(mean - ref_mean);
    if (std::abs(mean_diff) > epsilon_mean)
    {
      return false;
    }

    return true;
  }
}
