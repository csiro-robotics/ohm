// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef NDTVOXEL_H
#define NDTVOXEL_H

#if !GPUTIL_DEVICE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtx/norm.hpp>
#else  // GPUTIL_DEVICE
#ifndef __device__
#define __device__
#endif  // __device__
#ifndef __host__
#define __host__
#endif  // __host__
#endif  // GPUTIL_DEVICE

#define OHM_NDT_UNPACKED_MEAN 0

#if GPUTIL_DEVICE
// Define GPU type aliases
typedef float3 ndtvec3;
typedef double ndtreal;
typedef uint uint32_t;
#else  // GPUTIL_DEVICE
namespace ohm
{
  // Define CPU type aliases
  using ndtvec3 = glm::dvec3;
  using ndtreal = double;
#ifndef __device__
#define __device__
#endif  //  __device__
#endif  // GPUTIL_DEVICE
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


inline __device__ void initialiseNdt(NdtVoxel *ndt, float sensor_noise)
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


/// dot product of j-th and k-th columns of A
/// A is (4,3), assumed to be packed as follows, where z is non-represented zero
/// 0 1 3
/// z 2 4
/// z z 5
/// 6 7 8
inline __device__ double packedDot(const ndtreal A[9], const int j, const int k)
{
  const int col_first_el[] = { 0, 1, 3 };
  const int indj = col_first_el[j];
  const int indk = col_first_el[k];
  const int m = (j <= k) ? j : k;
  ndtreal d = A[6 + k] * A[6 + j];
  for (int i = 0; i <= m; ++i)
  {
    d += A[indj + i] * A[indk + i];
  }
  return d;
}


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
inline __device__ void unpackedA(const NdtVoxel &ndt, ndtreal A[9], const ndtvec3 sample_to_mean)
{
  const ndtreal point_count = ndtreal(ndt.point_count);
  const ndtreal one_on_num_pt_plus_one = ndtreal(1) / (point_count + ndtreal(1));
  const ndtreal sc_1 = ndt.point_count ? sqrt(point_count * one_on_num_pt_plus_one) : ndtreal(1);
  const ndtreal sc_2 = one_on_num_pt_plus_one * sqrt(point_count);

  for (int i = 0; i < 6; ++i)
  {
    A[i] = sc_1 * ndt.cov_sqrt_diag[i];
  }

  for (int i = 0; i < 3; ++i)
  {
    A[i + 6] = sc_2 * sample_to_mean[i];
  }
}

// Find x for Mx = y, given lower triangular M where M is @c cov_sqrt_diag
// Storage order for M:
// 0 z z
// 1 2 z
// 3 4 5
inline __device__ ndtvec3 solveTriangular(const NdtVoxel &ndt, const ndtvec3 &y)
{
  // Note: if we generate the voxel with point on a perfect plane, say (0, 0, 1, 0), then do this operation,
  // we get a divide by zero. We avoid this by seeding the covariance matrix with an identity matrix scaled
  // by the sensor noise (see initialiseNdt()).
  ndtvec3 x;
  ndtreal d;

  d = y.x;
  x.x = d / ndt.cov_sqrt_diag[0];

  d = y.y;
  d -= ndt.cov_sqrt_diag[1 + 0] * x.x;
  x.y = d / ndt.cov_sqrt_diag[1 + 1];

  d = y.z;
  d -= ndt.cov_sqrt_diag[3 + 0] * x.x;
  d -= ndt.cov_sqrt_diag[3 + 1] * x.y;
  x.z = d / ndt.cov_sqrt_diag[3 + 2];

  return x;
}

inline __device__ ndtvec3 calculateHit(NdtVoxel *ndt_voxel, float *voxel_value, ndtvec3 sample, ndtvec3 voxel_mean,
                                       float hit_value, float occupancy_threshold_value, float uninitialised_value,
                                       float sensor_noise)
{
  const float initial_value = *voxel_value;
  const bool was_uncertain = initial_value == uninitialised_value;
  // Initialise the ndt_voxel data if this transitions the voxel to an occupied state.
  if (was_uncertain || initial_value <= occupancy_threshold_value)
  {
    // Transitioned to occupied. Initialise.
    initialiseNdt(ndt_voxel, sensor_noise);
    *voxel_value = hit_value;
  }
  else
  {
    *voxel_value += hit_value;
  }

  // This has been taken from example code provided by Jason Williams as a sample on storing and using covarance data
  // using a packed, diagonal.
  // Code represents covariance via square root matrix, i.e., covariance P = C * C^T
  // Let old covariance be P, new covariance Pnew, old mean mu, new point z
  // The required update for the covariance is
  //   Pnew = num_pt/(num_pt + 1)*P + num_pt/(num_pt+1)^2 * (z-mu)(z-mu)^T
  // This code implements that update directly via a matrix square root by forming the matrix A
  // such that A^T A = Pnew. A is not square, so a modified Gram-Schmidt decomposition is utilised
  // to find the triangular square root matrix Cnew such that Pnew = Cnew Cnew^T
  // Reference: Maybeck 1978 Stochastic Models, Estimation and Control, vol 1, p381
  // https://www.sciencedirect.com/bookseries/mathematics-in-science-and-engineering/vol/141/part/P1

  const ndtvec3 sample_to_mean = sample - voxel_mean;
  ndtreal A[9];
  unpackedA(*ndt_voxel, A, sample_to_mean);

  // Update covariance.
  for (int k = 0; k < 3; ++k)
  {
    const int ind1 = (k * (k + 3)) >> 1;  // packed index of (k,k) term
    const int indk = ind1 - k;            // packed index of (1,k)
    const ndtreal ak = sqrt(packedDot(A, k, k));
    ndt_voxel->cov_sqrt_diag[ind1] = float(ak);
    if (ak > 0)
    {
      const ndtreal aki = ndtreal(1) / ak;
      for (int j = k + 1; j < 3; ++j)
      {
        const int indj = (j * (j + 1)) >> 1;
        const int indkj = indj + k;
        ndtreal c = packedDot(A, j, k) * aki;
        ndt_voxel->cov_sqrt_diag[indkj] = float(c);
        c *= aki;
        A[j + 6] -= c * A[k + 6];
        for (int l = 0; l <= k; ++l)
        {
          A[indj + l] -= c * A[indk + l];
        }
      }
    }
  }

  // Update the mean sub-voxel pattern
  const ndtreal point_count = ndtreal(ndt_voxel->point_count);
  const ndtreal one_on_pt_count_plus_one = 1.0 / (point_count + 1.0);
  voxel_mean = (point_count * voxel_mean + sample) * one_on_pt_count_plus_one;
#if OHM_NDT_UNPACKED_MEAN
  ndt_voxel->mean[0] = voxel_mean[0];
  ndt_voxel->mean[1] = voxel_mean[1];
  ndt_voxel->mean[2] = voxel_mean[2];
#endif  //

  if (ndt_voxel->point_count < 0xffffffffu)
  {
    ++ndt_voxel->point_count;
  }

  return voxel_mean;
}

inline __device__ ndtvec3 calculateMiss(NdtVoxel *ndt_voxel, float *voxel_value, ndtvec3 sensor, ndtvec3 sample,
                                        ndtvec3 voxel_mean, float occupancy_threshold_value, float uninitialised_value,
                                        float miss_value, float sensor_noise, unsigned sample_threshold)
{
  if (*voxel_value == uninitialised_value)
  {
    // First touch of the voxel. Apply the miss value as is.
    // Same behaviour as OccupancyMap.
    *voxel_value = miss_value;
    return voxel_mean;
  }

  // Direct value adjustment if not occupied or insufficient samples.
  if (*voxel_value < occupancy_threshold_value || ndt_voxel->point_count < sample_threshold)
  {
    // Re-enforcement of free voxel or too few points to resolve a guassing. Use standard value update.
    // Add miss value, same behaviour as OccupancyMap.
    *voxel_value += miss_value;
    return voxel_mean;
  }

  // Update of an occupied voxel. We have to unpack the covariance and apply NDT logic.

  // Notes:
  // - Equation references are in relation to the paper on which this is based (see class comments).
  // - Variable subscripts are denoted by '_<subscript>'; e.g., "z subscript i" is written "z_i".
  // - A transpose is donoted by [T]
  // - Ordinals are denoted by [#]; e.g.,
  //    - [-1] -> inverse
  //    - [2] -> square
  // - The paper used capital Sigma for the covariance matrix. We use P.
  //
  // Goal is to calculate equation (24)
  // p(m_k = 1|z_i) = 0.5 - np(x_ML|N(u,P)) (1 - p(x_ML|z_i))      (24)
  // We have already established we have sufficient points for a gaussian.

  // p(x_ML|N(u,P)) ~ exp( -0.5(x_ML - u)[T] P[-1](x_ML - u))     (22)
  // Where know values are:
  //  - u existing mean voxel position (sub-voxel position)
  //  - P is the covariance matrix.
  //  - z_i is the sample
  // To be calcualated:
  // - x_ML

  // p(x_ML|z_i) ~ exp( -0.5 || x_ML - z_i ||[2] / s_s[2] )       (23)
  // Where:
  // - s_s is the sensor noise

  // x_ML = l.t + l_0                                             (25)
  // Know:
  // - l : sensor ray = (sample - sensor) / ||sample - sensor||
  // - l_0 : sensor position

  // t =  a_x b_x + a_y b_y + a_z b_z /                           (28)
  //      a_x l_x + a_y l_y + a_z l-z
  //
  // a = P[-1] l
  // b = (l_0 - u)

  const ndtvec3 sensor_to_sample = sample - sensor;
  const ndtvec3 sensor_ray = glm::normalize(sensor_to_sample);  // Verified
  const ndtvec3 sensor_to_mean = sensor - voxel_mean;

  // Packed data solutions:
  const ndtvec3 a = solveTriangular(*ndt_voxel, sensor_ray);
  const ndtvec3 b_norm = solveTriangular(*ndt_voxel, sensor_to_mean);

  // const ndtvec3 a = covariance_inv * sensor_ray;  // Verified (unpacked version)
  // (28)
  // const ndtreal t = glm::dot(a, sensor_to_mean) / glm::dot(a, sensor_ray); // Verified (unpacked version)
  const ndtreal t = -glm::dot(a, b_norm) / glm::dot(a, a);  // Verified

  // (25)
  // Note: maximum_likelyhood is abbreviated to ml in assoicated variable names.
  const ndtvec3 voxel_maximum_likelyhood = sensor_ray * t + sensor;  // Verified

  // (22)
  // Unverified: json line 264
  // const ndtreal p_x_ml_given_voxel = std::exp(
  //   -0.5 * glm::dot(voxel_maximum_likelyhood - voxel_mean, covariance_inv * (voxel_maximum_likelyhood -
  //   voxel_mean)));
  // Corrected:
  const ndtreal p_x_ml_given_voxel =
    exp(-0.5 * glm::length2(solveTriangular(*ndt_voxel, voxel_maximum_likelyhood - voxel_mean)));

  // (23)
  // Verified: json: line 263
  const ndtreal sensor_noise_variance = sensor_noise * sensor_noise;
  const ndtreal p_x_ml_given_sample =
    exp(-0.5 * glm::length2(voxel_maximum_likelyhood - sample) / sensor_noise_variance);

  // Set the scaling factor by converting the miss value to a probability.
  const ndtreal scaling_factor = 1.0f - (1.0 / (1.0 + exp(miss_value)));
  // Verified: json line 267
  const ndtreal probability_update = 0.5 - scaling_factor * p_x_ml_given_voxel * (1.0 - p_x_ml_given_sample);

  // Check for NaN
  // This should no longer be occurring.
  if (probability_update == probability_update)
  {
    // Convert the probability to a log value.
    *voxel_value += log(probability_update / (1.0 - probability_update));
  }

  return voxel_maximum_likelyhood;
}

#if !GPUTIL_DEVICE
/// Perform an eigen decomposition on the covariance dat ain @p ndt.
/// Requires compilation with the Eigen maths library or it always returns false.
bool ohm_API eigenDecomposition(const NdtVoxel &ndt, glm::dvec3 *eigenvalues, glm::dmat3 *eigenvectors);


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
}  // namespace ohm
#endif  // !GPUTIL_DEVICE

#endif  // NDTVOXEL_H
