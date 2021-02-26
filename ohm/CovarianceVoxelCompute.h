// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef COVARIANCEVOXELCOMPUTE_H
#define COVARIANCEVOXELCOMPUTE_H

// Note: this header is included in GPU code.

#if GPUTIL_DEVICE
// Define GPU type aliases
typedef float3 CovVec3;
typedef float CovReal;
typedef uint uint32_t;

// Vector support functions
inline __device__ CovReal covdot(const CovVec3 a, const CovVec3 b)
{
  return dot(a, b);
}
inline __device__ CovReal covlength2(const CovVec3 v)
{
  return dot(v, v);
}
inline __device__ CovVec3 covnormalize(const CovVec3 v)
{
  return normalize(v);
}

#else  // GPUTIL_DEVICE
// Define CPU type aliases
using CovVec3 = glm::dvec3;
using CovReal = double;


inline CovReal covdot(const CovVec3 &a, const CovVec3 &b)
{
  return glm::dot(a, b);
}
inline CovReal covlength2(const CovVec3 &v)
{
  return glm::length2(v);
}
inline CovVec3 covnormalize(const CovVec3 &v)
{
  return glm::normalize(v);
}
#ifndef __device__
#define __device__
#endif  //  __device__
#endif  // GPUTIL_DEVICE

/// @ingroup voxelcovariance
/// Defines the covariance voxel structure. This is a modified covariance matrix packed to reduce memory storage.
typedef struct CovarianceVoxel_t  // NOLINT(readability-identifier-naming, modernize-use-using)
{
  /// Trianglar square root covariance matrix. Represents a covariance matrix via the triangular
  /// square root matrix, P = S * S^T.
  /// | cov[0]  |      .  |      .  |
  /// | cov[1]  | cov[2]  |      .  |
  /// | cov[3]  | cov[4]  | cov[5]  |
  float trianglar_covariance[6];  // NOLINT(readability-magic-numbers, modernize-avoid-c-arrays)
} CovarianceVoxel;

typedef struct IntensityMeanCov_t  // NOLINT(readability-identifier-naming, modernize-use-using)
{
  float intensity_mean;
  float intensity_cov;
} IntensityMeanCov;

typedef struct HitMissCount_t  // NOLINT(readability-identifier-naming, modernize-use-using)
{
  uint32_t hit_count;
  uint32_t miss_count;
} HitMissCount;

/// @ingroup voxelcovariance
/// Initialise the packed square root covariance matrix in @p cov
/// The covariance value is initialised to an identity matrix, scaled by the @p voxel_resolution .
/// This is to ensure we do not start with a zero matrix, which causes all sorts of mathematical problems.
/// @param[out] cov The @c CovarianceVoxel to initialse.
/// @param voxel_resolution The voxel resolution, which is used to seed the covariance.
inline __device__ void initialiseCovariance(CovarianceVoxel *cov, float voxel_resolution)
{
  const float covariance_scale_factor = 0.1f;
  // Initialise the square root covariance matrix to a scaled identity matrix.
  // NOLINTNEXTLINE(readability-magic-numbers)
  cov->trianglar_covariance[0] = cov->trianglar_covariance[2] = cov->trianglar_covariance[5] =
    covariance_scale_factor * voxel_resolution;
  cov->trianglar_covariance[1] = cov->trianglar_covariance[3] = cov->trianglar_covariance[4] = 0;
}

/// @ingroup voxelcovariance
/// dot product of j-th and k-th columns of A
/// A is (4,3), assumed to be packed as follows, where z is non-represented zero
/// 0 1 3
/// z 2 4
/// z z 5
/// 6 7 8
inline __device__ double packedDot(const CovReal A[9],  // NOLINT
                                   const int j, const int k)
{
  const int col_first_el[] = { 0, 1, 3 };  // NOLINT(modernize-avoid-c-arrays)
  const int indj = col_first_el[j];
  const int indk = col_first_el[k];
  const int m = (j <= k) ? j : k;
  CovReal d = A[6 + k] * A[6 + j];  // // NOLINT(readability-magic-numbers)
  for (int i = 0; i <= m; ++i)
  {
    d += A[indj + i] * A[indk + i];
  }
  return d;
}


/// @ingroup voxelcovariance
/// Unpack the covariance matrix storage.
///
/// Unpacks covariance matrix square root and mean into the following sparse 3,4 form:
///
/// |         |         |         |
/// | ------- | ------- | ------- |
/// | cov[0]  | cov[1]  | cov[3]  |
/// | .       | cov[2]  | cov[4]  |
/// | .       | .       | cov[5]  |
/// | mean[0] | mean[1] | mean[2] |
///
/// With corresponding layout in the output @p matrix.
///
/// |   |   |   |
/// | - | - | - |
/// | 0 | 1 | 3 |
/// | . | 2 | 4 |
/// | . | . | 5 |
/// | 6 | 7 | 8 |
///
/// Items marked `cov[n]` are extracted from the @c cov->trianglar_covariance, while `mean[n]` items are derived from
/// @p sample_to_mean . Items marked '.' are not represented in the martix and are treated as zero.
/// Note that the extracted values also have a co-efficient applied based on the @p point_count .
///
/// @param cov Covariance details of the voxel in question.
/// @param point_count Number of samples which have been used to generate the @c cov and voxel mean.
/// @param sample_to_mean The difference between the new sample point and the voxel mean.
/// @param[out] matrix The matrix to unpack to. This is an array of 9 elements.
inline __device__ void unpackCovariance(const CovarianceVoxel *cov, unsigned point_count, const CovVec3 sample_to_mean,
                                        CovReal *matrix)
{
  // NOLINTNEXTLINE(google-readability-casting)
  const CovReal one_on_num_pt_plus_one = (CovReal)1 / (point_count + (CovReal)1);
  // NOLINTNEXTLINE(google-readability-casting)
  const CovReal sc_1 = point_count ? sqrt(point_count * one_on_num_pt_plus_one) : (CovReal)1;
  // NOLINTNEXTLINE(google-readability-casting)
  const CovReal sc_2 = one_on_num_pt_plus_one * sqrt((CovReal)point_count);

  for (int i = 0; i < 6; ++i)  // NOLINT(readability-magic-numbers)
  {
    matrix[i] = sc_1 * cov->trianglar_covariance[i];
  }

  matrix[0 + 6] = sc_2 * sample_to_mean.x;  // NOLINT(readability-magic-numbers)
  matrix[1 + 6] = sc_2 * sample_to_mean.y;  // NOLINT(readability-magic-numbers)
  matrix[2 + 6] = sc_2 * sample_to_mean.z;  // NOLINT(readability-magic-numbers)
}

/// @ingroup voxelcovariance
/// Find x for Mx = y, given lower triangular M where M is @c trianglar_covariance
/// Storage order for M:
///
/// @code{.unparsed}
/// 0 z z
/// 1 2 z
/// 3 4 5
/// @endcode
/// @param cov Covariance details of the voxel in question.
/// @param y The vector to solve for.
inline __device__ CovVec3 solveTriangular(const CovarianceVoxel *cov, const CovVec3 y)
{
  // Note: if we generate the voxel with point on a perfect plane, say (0, 0, 1, 0), then do this operation,
  // we get a divide by zero. We avoid this by seeding the covariance matrix with an identity matrix scaled
  // by the sensor noise (see initialiseCovariance()).
  CovVec3 x;
  CovReal d;

  d = y.x;
  x.x = d / cov->trianglar_covariance[0];

  d = y.y;
  d -= cov->trianglar_covariance[1 + 0] * x.x;
  x.y = d / cov->trianglar_covariance[1 + 1];

  d = y.z;
  d -= cov->trianglar_covariance[3 + 0] * x.x;
  d -= cov->trianglar_covariance[3 + 1] * x.y;
  x.z = d / cov->trianglar_covariance[3 + 2];

  return x;
}

/// @ingroup voxelcovariance
/// Calculate the two likelihoods required for Normalised Distribution Transform (NDT) update logic.
///
/// This algorithm is based on the following paper:
/// > 3D normal distributions transform occupancy maps: An efficient representation for mapping in dynamic
/// > environments
/// > Jari P. Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal
///
/// This improves the probably adjustment for a voxel using the voxel covariance (if present). This only takes effect
/// when there have been samples collected for the voxel and `point_count &gt; 0`. The standard occupancy adjustment
/// is used whenever the `point_count &lt; sample_threshold`, with @p miss_value added to @p voxel_value or
/// @p voxel_value set to @p miss_value when @p voxel_value equals @p uninitialised_value .
///
/// @param cov_voxel The packed covariance for the voxel.
/// @param sensor The location of the sensor from where the sample was taken.
/// @param sample The sample position.
/// @param voxel_mean The current accumulated mean position within the voxel. Only valid if @p point_count is &gt; 0.
/// @param sensor_noise The sensor range noise error (standard deviation). Must be greater than zero.
/// @param[out] p_x_ml_given_voxel is the likelihood p(x_ML|N(u,P))
/// @param[out] p_x_ml_given_sample is the likelihood p(x_ML|z_i)
/// @return The point where the ray has the highest likelihood in the voxel Gaussian, x_ML
inline __device__ CovVec3 calculateSampleLikelihoods(const CovarianceVoxel *cov_voxel, CovVec3 sensor, CovVec3 sample,
                                                     CovVec3 voxel_mean, float sensor_noise,
                                                     CovReal *p_x_ml_given_voxel, CovReal *p_x_ml_given_sample)
{
  const CovVec3 sensor_to_sample = sample - sensor;
  const CovVec3 sensor_ray = covnormalize(sensor_to_sample);  // Verified
  const CovVec3 sensor_to_mean = sensor - voxel_mean;

  // Packed data solutions:
  const CovVec3 a = solveTriangular(cov_voxel, sensor_ray);
  const CovVec3 b_norm = solveTriangular(cov_voxel, sensor_to_mean);

  // const CovVec3 a = covariance_inv * sensor_ray;  // Verified (unpacked version)
  // (28)
  // const CovReal t = covdot(a, sensor_to_mean) / covdot(a, sensor_ray); // Verified (unpacked version)
  const CovReal t = -covdot(a, b_norm) / covdot(a, a);  // Verified

  // (25)
  // Note: maximum_likelihood is abbreviated to ml in associated variable names.
  const CovVec3 voxel_maximum_likelihood = sensor_ray * t + sensor;  // Verified

  // (22)
  // Unverified: json line 264
  // const CovReal p_x_ml_given_voxel = std::exp(
  //   -0.5 * covdot(voxel_maximum_likelihood - voxel_mean, covariance_inv * (voxel_maximum_likelihood -
  //   voxel_mean)));
  // Corrected:
  *p_x_ml_given_voxel = exp(-0.5 * covlength2(solveTriangular(cov_voxel, voxel_maximum_likelihood - voxel_mean)));

  // (23)
  // Verified: json: line 263
  const CovReal sensor_noise_variance = sensor_noise * sensor_noise;
  *p_x_ml_given_sample = exp(-0.5 * covlength2(voxel_maximum_likelihood - sample) / sensor_noise_variance);

  return voxel_maximum_likelihood;
}


/// @ingroup voxelcovariance
/// Calculate a voxel hit with packed covariance. This supports Normalised Distribution Transform (NDT) logic in
/// @c calculateHitNdt() .
///
/// The square root covariance in @p cov_voxel and occupancy in @p voxel_value are both updated, but the voxel mean
/// calculation is not performed here. However, it is expected that the voxel mean will be updated after this call and
/// the @c point_count incremented, otherwise future behaviour is undefined.
///
/// The @p cov_voxel may be zero and is fully initialised when the @p point_count is zero, implying this is the first
/// hit. It will also be reinitialised whenever the @p voxel_value is below the the @p reinitialise_threshold and the
/// @p point_count is above @p reinitialise_sample_count .
///
/// This reinitialisation is to handle situations where a voxel may have been occupied by a transient object, become
/// free, then becomes occupied once more. In this case, the new occupancy covariance may differ and should disregard
/// the previous covariance and mean. The @c reinitialise_threshold is used as the primary trigger to indicate previous
/// data may be invalid while the @c reinitialise_sample_count is intended to prevent repeated reintialisation as the
/// probablity value may oscillate around the threshold.
///
/// @param[in,out] cov_voxel The packed covariance to update for the voxel being updated.
/// @param[in,out] voxel_value The occupancy value for the voxel being updated.
/// @param sample The sample which falls in this voxel.
/// @param voxel_mean The current accumulated mean position within the voxel. Only valid if @p point_count is &gt; 0.
/// @param point_count The number of samples which have been used to generate the @p voxel_mean and @p cov_voxel.
/// @param hit_value The log probability value increase for occupancy on a hit. This must be greater than zero to
/// increase the voxel occupancy probability.
/// @param uninitialised_value The @p voxel_value for an uncertain voxel - one which has yet to be observed.
/// @param voxel_resolution The voxel size along each cubic edge.
/// @param reinitialise_threshold @p voxel_value threshold below which the covariance and mean should reset.
/// @param reinitialise_sample_count The @p point_count required to allow @c reinitialise_threshold to be triggered.
/// @return True if the covariance value is re-initialised. This should be used as a signal to diregard the current
///     @p voxel_mean and @c point_count and restart accumulating those values.
inline __device__ bool calculateHitWithCovariance(CovarianceVoxel *cov_voxel, float *voxel_value, CovVec3 sample,
                                                  CovVec3 voxel_mean, unsigned point_count, float hit_value,
                                                  float uninitialised_value, float voxel_resolution,
                                                  float reinitialise_threshold, unsigned reinitialise_sample_count)
{
  const float initial_value = *voxel_value;
  const bool was_uncertain = initial_value == uninitialised_value;
  bool initialised_covariance = false;

  if (point_count == 0 || (initial_value < reinitialise_threshold && point_count >= reinitialise_sample_count))
  {
    initialiseCovariance(cov_voxel, voxel_resolution);
    initialised_covariance = true;
    // Reinitialising co-variance. We need to ensure the point count is zero when we reset the covariance to
    // correctly calculate the covariance.
    point_count = 0;
  }

  // Initialise the cov_voxel data if this transitions the voxel to an occupied state (the voxel may have already had
  // misses so we could have was_uncertain==false && point_count==0).
  *voxel_value = (!was_uncertain) ? hit_value + initial_value : hit_value;

  // Code represents covariance via square root matrix, i.e., covariance P = C * C^T
  // Let old covariance be P, new covariance Pnew, old mean mu, new point z
  // The required update for the covariance is
  //   Pnew = num_pt/(num_pt + 1)*P + num_pt/(num_pt+1)^2 * (z-mu)(z-mu)^T
  // This code implements that update directly via a matrix square root by forming the matrix A
  // such that A^T A = Pnew. A is not square, so a modified Gram-Schmidt decomposition is utilised
  // to find the triangular square root matrix Cnew such that Pnew = Cnew Cnew^T
  // Reference: Maybeck 1978 Stochastic Models, Estimation and Control, vol 1, p381
  // https://www.sciencedirect.com/bookseries/mathematics-in-science-and-engineering/vol/141/part/P1

  // sample_to_mean should be zero when we (re)initialise the covariance.
  // We use sample - sample rather than a hard zero due to API differences between CPU and GPU code.
  const CovVec3 sample_to_mean = (!initialised_covariance) ? sample - voxel_mean :
  // clang-format off
#if !defined(GPUTIL_DEVICE) || GPUTIL_DEVICE == 0
                                                             CovVec3(0, 0, 0)
#else   // !defined(GPUTIL_DEVICE) || GPUTIL_DEVICE == 0
                                                             make_float3(0, 0, 0)
#endif  // !defined(GPUTIL_DEVICE) || GPUTIL_DEVICE == 0
  ;
  // clang-format on
  CovReal unpacked_covariance[9];  // NOLINT(readability-magic-numbers, modernize-avoid-c-arrays)
  unpackCovariance(cov_voxel, point_count, sample_to_mean, unpacked_covariance);

  // Update covariance.
  for (int k = 0; k < 3; ++k)
  {
    // NOLINTNEXTLINE(hicpp-signed-bitwise)
    const int ind1 = (k * (k + 3)) >> 1;  // packed index of (k,k) term
    const int indk = ind1 - k;            // packed index of (1,k)
    const CovReal ak = sqrt(packedDot(unpacked_covariance, k, k));
    cov_voxel->trianglar_covariance[ind1] = (float)ak;  // NOLINT(google-readability-casting)
    if (ak > 0)
    {
      const CovReal aki = (CovReal)1 / ak;  // NOLINT(google-readability-casting)
      for (int j = k + 1; j < 3; ++j)
      {
        const int indj = (j * (j + 1)) >> 1;  // NOLINT(hicpp-signed-bitwise)
        const int indkj = indj + k;
        CovReal c = packedDot(unpacked_covariance, j, k) * aki;
        cov_voxel->trianglar_covariance[indkj] = (float)c;  // NOLINT(google-readability-casting)
        c *= aki;
        unpacked_covariance[j + 6] -= c * unpacked_covariance[k + 6];  // NOLINT(readability-magic-numbers)
        for (int l = 0; l <= k; ++l)
        {
          unpacked_covariance[indj + l] -= c * unpacked_covariance[indk + l];
        }
      }
    }
  }

  return initialised_covariance;
}

/// @ingroup voxelcovariance
/// Calculate the update to a voxel's intensity distribution using a new hit. This supports Normalised Distribution
/// Transform-Traversability Map (NDT-TM) logic in @c calculateHitNdt() .
///
/// @param[in,out] intensity_mean_voxel The mean intensity for the voxel being updated.
/// @param[in,out] intensity_cov_voxel The covariance of intensity for the voxel being updated.
/// @param voxel_value The occupancy value for the voxel being updated.
/// @param intensity_sample The intensity measurement of the sample that falls in this voxel.
/// @param initial_intensity_covariance The covariance to utilise in voxel when initialising with a new sample.
/// @param point_count The number of samples which have been used to generate the @p intensity_mean_voxel and @p
/// intensity_cov_voxel.
/// @param reinitialise_threshold @p voxel_value threshold below which the covariance and mean should reset.
/// @param reinitialise_sample_count The @p point_count required to allow @c reinitialise_threshold to be triggered.
/// @return True if the covariance value is re-initialised. This should be used as a signal to diregard the current
///     @p voxel_mean and @c point_count and restart accumulating those values.
inline __device__ void calculateIntensityUpdateOnHit(IntensityMeanCov *intensity, float voxel_value,
                                                     float intensity_sample, float initial_intensity_covariance,
                                                     unsigned point_count, float reinitialise_threshold,
                                                     unsigned reinitialise_sample_count)
{
  // Branchless for better GPU execution.
  const float initial_value = voxel_value;
  const bool needs_reset =
    point_count == 0 || (initial_value < reinitialise_threshold && point_count >= reinitialise_sample_count);
  const float delta = intensity->intensity_mean - intensity_sample;
  const float point_count_float = (float)point_count;  // NOLINT
  float inv_point_count_plus_one = 1.0f / (point_count_float + 1.0f);
  intensity->intensity_mean =
    (!needs_reset) ?  //
      inv_point_count_plus_one * (point_count_float * intensity->intensity_mean + intensity_sample) :
      intensity_sample;
  intensity->intensity_cov = (!needs_reset) ?  //
                               inv_point_count_plus_one * (point_count_float * intensity->intensity_cov +
                                                           inv_point_count_plus_one * delta * delta) :
                               initial_intensity_covariance;
}


/// @ingroup voxelcovariance
/// Update voxel hit/miss count upon hit. This supports Normalised Distribution Transform-Traversability Map (NDT-TM)
/// logic in @c calculateMissNdt() . The update is nontrivial as it selectively updates hit or miss depending on whether
/// the return is consistent with the Gaussian already in the voxel. Assumes  @c calculateHitWithCovariance() has just
/// been called.
///
/// The @p cov_voxel may be zero and is fully initialised when the @p point_count is zero, implying this is the first
/// hit. It will also be reinitialised whenever the @p voxel_value is below the the @p reinitialise_threshold and the
/// @p point_count is above @p reinitialise_sample_count .
///
/// This reinitialisation is to handle situations where a voxel may have been occupied by a transient object, become
/// free, then becomes occupied once more. In this case, the new occupancy covariance may differ and should disregard
/// the previous covariance and mean. The @c reinitialise_threshold is used as the primary trigger to indicate previous
/// data may be invalid while the @c reinitialise_sample_count is intended to prevent repeated reinitialisation as the
/// probability value may oscillate around the threshold.
///
/// @param[in,out] cov_voxel The packed covariance to update for the voxel being updated.
/// @param voxel_value The occupancy value for the voxel being updated.
/// @param[in,out] hit_count The number of hits in the current voxel.
/// @param[in,out] miss_count The number of misses in the current voxel.
/// @param sample The sample which falls in this voxel.
/// @param voxel_mean The current accumulated mean position within the voxel. Only valid if @p point_count is &gt; 0.
/// @param point_count The number of samples which have been used to generate the @p voxel_mean and @p cov_voxel.
/// @param uninitialised_value The @p voxel_value for an uncertain voxel - one which has yet to be observed.
/// @param reinitialise_permeability_with_covariance If true the voxel permeability is reinitialised any time the
/// covariance is reinitialised. Otherwise, it is only initialised on first observation.
/// @param adaptation_rate The adaptation rate for ellipsoid intersections. This value is given [0, 1], but it's
/// usage sees it scaled [0, 0.5].
/// @param sensor_noise The sensor range noise error (standard deviation). Must be greater than zero.
/// @param reinitialise_threshold @p voxel_value threshold below which the covariance and mean should reset.
/// @param reinitialise_sample_count The @p point_count required to allow @c reinitialise_threshold to be triggered.
/// @param sample_threshold The @p point_count required before using NDT logic, i.e., before the covariance value is
/// usable.
inline __device__ void calculateHitMissUpdateOnHit(CovarianceVoxel *cov_voxel, float voxel_value,
                                                   HitMissCount *hit_miss_count, CovVec3 sensor, CovVec3 sample,
                                                   CovVec3 voxel_mean, unsigned point_count, float uninitialised_value,
                                                   bool reinitialise_permeability_with_covariance,
                                                   float adaptation_rate, float sensor_noise,
                                                   float reinitialise_threshold, unsigned reinitialise_sample_count,
                                                   unsigned sample_threshold)
{
  // Branchless for better GPU execution
  const bool needs_reset =
    voxel_value == uninitialised_value ||
    (reinitialise_permeability_with_covariance &&
     (point_count == 0 || (voxel_value < reinitialise_threshold && point_count >= reinitialise_sample_count)));

  const unsigned initial_hit = (!needs_reset) ? hit_miss_count->hit_count : 0;
  const unsigned initial_miss = (!needs_reset) ? hit_miss_count->miss_count : 0;

  CovReal p_x_ml_given_voxel;
  CovReal p_x_ml_given_sample;
  calculateSampleLikelihoods(cov_voxel, sensor, sample, voxel_mean, sensor_noise, &p_x_ml_given_voxel,
                             &p_x_ml_given_sample);
  const CovReal prod = p_x_ml_given_voxel * p_x_ml_given_sample;
  const CovReal eta = (CovReal)0.5 * adaptation_rate;  // NOLINT

  // Logically we should yeild the following results:
  // 1. needs_reset is true:
  //    - initial_hit = 0
  //    - initial_miss = 0
  //    => hit_count = 0
  //    => miss_count = 0
  // 2. needs_reset is false and point_count < sample_threshold
  //    - initial_hit = hit_miss_count->hit_count
  //    - initial_miss = hit_miss_count->miss_count
  //    - hit_count increment_condition is true
  //    - miss_count increment_condition is false
  //    -> ++hit_count
  // 3. needs_reset is false and point_count >= sample_threshold
  //    - initial_hit = hit_miss_count->hit_count
  //    - initial_miss = hit_miss_count->miss_count
  //    - hit_count increment_condition true if (prod >= eta)
  //    - miss_count increment_condition true if (prod < eta && p_x_ml_given_voxel >= eta)
  //    -> hit_count may increment
  //    -> miss_count may increment

  hit_miss_count->hit_count = initial_hit + (!needs_reset && (point_count < sample_threshold || prod >= eta));
  hit_miss_count->miss_count =
    initial_miss + (!needs_reset && point_count >= sample_threshold && prod < eta && p_x_ml_given_voxel >= eta);
}


/// @ingroup voxelcovariance
/// Calculate a voxel miss (ray passthrough) using Normalised Distribution Transform (NDT) logic.
///
/// This algorithm is based on the following paper:
/// > 3D normal distributions transform occupancy maps: An efficient representation for mapping in dynamic
/// > environments
/// > Jari P. Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal
///
/// This improves the probably adjustment for a voxel using the voxel covariance (if present). This only takes effect
/// when there have been samples collected for the voxel and `point_count &gt; 0`. The standard occupancy adjustment
/// is used whenever the `point_count &lt; sample_threshold`, with @p miss_value added to @p voxel_value or
/// @p voxel_value set to @p miss_value when @p voxel_value equals @p uninitialised_value .
///
/// @param cov_voxel The packed covariance for the voxel. Only used if the `point_count &gt; sample_threshold`.
/// @param[in,out] voxel_value The current voxel log probably value.
/// @param[in,out] hit_count The number of hits in the current voxel.
/// @param[in,out] miss_count The number of misses in the current voxel.
/// @param sensor The location of the sensor from where the sample was taken.
/// @param sample The sample position.
/// @param voxel_mean The current accumulated mean position within the voxel. Only valid if @p point_count is &gt; 0.
/// @param point_count The number of samples which have been used to generate the @p voxel_mean and @p cov_voxel.
/// @param uninitialised_value The @p voxel_value for an uncertain voxel - one which has yet to be observed.
/// @param miss_value The @p voxel_value adjustment to apply from a miss. This must be less than zero to decrease the
/// occupancy probability. The NDT scaling factor is also derived from this value.
/// @param adaptation_rate The adaptation rate for ellipsoid intersections. This value is given [0, 1], but it's
/// usage sees it scaled [0, 0.5].
/// @param sensor_noise The sensor range noise error (standard deviation). Must be greater than zero.
/// @param sample_threshold The @p point_count required before using NDT logic, i.e., before the covariance value is
/// usable.
/// @return The point where the ray has the highest likelihood in the voxel Gaussian, x_ML
inline __device__ CovVec3 calculateMissNdt(const CovarianceVoxel *cov_voxel, float *voxel_value,
                                           HitMissCount *hit_miss_count, CovVec3 sensor, CovVec3 sample,
                                           CovVec3 voxel_mean, unsigned point_count, float uninitialised_value,
                                           float miss_value, float adaptation_rate, float sensor_noise,
                                           unsigned sample_threshold)
{
  if (*voxel_value == uninitialised_value)
  {
    // First touch of the voxel. Apply the miss value as is.
    // Same behaviour as OccupancyMap.
    *voxel_value = miss_value;
    hit_miss_count->hit_count = 0;
    hit_miss_count->miss_count = 1;
    return voxel_mean;
  }

  // Direct value adjustment if not occupied or insufficient samples.
  if (point_count < sample_threshold)
  {
    // Re-enforcement of free voxel or too few points to resolve a Gaussian. Use standard value update.
    // Add miss value, same behaviour as OccupancyMap.
    *voxel_value += miss_value;
    ++hit_miss_count->miss_count;
    return voxel_mean;
  }

  // Update of an occupied voxel. We have to unpack the covariance and apply NDT logic.

  // Notes:
  // - Equation references are in relation to the paper on which this is based (see class comments).
  // - Variable subscripts are denoted by '_<subscript>'; e.g., "z subscript i" is written "z_i".
  // - A transpose is denoted by [T]
  // - Ordinals are denoted by [#]; e.g.,
  //    - [-1] -> inverse
  //    - [2] -> square
  // - The paper used capital Sigma for the covariance matrix. We use P.
  //
  // Goal is to calculate equation (24)
  // p(m_k = 1|z_i) = 0.5 - eta p(x_ML|N(u,P)) (1 - p(x_ML|z_i))  (24)
  // We have already established we have sufficient points for a Gaussian.

  // p(x_ML|N(u,P)) ~ exp( -0.5(x_ML - u)[T] P[-1](x_ML - u))     (22)
  // Where know values are:
  //  - u existing mean voxel position (voxel mean position)
  //  - P is the covariance matrix.
  //  - z_i is the sample
  // To be calculated:
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

  CovReal p_x_ml_given_voxel;
  CovReal p_x_ml_given_sample;
  const CovVec3 voxel_maximum_likelihood = calculateSampleLikelihoods(
    cov_voxel, sensor, sample, voxel_mean, sensor_noise, &p_x_ml_given_voxel, &p_x_ml_given_sample);

  const CovReal scaling_factor = 0.5 * adaptation_rate;
  // Verified: json line 267
  const CovReal prod = p_x_ml_given_voxel * (1.0 - p_x_ml_given_sample);
  const CovReal probability_update = 0.5 - scaling_factor * prod;

  // NDT-TM update of miss count
  hit_miss_count->miss_count += prod < scaling_factor ? 1 : 0;

  // Check for NaN
  // This should no longer be occurring.
  if (probability_update == probability_update)
  {
    // Convert the probability to a log value.
    *voxel_value +=
      (float)log(probability_update / ((CovReal)1 - probability_update));  // NOLINT(google-readability-casting)
  }

  return voxel_maximum_likelihood;
}

#endif  // COVARIANCEVOXEL_H
