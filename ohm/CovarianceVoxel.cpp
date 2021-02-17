// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmConfig.h"

// #define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

// Must come after glm includes due to usage on GPU.
#include "CovarianceVoxel.h"

#include "NdtMap.h"
#include "OccupancyMap.h"
#include "Voxel.h"
#include "VoxelMean.h"
#include "VoxelOccupancy.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_factorisation.hpp>

#ifdef OHM_WITH_EIGEN
#include <Eigen/Dense>
#endif  // OHM_WITH_EIGEN

#include <3esservermacros.h>
#ifdef TES_ENABLE
#include <3esserver.h>
#include <shapes/3esshapes.h>
#endif  // TES_ENABLE

#include <array>

namespace ohm
{
#if OHM_COV_DEBUG
namespace
{
unsigned max_iterations = 0;
double max_error = 0;
}  // namespace
#endif  // OHM_COV_DEBUG

namespace
{
#ifdef OHM_WITH_EIGEN
void covarianceEigenDecompositionEigen(const CovarianceVoxel *cov, glm::dmat3 *eigenvectors, glm::dvec3 *eigenvalues)
{
  // This has been noted to be ~3x faster than the GLM iterative version.
  const glm::dmat3 cov_mat = covarianceMatrix(cov);
  // Both GLM and Eigen are column major. Direct mapping is fine.
  const auto cov_eigen = Eigen::Matrix3d::ConstMapType(glm::value_ptr(cov_mat), 3, 3);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov_eigen);
  Eigen::Vector3d evals = Eigen::Vector3d::Ones();
  Eigen::Matrix3d evecs = Eigen::Matrix3d::Identity();

  if (eigensolver.info() == Eigen::Success)
  {
    evals = eigensolver.eigenvalues();
    evecs = eigensolver.eigenvectors();
  }
  const double det = evecs.determinant();
  if (det < 0.0)
  {
    evecs.col(0) = -evecs.col(0);  // must be valid rotation matrix (determinant=+1)
  }
  else if (det == 0.0)
  {
    evecs = Eigen::Matrix3d::Identity();
  }

  *eigenvalues = glm::dvec3(evals[0], evals[1], evals[2]);

  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
    {
      (*eigenvectors)[c][r] = evecs(r, c);
    }
  }
}
#else  // OHM_WITH_EIGEN
void covarianceEigenDecompositionGlm(const CovarianceVoxel *cov, glm::dmat3 *eigenvectors, glm::dvec3 *eigenvalues)
{
  // This has been noted to be ~3x slower than the Eigen solver.
  const glm::dmat3 mat = covarianceMatrix(cov);

  *eigenvectors = glm::dmat3(1.0);  // Identity initialisation

  // Use QR decomposition to implement the QR algorithm.
  // For this algorithm, we iterate as follows:
  //
  // P = cov * cov^T  // cov by cov transpose
  // for i = 1 to N
  //   Q, R = qr_decomposition(P)
  //   P = R * Q
  // end
  //
  // The eigenvalues converge in the diagonal of R.
  // Meanwhile the eigenvectors are found by the product all the Q matrices
  //
  // We set a hard iteration limit, but we also check for convergence with last iteration and may early out.
  const unsigned iteration_limit = 20;
  glm::dmat3 q, r;
  glm::dvec3 eigenvalues_last, eigenvalues_current(0);
  const glm::dvec3 delta_threshold(1e-9);
  for (unsigned i = 0; i < iteration_limit; ++i)
  {
    eigenvalues_last = eigenvalues_current;
#if OHM_COV_DEBUG
    max_iterations = std::max(max_iterations, i + 1);
#endif  // OHM_COV_DEBUG

    glm::qr_decompose(mat, q, r);
    // Progressively refine the eigenvectors.
    *eigenvectors = *eigenvectors * q;
    // Update eigenvalues and check for convergence
    eigenvalues_current[0] = r[0][0];
    eigenvalues_current[1] = r[1][1];
    eigenvalues_current[2] = r[2][2];

    mat = r * q;

    const glm::dvec3 eval_delta = glm::abs(eigenvalues_current - eigenvalues_last);
    if (glm::all(glm::lessThanEqual(eval_delta, delta_threshold)))
    {
      break;
    }
  }

#if OHM_COV_DEBUG
  const glm::dvec3 eval_delta = glm::abs(eigenvalues_current - eigenvalues_last);
  max_error = std::max(eval_delta.x, max_error);
  max_error = std::max(eval_delta.y, max_error);
  max_error = std::max(eval_delta.z, max_error);
#endif  // OHM_COV_DEBUG

  *eigenvalues = eigenvalues_current;
}
#endif  // OHM_WITH_EIGEN
}  // namespace

void covarianceEigenDecomposition(const CovarianceVoxel *cov, glm::dmat3 *eigenvectors, glm::dvec3 *eigenvalues)
{
#ifdef OHM_WITH_EIGEN
  covarianceEigenDecompositionEigen(cov, eigenvectors, eigenvalues);
#else   // OHM_WITH_EIGEN
  covarianceEigenDecompositionGlm(cov, eigenvectors, eigenvalues);
#endif  // OHM_WITH_EIGEN
}


void covarianceEstimatePrimaryNormal(const CovarianceVoxel *cov, glm::dvec3 *normal, int preferred_axis)
{
  glm::dmat3 eigenvectors;
  glm::dvec3 eigenvalues;

  covarianceEigenDecomposition(cov, &eigenvectors, &eigenvalues);

  int smallest_eval_index = preferred_axis;
  for (int i = 0; i < 3; ++i)
  {
    if (eigenvalues[i] < eigenvalues[smallest_eval_index])
    {
      smallest_eval_index = i;
    }
  }

  const double length2 = glm::length2(eigenvectors[smallest_eval_index]);
  // const double confidence = 1.0 - length2;
  *normal = (length2 > 0) ? eigenvectors[smallest_eval_index] / glm::sqrt(length2) : eigenvectors[smallest_eval_index];
  // return confidence;
}


bool covarianceUnitSphereTransformation(const CovarianceVoxel *cov, glm::dquat *rotation, glm::dvec3 *scale)
{
  glm::dmat3 eigenvectors;
  glm::dvec3 eigenvalues;

  covarianceEigenDecomposition(cov, &eigenvectors, &eigenvalues);

  const double det = glm::determinant(eigenvectors);
  if (det < 0.0)
  {
    eigenvectors[0] = -eigenvectors[0];  // must be valid rotation matrix (determinant=+1)
  }
  else if (det == 0.0)
  {
    eigenvectors = glm::dmat3(1.0);
  }

  *rotation = glm::dquat(eigenvectors);
  for (int i = 0; i < 3; ++i)
  {
    const double eval = std::abs(eigenvalues[i]);  // abs just in case.
    const double epsilon = 1e-9;
    (*scale)[i] = (eval > epsilon) ? std::sqrt(eval) : eval;
  }

  return true;
}


#if OHM_COV_DEBUG
#include <iostream>
void covDebugStats()
{
  std::cout << "QR algorithm max iterations: " << max_iterations << std::endl;
  std::cout << "QR algorithm max error: " << max_error << std::endl;
}
#endif  // OHM_COV_DEBUG

void integrateNdtHit(NdtMap &map, const Key &key, const glm::dvec3 &sensor, const glm::dvec3 &sample, bool ndt_tm,
                     const float sample_intensity)
{
  OccupancyMap &occupancy_map = map.map();
  Voxel<float> occupancy_voxel(&occupancy_map, occupancy_map.layout().occupancyLayer(), key);
  Voxel<VoxelMean> mean_voxel(occupancy_voxel, occupancy_map.layout().meanLayer());
  Voxel<CovarianceVoxel> cov_voxel(occupancy_voxel, occupancy_map.layout().covarianceLayer());
  const glm::dvec3 voxel_centre = occupancy_map.voxelCentreGlobal(key);

  assert(occupancy_voxel.isValid());
  assert(mean_voxel.isValid());
  assert(cov_voxel.isValid());

  // Keep clang analysis happy
#ifdef __clang_analyzer__
  if (!occupancy_voxel.voxelMemory() || !mean_voxel.voxelMemory() || !cov_voxel.voxelMemory())
  {
    return;
  }
#endif  // __clang_analyzer__

  // NDT-OM
  CovarianceVoxel cov;
  VoxelMean mean;
  float occupancy;
  occupancy_voxel.read(&occupancy);
  mean_voxel.read(&mean);
  cov_voxel.read(&cov);

  float updated_value = occupancy;
  const glm::dvec3 voxel_pos = position(mean, voxel_centre, occupancy_map.resolution());

  // NDT-TM
  if (ndt_tm)
  {
    Voxel<IntensityMeanCov> intensity_voxel(&occupancy_map, occupancy_map.layout().intensityLayer(), key);
    Voxel<HitMissCount> hit_miss_count_voxel(&occupancy_map, occupancy_map.layout().hitMissCountLayer(), key);
    IntensityMeanCov intensity;
    HitMissCount hit_miss_count;
    intensity_voxel.read(&intensity);
    hit_miss_count_voxel.read(&hit_miss_count);

    const bool reinitialise_permeability_with_covariance = true; // TODO: make a parameter of map
    calculateHitMissUpdateOnHit(&cov, updated_value, &hit_miss_count, sensor, sample, voxel_pos, mean.count,
                                unobservedOccupancyValue(), reinitialise_permeability_with_covariance,
                                map.adaptationRate(), map.sensorNoise(), map.reinitialiseCovarianceThreshold(),
                                map.reinitialiseCovariancePointCount(), map.ndtSampleThreshold());

    hit_miss_count_voxel.write(hit_miss_count);

    calculateIntensityUpdateOnHit(&intensity, updated_value, sample_intensity,
                                  map.initialIntensityCovariance(), mean.count, map.reinitialiseCovarianceThreshold(),
                                  map.reinitialiseCovariancePointCount());

    intensity_voxel.write(intensity);
  }

  if (calculateHitWithCovariance(&cov, &updated_value, sample, voxel_pos, mean.count, occupancy_map.hitValue(),
                                 unobservedOccupancyValue(), float(occupancy_map.resolution()),
                                 map.reinitialiseCovarianceThreshold(), map.reinitialiseCovariancePointCount()))
  {
    // Covariance matrix has reset. Reset the point count to clear the mean value.
    mean.count = 0;
  }

  // Ensure we update the occupancy within the configured map limits.
  occupancyAdjustUp(
    &occupancy, occupancy, updated_value, unobservedOccupancyValue(), occupancy_map.maxVoxelValue(),
    occupancy_map.saturateAtMinValue() ? occupancy_map.minVoxelValue() : std::numeric_limits<float>::lowest(),
    occupancy_map.saturateAtMaxValue() ? occupancy_map.maxVoxelValue() : std::numeric_limits<float>::max(), false);
  occupancy_voxel.write(occupancy);

  // Update the voxel mean.
  updatePosition(&mean, sample, voxel_centre, occupancy_map.resolution());
  mean_voxel.write(mean);
  cov_voxel.write(cov);
}


void integrateNdtMiss(NdtMap &map, const Key &key, const glm::dvec3 &sensor, const glm::dvec3 &sample, bool ndt_tm)
{
  OccupancyMap &occupancy_map = map.map();
  Voxel<float> occupancy_voxel(&occupancy_map, occupancy_map.layout().occupancyLayer(), key);
  Voxel<const VoxelMean> mean_voxel(occupancy_voxel, occupancy_map.layout().meanLayer());
  Voxel<const CovarianceVoxel> cov_voxel(occupancy_voxel, occupancy_map.layout().covarianceLayer());
  const glm::dvec3 voxel_centre = occupancy_map.voxelCentreGlobal(key);

  assert(occupancy_voxel.isValid());
  assert(mean_voxel.isValid());
  assert(cov_voxel.isValid());

  // Keep clang analysis happy
#ifdef __clang_analyzer__
  if (!occupancy_voxel.voxelMemory() || !mean_voxel.voxelMemory() || !cov_voxel.voxelMemory())
  {
    return;
  }
#endif  // __clang_analyzer__

  // NDT-OM
  CovarianceVoxel cov;
  VoxelMean mean;
  float occupancy;
  occupancy_voxel.read(&occupancy);
  mean_voxel.read(&mean);
  cov_voxel.read(&cov);

  float updated_value = occupancy;
#ifdef TES_ENABLE
  const float initial_value = occupancy;
  glm::dvec3 voxel_maximum_likelihood;
#endif  // TES_ENABLE
  const glm::dvec3 voxel_mean = position(mean, voxel_centre, occupancy_map.resolution());

  if (ndt_tm)
  {
    Voxel<HitMissCount> hit_miss_count_voxel(&occupancy_map, occupancy_map.layout().hitMissCountLayer(), key);
    HitMissCount hit_miss_count;
    hit_miss_count_voxel.read(&hit_miss_count);

#ifdef TES_ENABLE
    voxel_maximum_likelihood =
#endif  // TES_ENABLE
      calculateMissNdt(&cov, &updated_value, &hit_miss_count, sensor, sample, voxel_mean, mean.count,
                       unobservedOccupancyValue(), occupancy_map.missValue(), map.adaptationRate(), map.sensorNoise(),
                       map.ndtSampleThreshold());

    hit_miss_count_voxel.write(hit_miss_count);
  }
  else
  {
    HitMissCount hit_miss_count{ 0, 0 };
#ifdef TES_ENABLE
    voxel_maximum_likelihood =
#endif  // TES_ENABLE
      calculateMissNdt(&cov, &updated_value, &hit_miss_count, sensor, sample, voxel_mean, mean.count,
                       unobservedOccupancyValue(), occupancy_map.missValue(), map.adaptationRate(), map.sensorNoise(),
                       map.ndtSampleThreshold());
  }

  occupancyAdjustDown(
    &occupancy, occupancy, updated_value, unobservedOccupancyValue(), occupancy_map.minVoxelValue(),
    occupancy_map.saturateAtMinValue() ? occupancy_map.minVoxelValue() : std::numeric_limits<float>::lowest(),
    occupancy_map.saturateAtMaxValue() ? occupancy_map.maxVoxelValue() : std::numeric_limits<float>::max(), false);
  occupancy_voxel.write(occupancy);

#ifdef TES_ENABLE
  if (map.trace())
  {
    const glm::dvec3 voxel_centre = occupancy_map.voxelCentreGlobal(key);
    TES_BOX_W(g_tes, TES_COLOUR(OrangeRed), tes::Id(cov_voxel.voxelMemory()),
              tes::Transform(glm::value_ptr(voxel_centre), glm::value_ptr(glm::dvec3(occupancy_map.resolution()))));
    TES_SERVER_UPDATE(g_tes, 0.0f);

    bool drew_surfel = false;
    glm::dquat rot;
    glm::dvec3 scale;
    if (covarianceUnitSphereTransformation(&cov, &rot, &scale))
    {
      TES_SPHERE(g_tes, TES_COLOUR(SeaGreen), tes::Id(cov_voxel.voxelMemory()),
                 tes::Transform(tes::Vector3d(glm::value_ptr(voxel_mean)), tes::Quaterniond(rot.x, rot.y, rot.z, rot.w),
                                tes::Vector3d(glm::value_ptr(scale))));
      drew_surfel = true;
    }

    // Trace the voxel mean, maximum likelihood point and the ellipsoid.
    // Mean
    const float mean_pos_radius = 0.05f;
    const float likely_pos_radius = 0.1f;
    TES_SPHERE(g_tes, TES_COLOUR(OrangeRed), tes::Id(&voxel_mean),
               tes::Spherical(glm::value_ptr(voxel_mean), mean_pos_radius));
    // Maximum likelihood
    TES_SPHERE_W(g_tes, TES_COLOUR(PowderBlue), tes::Id(&voxel_maximum_likelihood),
                 tes::Spherical(glm::value_ptr(voxel_maximum_likelihood), likely_pos_radius));

    std::array<char, 64> text;  // NOLINT(readability-magic-numbers)
    text[0] = '\0';
    sprintf(text.data(), "P %.3f", valueToProbability(occupancy - initial_value));
    TES_TEXT2D_WORLD(g_tes, TES_COLOUR(White), text.data(), tes::Id(),
                     tes::Spherical(tes::Vector3d(glm::value_ptr(voxel_centre))));

    TES_SERVER_UPDATE(g_tes, 0.0f);
    TES_BOX_END(g_tes, tes::Id(cov_voxel.voxelMemory()));
    TES_SPHERE_END(g_tes, tes::Id(&voxel_mean));
    TES_SPHERE_END(g_tes, tes::Id(&voxel_maximum_likelihood));
    if (drew_surfel)
    {
      TES_SPHERE_END(g_tes, tes::Id(cov_voxel.voxelMemory()));
    }
  }
#endif  // TES_ENABLE
}
}  // namespace ohm
