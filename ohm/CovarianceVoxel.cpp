// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmConfig.h"

// #define GLM_ENABLE_EXPERIMENTAL
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtx/norm.hpp>

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

using namespace ohm;

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

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov_eigen * cov_eigen.transpose());
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
    const glm::dmat3 cov_mat = covarianceMatrix(cov);
    glm::dmat3 mat_p = cov_mat * glm::transpose(cov_mat);

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

      glm::qr_decompose(mat_p, q, r);
      // Progressively refine the eigenvectors.
      *eigenvectors = *eigenvectors * q;
      // Update eigenvalues and check for convergence
      eigenvalues_current[0] = r[0][0];
      eigenvalues_current[1] = r[1][1];
      eigenvalues_current[2] = r[2][2];

      mat_p = r * q;

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

void ohm::covarianceEigenDecomposition(const CovarianceVoxel *cov, glm::dmat3 *eigenvectors, glm::dvec3 *eigenvalues)
{
#ifdef OHM_WITH_EIGEN
  covarianceEigenDecompositionEigen(cov, eigenvectors, eigenvalues);
#else   // OHM_WITH_EIGEN
  covarianceEigenDecompositionGlm(cov, eigenvectors, eigenvalues);
#endif  // OHM_WITH_EIGEN
}


void ohm::covarianceEstimatePrimaryNormal(const CovarianceVoxel *cov, glm::dvec3 *normal, int preferred_axis)
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


bool ohm::covarianceUnitSphereTransformation(const CovarianceVoxel *cov, glm::dquat *rotation, glm::dvec3 *scale)
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
    (*scale)[i] = (eval > 1e-9) ? std::sqrt(eval) : eval;
  }

  return true;
}


#if OHM_COV_DEBUG
#include <iostream>
void ohm::covDebugStats()
{
  std::cout << "QR algorithm max iterations: " << max_iterations << std::endl;
  std::cout << "QR algorithm max error: " << max_error << std::endl;
}
#endif  // OHM_COV_DEBUG

void ohm::integrateNdtHit(NdtMap &map, const Key &key, const glm::dvec3 &sample)
{
  OccupancyMap &occupancy_map = map.map();
  Voxel<float> occupancy(&occupancy_map, occupancy_map.layout().occupancyLayer(), key);
  Voxel<VoxelMean> mean(occupancy, occupancy_map.layout().meanLayer());
  Voxel<CovarianceVoxel> cov(occupancy, occupancy_map.layout().covarianceLayer());

  assert(occupancy.isValid());
  assert(mean.isValid());
  assert(cov.isValid());

  glm::dvec3 voxel_mean = positionUnsafe(mean);

  float *voxel_value = occupancy.dataPtr();
  float updated_value = *voxel_value;
  if (calculateHitWithCovariance(&cov.data(), &updated_value, sample, voxel_mean, mean.data().count,
                                 occupancy_map.hitValue(), unorbservedOccupancyValue(),
                                 float(occupancy_map.resolution()), map.reinitialiseCovarianceTheshold(),
                                 map.reinitialiseCovariancePointCount()))
  {
    // Covariance matrix has reset. Reset the point count to clear the mean value.
    mean.data().count = 0;
  }

  // Ensure we update the occupancy within the configured map limits.
  occupancyAdjustUp(
    voxel_value, *voxel_value, updated_value, unorbservedOccupancyValue(), occupancy_map.maxVoxelValue(),
    occupancy_map.saturateAtMinValue() ? occupancy_map.minVoxelValue() : std::numeric_limits<float>::lowest(),
    occupancy_map.saturateAtMaxValue() ? occupancy_map.maxVoxelValue() : std::numeric_limits<float>::max(), false);

  // Update the voxel mean.
  updatePositionUnsafe(mean, sample);
}


void ohm::integrateNdtMiss(NdtMap &map, const Key &key, const glm::dvec3 &sensor, const glm::dvec3 &sample)
{
  OccupancyMap &occupancy_map = map.map();
  Voxel<float> occupancy(&occupancy_map, occupancy_map.layout().occupancyLayer(), key);
  Voxel<const VoxelMean> mean(occupancy, occupancy_map.layout().meanLayer());
  Voxel<const CovarianceVoxel> cov(occupancy, occupancy_map.layout().covarianceLayer());

  assert(occupancy.isValid());
  assert(mean.isValid());
  assert(cov.isValid());

  float *voxel_value = occupancy.dataPtr();
  float updated_value = *voxel_value;
#ifdef TES_ENABLE
  const float initial_value = *voxel_value;
#endif  // TES_ENABLE
  const glm::dvec3 voxel_mean = positionUnsafe(mean);

#ifdef TES_ENABLE
  const glm::dvec3 voxel_maximum_likelihood =
#endif  // TES_ENABLE
    calculateMissNdt(cov.dataPtr(), &updated_value, sensor, sample, voxel_mean, mean.data().count,
                     unorbservedOccupancyValue(), occupancy_map.missValue(), map.adaptationRate(), map.sensorNoise(),
                     map.ndtSampleThreshold());
  occupancyAdjustDown(
    voxel_value, *voxel_value, updated_value, unorbservedOccupancyValue(), occupancy_map.minVoxelValue(),
    occupancy_map.saturateAtMinValue() ? occupancy_map.minVoxelValue() : std::numeric_limits<float>::lowest(),
    occupancy_map.saturateAtMaxValue() ? occupancy_map.maxVoxelValue() : std::numeric_limits<float>::max(), false);

#ifdef TES_ENABLE
  if (map.trace())
  {
    const glm::dvec3 voxel_centre = occupancy_map.voxelCentreGlobal(key);
    TES_BOX_W(g_3es, TES_COLOUR(OrangeRed), TES_PTR_ID(cov.dataPtr()), glm::value_ptr(voxel_centre),
              glm::value_ptr(glm::dvec3(occupancy_map.resolution())));
    TES_SERVER_UPDATE(g_3es, 0.0f);

    bool drew_surfel = false;
    glm::dquat rot;
    glm::dvec3 scale;
    if (covarianceUnitSphereTransformation(cov.dataPtr(), &rot, &scale))
    {
      TES_SPHERE(g_3es, TES_COLOUR(SeaGreen), TES_PTR_ID(cov.dataPtr()), glm::value_ptr(voxel_mean),
                 glm::value_ptr(scale), tes::Quaterniond(rot.x, rot.y, rot.z, rot.w));
      drew_surfel = true;
    }

    // Trace the voxel mean, maximum likelihood point and the ellipsoid.
    // Mean
    TES_SPHERE(g_3es, TES_COLOUR(OrangeRed), TES_PTR_ID(&voxel_mean), glm::value_ptr(voxel_mean), 0.05f);
    // Maximum likelihood
    TES_SPHERE_W(g_3es, TES_COLOUR(PowderBlue), TES_PTR_ID(&voxel_maximum_likelihood),
                 glm::value_ptr(voxel_maximum_likelihood), 0.1f);

    char text[64];
    sprintf(text, "P %.3f", ohm::valueToProbability(*voxel_value - initial_value));
    TES_TEXT2D_WORLD(g_3es, TES_COLOUR(White), text, glm::value_ptr(voxel_centre));

    TES_SERVER_UPDATE(g_3es, 0.0f);
    TES_BOX_END(g_3es, TES_PTR_ID(cov.dataPtr()));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_mean));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_maximum_likelihood));
    if (drew_surfel)
    {
      TES_SPHERE_END(g_3es, TES_PTR_ID(cov.dataPtr()));
    }
  }
#endif  // TES_ENABLE
}
