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

#include <glm/gtc/matrix_access.hpp>

#define WITH_EIGEN 1

using namespace ohm;

#if WITH_EIGEN
// FIXME: add cmake option.
#include <Eigen/Dense>

bool ohm::eigenDecomposition(const CovarianceVoxel *cov, glm::dvec3 *eigenvalues, glm::dmat3 *eigenvectors)
{
  const glm::dmat3 cov_mat = covarianceMatrix(cov);
  Eigen::Matrix3d cov_eigen;

  cov_eigen << glm::row(cov_mat, 0)[0], glm::row(cov_mat, 0)[1], glm::row(cov_mat, 0)[2],  //
    glm::row(cov_mat, 1)[0], glm::row(cov_mat, 1)[1], glm::row(cov_mat, 1)[2],           //
    glm::row(cov_mat, 2)[0], glm::row(cov_mat, 2)[1], glm::row(cov_mat, 2)[2];

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

  return true;
}

#else  // WITH_EIGEN

bool ohm::eigenDecomposition(const CovarianceVoxel &, glm::dvec3 *, glm::mat3 *)
{
  return false;
}

#endif  // WITH_EIGEN
