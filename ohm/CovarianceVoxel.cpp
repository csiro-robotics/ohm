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

#include <glm/gtx/matrix_factorisation.hpp>

using namespace ohm;

void ohm::covarianceEigenDecomposition(const CovarianceVoxel *cov, glm::dmat3 *eigenvectors, glm::dvec3 *eigenvalues)
{
  const glm::dmat3 cov_mat = covarianceMatrix(cov);
  const glm::dmat3 mat_p = cov_mat * glm::transpose(cov_mat);

  glm::dmat3 eigenvalues_matrix;
  glm::qr_decompose(mat_p, *eigenvectors, eigenvalues_matrix);
  (*eigenvalues)[0] = eigenvalues_matrix[0][0];
  (*eigenvalues)[1] = eigenvalues_matrix[1][1];
  (*eigenvalues)[2] = eigenvalues_matrix[2][2];
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
    (*scale)[i] = (eigenvalues[i] > 1e-9) ? 2.0 * std::sqrt(eigenvalues[i]) : eigenvalues[i];
  }

  return true;
}
