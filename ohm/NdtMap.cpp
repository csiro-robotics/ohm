//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#include "NdtMap.h"

#include "OccupancyMap.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "SubVoxel.h"
#include "Voxel.h"
#include "VoxelLayout.h"

#include <glm/mat3x3.hpp>
#include <glm/vec4.hpp>

#include <glm/gtc/matrix_access.hpp>

#include <cassert>

using namespace ohm;

namespace ohm
{
  struct NdtMapDetail
  {
    OccupancyMap *map = nullptr;
    int covariance_layer_index = -1;
    bool borrowed_map = false;
  };

  struct NdtVoxelDetail
  {
    float cov_sqrt_diag[6];
    // TODO: check packing. Should be tightly packed.
    uint32_t point_count;
    // Padding: voxel alignment is 8 bytes
    uint32_t padding;

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
    double *unpackedA(double A[9], const glm::dvec3 sample_to_mean) const
    {
      if (point_count == 0)
      {
        for (int i = 0; i < 9; ++i)
        {
          A[i] = 0;
        }
        return A;
      }

      const double one_on_num_pt_plus_one = 1.0 / (double(point_count) + 1);
      const double sc_1 = std::sqrt(double(point_count) * one_on_num_pt_plus_one);
      const double sc_2 = one_on_num_pt_plus_one * std::sqrt(double(point_count));

      for (int i = 0; i < 6; ++i)
      {
        A[i] = sc_1 * double(cov_sqrt_diag[i]);
      }

      for (int i = 0; i < 3; ++i)
      {
        A[i + 6] = sc_2 * sample_to_mean[i];
      }

      return A;
    }
  };


  /// dot product of j-th and k-th columns of A
  /// A is (4,3), assumed to be packed as follows, where z is non-represented zero
  /// 0 1 3
  /// z 2 4
  /// z z 5
  /// 6 7 8
  double packed_dot(const double A[9], const int j, const int k)
  {
    const static int col_first_el[] = {0, 1, 3};
    double d = A[6 + k] * A[6 + j];
    const int indj = col_first_el[j], indk = col_first_el[k], m = std::min(j, k);
    for (int i = 0; i <= m; ++i)
    {
      d += A[indj + i] * A[indk + i];
    }
    return d;
  }

  glm::dmat3 unpackCovariance(const double A[9])
  {
    glm::dmat3 cov;
    glm::dvec3 *col = &cov[0];
    (*col)[0] = A[0];
    (*col)[1] = (*col)[2] = 0;

    col = &cov[1];
    (*col)[0] = A[1];
    (*col)[1] = A[2];
    (*col)[2] = 0;

    col = &cov[2];
    (*col)[0] = A[3];
    (*col)[1] = A[4];
    (*col)[2] = A[5];

    return cov;
  }
}


NdtMap::NdtMap(OccupancyMap *map, bool borrowed_map)
  : imp_(new NdtMapDetail)
{
  imp_->map = map;
  imp_->borrowed_map = borrowed_map;
  imp_->covariance_layer_index = enableNdt(map);
}


NdtMap::~NdtMap()
{
  if (imp_)
  {
    if (!imp_->borrowed_map)
    {
      delete imp_->map;
    }
    delete imp_;
  }
}


OccupancyMap &NdtMap::map()
{
  return *imp_->map;
}


const OccupancyMap &NdtMap::map() const
{
  return *imp_->map;
}


bool NdtMap::borrowedMap() const
{
  return imp_->borrowed_map;
}


void NdtMap::integrateHit(Voxel &voxel, const glm::dvec3 &/*sensor*/, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;
  // NDT probably value update is the same as for the basic occupancy map.
  voxel.setValue(!voxel.isUncertain() ? voxel.value() + map.hitValue() : map.hitValue());

  assert(map.layout().hasSubVoxelPattern());
  OccupancyVoxel *voxel_occupancy = voxel.layerContent<OccupancyVoxel *>(map.layout().occupancyLayer());
  const glm::dvec3 voxel_mean(subVoxelToLocalCoord<glm::dvec3>(voxel_occupancy->sub_voxel, map.resolution()));
  voxel_occupancy->sub_voxel = subVoxelUpdate(voxel_occupancy->sub_voxel, sample - voxel.centreGlobal(),
                                              map.resolution(), map.subVoxelWeighting());

  // This has been taken from example code provided by Jason Williams as a sample on storing and using covarance data
  // using a packed, diagonal.
  // FIXME: document the reasoning and maths behind this cose.

  // Resolve the covariance matrix for the voxel.
  NdtVoxelDetail *ndt_voxel = voxel.layerContent<NdtVoxelDetail *>(imp_->covariance_layer_index);
  double cov_sqrt[6];
  const glm::dvec3 sample_to_mean = sample - voxel_mean;
  double A[9];
  ndt_voxel->unpackedA(A, sample_to_mean);

  // Update covariance.
  for (int k = 0; k < 3; ++k)
  {
    const int ind1 = (k * (k + 3)) >> 1;  // Packed index of (k,k) term
    const int indk = ind1 - k;            // Packed index of (1,k)

    const double ak = sqrt(packed_dot(A, k, k));
    cov_sqrt[ind1] = ak;

    if (ak > 0.0)
    {
      const double aki = 1.0 / ak;
      for (int j = k + 1; j < 3; ++j)
      {
        const int indj = (j * (j + 1)) >> 1;
        const int indkj = indj + k;
        double c = packed_dot(A, j, k) * aki;
        cov_sqrt[indkj] = c;
        c *= aki;
        A[j + 6] -= c * A[k + 6];
        for (int l = 0; l < k; ++l)
        {
          A[indj + l] -= c * A[indk + l];
        }
      }
    }
  }

  // Update the packed covariance storage.
  for (int i = 0; i < 6; ++i)
  {
    ndt_voxel->cov_sqrt_diag[i] = float(cov_sqrt[i]);
  }

  if (ndt_voxel->point_count < std::numeric_limits<decltype(ndt_voxel->point_count)>::max())
  {
    ++ndt_voxel->point_count;
  }
}


void NdtMap::integrateMiss(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;
  float update_value = map.missValue();

  if (voxel.isUncertain())
  {
    // First touch of the voxel. Apply the miss value as is.
    // Same behaviour as OccupancyMap.
    voxel.setValue(update_value);
    return;
  }

  if (voxel.isFree())
  {
    // Re-enforcement of free voxel. Add miss value.
    // Same behaviour as OccupancyMap.
    voxel.setValue(update_value + voxel.value());
    return;
  }

  // Update of an occupied voxel. We have to unpack the covariance and apply NDT logic.
  NdtVoxelDetail *ndt_voxel = voxel.layerContent<NdtVoxelDetail *>(imp_->covariance_layer_index);
  if (ndt_voxel->point_count < 4)
  {
    // Too few points to resolve a guassing. Use standard value update.
    voxel.setValue(update_value + voxel.value());
    return;
  }

  assert(map.layout().hasSubVoxelPattern());
  const OccupancyVoxel *voxel_occupancy = voxel.layerContent<const OccupancyVoxel *>(map.layout().occupancyLayer());
  const glm::dvec3 voxel_mean(subVoxelToLocalCoord<glm::dvec3>(voxel_occupancy->sub_voxel, map.resolution()));

  // Unapack the covariance data into a matrix.
  // FIXME: Validate that this is correctly extracting the covariance matrix.
  const glm::dvec3 sample_to_mean = sample - voxel_mean;
  double A[9];
  ndt_voxel->unpackedA(A, sample_to_mean);
  glm::dmat3 covariance = unpackCovariance(A);

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

  const glm::dmat3 covariance_inv = glm::inverse(covariance);
  const glm::dvec3 sample_ray = glm::normalize(sample_to_mean);
  const glm::dvec3 sensor_to_mean = sensor - voxel_mean;

  const glm::dvec3 a = covariance_inv * sample_ray;
  // (28)
  const double t = glm::dot(a, sensor_to_mean) / glm::dot(a, sample_ray);

  // (25)
  const glm::dvec3 voxel_maximum_likelyhood = sample_ray * t + sample;
  // TODO: should probably validate this falls within the voxel.

  // (22)
  const double prob_hit_given_cov = std::exp(-0.5 * glm::dot(voxel_maximum_likelyhood - voxel_mean, covariance_inv * (voxel_maximum_likelyhood - voxel_mean)));

  // (23)
  const glm::dvec3 sample_to_voxel_ml = sample - voxel_maximum_likelyhood;
  const double sensor_noise = 0.05;
  const double prob_voxel_given_sample = std::exp(-0.5 * glm::dot(sample_to_voxel_ml, sample_to_voxel_ml) / (sensor_noise * sensor_noise));

  const double scaling_factor = 0.4;
  const double probability_update = 0.5 - scaling_factor * prob_hit_given_cov * prob_voxel_given_sample;

  if (std::isnan(probability_update))
  {
    // TODO: (KS) work out how we get a nan value here. It should not happen.
    return;
  }

  update_value = ohm::probabilityToValue(probability_update);
  assert(update_value <= 0);
  voxel.setValue(update_value + voxel.value());
}


int NdtMap::enableNdt(OccupancyMap *map)
{
  map->setSubVoxelsEnabled(true);

  // We need additional layers in the map to store information for the NDT update.
  MapLayout layout = map->layout();
  MapLayer &layer = *layout.addLayer("covariance");
  VoxelLayout voxel = layer.voxelLayout();
  // Add members to represent a diagonal of the covariance matrix. This is an approximation of the full matrix
  // but it greatly reduces the per voxel memory usage.
  voxel.addMember("P00", DataType::kFloat, 0);
  voxel.addMember("P11", DataType::kFloat, 0);
  voxel.addMember("P22", DataType::kFloat, 0);
  voxel.addMember("P33", DataType::kFloat, 0);
  voxel.addMember("P44", DataType::kFloat, 0);
  voxel.addMember("P55", DataType::kFloat, 0);
  voxel.addMember("sample_count", DataType::kUInt32, 0);

  map->updateLayout(layout);
  return layer.layerIndex();
}
