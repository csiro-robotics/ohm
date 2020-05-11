// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include "NdtMap.h"

#include "OccupancyMap.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "NdtVoxel.h"
#include "SubVoxel.h"
#include "Voxel.h"
#include "VoxelLayout.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/mat3x3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#include <3esservermacros.h>
#ifdef TES_ENABLE
#include <3esserver.h>
#include <shapes/3esshapes.h>
#endif  // TES_ENABLE

#include <cassert>

using namespace ohm;

namespace ohm
{
  struct NdtMapDetail
  {
    OccupancyMap *map = nullptr;
    double sensor_noise = 0.05;
    int covariance_layer_index = -1;
    unsigned sample_threshold = 10;
    bool borrowed_map = false;
    bool trace = false;
  };


  /// dot product of j-th and k-th columns of A
  /// A is (4,3), assumed to be packed as follows, where z is non-represented zero
  /// 0 1 3
  /// z 2 4
  /// z z 5
  /// 6 7 8
  inline double packedDot(const double A[9], const int j, const int k)
  {
    const static int col_first_el[] = { 0, 1, 3 };
    double d = A[6 + k] * A[6 + j];
    const int indj = col_first_el[j], indk = col_first_el[k], m = std::min(j, k);
    for (int i = 0; i <= m; ++i)
    {
      d += A[indj + i] * A[indk + i];
    }
    return d;
  }
}  // namespace ohm


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


void NdtMap::setSensorNoise(double noise_range)
{
  imp_->sensor_noise = noise_range;
}


double NdtMap::sensorNoise() const
{
  return imp_->sensor_noise;
}


void NdtMap::setNdtSampleThreshold(unsigned sample_count)
{
  imp_->sample_threshold = sample_count;
}


unsigned NdtMap::ndtSampleThreshold()
{
  return imp_->sample_threshold;
}


void NdtMap::setTrace(bool trace)
{
  imp_->trace = trace;
}


bool NdtMap::trace() const
{
  return imp_->trace;
}


int NdtMap::covarianceLayerIndex() const
{
  return imp_->covariance_layer_index;
}


void NdtMap::integrateHit(Voxel &voxel, const glm::dvec3 & /*sensor*/, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;
  const bool was_uncertain = voxel.isUncertain();
  const float initial_value = voxel.value();
  // NDT probably value update is the same as for the basic occupancy map.
  voxel.setValue(!was_uncertain ? initial_value + map.hitValue() : map.hitValue());

  assert(map.layout().hasSubVoxelPattern());
  OccupancyVoxel *voxel_occupancy = voxel.layerContent<OccupancyVoxel *>(map.layout().occupancyLayer());
  NdtVoxel *ndt_voxel = voxel.layerContent<NdtVoxel *>(imp_->covariance_layer_index);

  // Initialise the ndt_voxel data if this transitions the voxel to an occupied state.
  if (was_uncertain || initial_value <= map.occupancyThresholdValue())
  {
    // Transitioned to occupied. Initialise.
    initialise(ndt_voxel, imp_->sensor_noise);
  }

  const glm::dvec3 voxel_centre = voxel.centreGlobal();
  glm::dvec3 voxel_mean;
#if OHM_NDT_UNPACKED_MEAN
  voxel_mean = glm::dvec3(ndt_voxel->mean[0], ndt_voxel->mean[1], ndt_voxel->mean[2]);
#else   // OHM_NDT_UNPACKED_MEAN
  voxel_mean = subVoxelToLocalCoord<glm::dvec3>(voxel_occupancy->sub_voxel, map.resolution()) + voxel_centre;
#endif  // OHM_NDT_UNPACKED_MEAN

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

  const glm::dvec3 sample_to_mean = sample - voxel_mean;
  double A[9];
  unpackedA(*ndt_voxel, A, sample_to_mean);

  // Update covariance.
  for (size_t k = 0; k < 3; ++k)
  {
    const size_t ind1 = (k * (k + 3)) >> 1;  // packed index of (k,k) term
    const size_t indk = ind1 - k;            // packed index of (1,k)
    const double ak = std::sqrt(packedDot(&A[0], k, k));
    ndt_voxel->cov_sqrt_diag[ind1] = float(ak);
    if (ak > 0.0)
    {
      const double aki = 1.0 / ak;
      for (size_t j = k + 1; j < 3; ++j)
      {
        const size_t indj = (j * (j + 1)) >> 1, indkj = indj + k;
        double c = packedDot(&A[0], j, k) * aki;
        ndt_voxel->cov_sqrt_diag[indkj] = float(c);
        c *= aki;
        A[j + 6] -= c * A[k + 6];
        for (size_t l = 0; l <= k; ++l)
        {
          A[indj + l] -= c * A[indk + l];
        }
      }
    }
  }

  // Update the mean sub-voxel pattern
  const double point_count = double(ndt_voxel->point_count);
  const double one_on_pt_count_plus_one = 1.0 / (point_count + 1.0);
  voxel_mean = (point_count * voxel_mean + sample) * one_on_pt_count_plus_one;
#if OHM_NDT_UNPACKED_MEAN
  ndt_voxel->mean[0] = voxel_mean[0];
  ndt_voxel->mean[1] = voxel_mean[1];
  ndt_voxel->mean[2] = voxel_mean[2];
#endif  //
  voxel_occupancy->sub_voxel = subVoxelCoord(voxel_mean - voxel_centre, map.resolution());

  if (ndt_voxel->point_count < std::numeric_limits<decltype(ndt_voxel->point_count)>::max())
  {
    ++ndt_voxel->point_count;
  }
}


void NdtMap::integrateMiss(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;
  float update_value = map.missValue();

  TES_IF(imp_->trace)
  {
    TES_BOX_W(g_3es, TES_COLOUR(OrangeRed),
              TES_PTR_ID(voxel.layerContent<const NdtVoxel *>(imp_->covariance_layer_index)),
              glm::value_ptr(voxel.centreGlobal()), glm::value_ptr(glm::dvec3(imp_->map->resolution())));
    TES_SERVER_UPDATE(g_3es, 0.0f);
  }

  if (voxel.isUncertain())
  {
    // First touch of the voxel. Apply the miss value as is.
    // Same behaviour as OccupancyMap.
    voxel.setValue(update_value);
    TES_IF(imp_->trace)
    {
      TES_BOX_END(g_3es, TES_PTR_ID(voxel.layerContent<const NdtVoxel *>(imp_->covariance_layer_index)));
    }
    return;
  }

  if (voxel.isFree())
  {
    // Re-enforcement of free voxel. Add miss value.
    // Same behaviour as OccupancyMap.
    voxel.setValue(update_value + voxel.value());
    TES_IF(imp_->trace)
    {
      TES_BOX_END(g_3es, TES_PTR_ID(voxel.layerContent<const NdtVoxel *>(imp_->covariance_layer_index)));
    }
    return;
  }

  // Update of an occupied voxel. We have to unpack the covariance and apply NDT logic.
  NdtVoxel *ndt_voxel = voxel.layerContent<NdtVoxel *>(imp_->covariance_layer_index);
  if (ndt_voxel->point_count < imp_->sample_threshold)
  {
    // Too few points to resolve a guassing. Use standard value update.
    voxel.setValue(update_value + voxel.value());
    if (imp_->trace)
    {
      TES_BOX_END(g_3es, TES_PTR_ID(ndt_voxel));
    }
    return;
  }

#if OHM_NDT_UNPACKED_MEAN
  const glm::dvec3 voxel_mean(ndt_voxel->mean[0], ndt_voxel->mean[1], ndt_voxel->mean[2]);
#else   // OHM_NDT_UNPACKED_MEAN
  assert(map.layout().hasSubVoxelPattern());
  const OccupancyVoxel *voxel_occupancy = voxel.layerContent<const OccupancyVoxel *>(map.layout().occupancyLayer());
  const glm::dvec3 voxel_mean(subVoxelToLocalCoord<glm::dvec3>(voxel_occupancy->sub_voxel, map.resolution()) +
                              voxel.centreGlobal());
#endif  // OHM_NDT_UNPACKED_MEAN

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

  const glm::dvec3 sensor_to_sample = sample - sensor;
  const glm::dvec3 sensor_ray = glm::normalize(sensor_to_sample);  // Verified
  const glm::dvec3 sensor_to_mean = sensor - voxel_mean;

  // Packed data solutions:
  const glm::dvec3 a = solveTriangular(*ndt_voxel, sensor_ray);
  const glm::dvec3 b_norm = solveTriangular(*ndt_voxel, sensor_to_mean);

  // const glm::dvec3 a = covariance_inv * sensor_ray;  // Verified (unpacked version)
  // (28)
  // const double t = glm::dot(a, sensor_to_mean) / glm::dot(a, sensor_ray); // Verified (unpacked version)
  const double t = -glm::dot(a, b_norm) / glm::dot(a, a);  // Verified

  // (25)
  // Note: maximum_likelyhood is abbreviated to ml in assoicated variable names.
  const glm::dvec3 voxel_maximum_likelyhood = sensor_ray * t + sensor;  // Verified

  // (22)
  // Unverified: json line 264
  // const double p_x_ml_given_voxel = std::exp(
  //   -0.5 * glm::dot(voxel_maximum_likelyhood - voxel_mean, covariance_inv * (voxel_maximum_likelyhood -
  //   voxel_mean)));
  // Corrected:
  const double p_x_ml_given_voxel =
    exp(-0.5 * glm::length2(solveTriangular(*ndt_voxel, voxel_maximum_likelyhood - voxel_mean)));

  // (23)
  // Verified: json: line 263
  const double sensor_noise_variance = imp_->sensor_noise * imp_->sensor_noise;
  const double p_x_ml_given_sample = exp(-0.5 * glm::length2(voxel_maximum_likelyhood - sample) / sensor_noise_variance);

  const double scaling_factor = map.missProbability();
  // Verified: json line 267
  const double probability_update = 0.5 - scaling_factor * p_x_ml_given_voxel * (1.0 - p_x_ml_given_sample);

#ifdef TES_ENABLE
  TES_IF(imp_->trace)
  {
    bool drew_surfel = false;
    glm::dvec3 evals;
    glm::dmat3 evecs;
    if (eigenDecomposition(*ndt_voxel, &evals, &evecs))
    {
      glm::dquat rot(evecs);
      rot = glm::normalize(rot);
      TES_SPHERE(g_3es, TES_COLOUR(SeaGreen), TES_PTR_ID(ndt_voxel), glm::value_ptr(voxel_mean),
                 glm::value_ptr(2.0 * evals), tes::Quaterniond(rot.x, rot.y, rot.z, rot.w));
      drew_surfel = true;
    }

    // Trace the voxel mean, maximum likelyhood point and the ellipsoid.
    // Mean
    TES_SPHERE(g_3es, TES_COLOUR(OrangeRed), TES_PTR_ID(&voxel_mean), glm::value_ptr(voxel_mean), 0.05f);
    // Maximum likelyhood
    TES_SPHERE_W(g_3es, TES_COLOUR(PowderBlue), TES_PTR_ID(&voxel_maximum_likelyhood),
                 glm::value_ptr(voxel_maximum_likelyhood), 0.1f);

    glm::dvec3 pos = voxel.centreGlobal();
    char text[64];
    const float z_step = float(map.resolution() * 0.05);
    pos.z += z_step;
    sprintf(text, "Px_u %.3f", p_x_ml_given_voxel);
    TES_TEXT2D_WORLD(g_3es, TES_COLOUR(White), text, glm::value_ptr(pos));
    pos.z -= z_step;
    sprintf(text, "Px_z %.3f", p_x_ml_given_sample);
    TES_TEXT2D_WORLD(g_3es, TES_COLOUR(White), text, glm::value_ptr(pos));
    pos.z -= z_step;
    sprintf(text, "P %.3f", probability_update);
    TES_TEXT2D_WORLD(g_3es, TES_COLOUR(White), text, glm::value_ptr(pos));

    TES_SERVER_UPDATE(g_3es, 0.0f);
    TES_BOX_END(g_3es, TES_PTR_ID(ndt_voxel));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_mean));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_maximum_likelyhood));
    TES_IF(drew_surfel) { TES_SPHERE_END(g_3es, TES_PTR_ID(ndt_voxel)); }
  }
#endif // TES_ENABLE

  if (std::isnan(probability_update))
  {
    // This should no longer be occurring.
    return;
  }

  update_value = ohm::probabilityToValue(probability_update);
  assert(update_value <= 0);

  if (update_value == 0)
  {
    // No change.
    return;
  }

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
#if OHM_NDT_UNPACKED_MEAN
  voxel.addMember("mean_x", DataType::kFloat, 0);
  voxel.addMember("mean_y", DataType::kFloat, 0);
  voxel.addMember("mean_z", DataType::kFloat, 0);
#endif  // OHM_NDT_UNPACKED_MEAN
  voxel.addMember("sample_count", DataType::kUInt32, 0);

  map->updateLayout(layout);
  return layer.layerIndex();
}


void NdtMap::debugDraw()
{
#ifdef TES_ENABLE
  if (!g_3es || g_3es->connectionCount() == 0)
  {
    return;
  }

  std::vector<tes::Sphere> ellipsoids;
  std::vector<tes::Shape *> shape_ptrs;

  const auto send = [&ellipsoids, &shape_ptrs]()  //
  {
    if (!ellipsoids.empty())
    {
      shape_ptrs.clear();
      for (size_t i = 0; i < ellipsoids.size(); ++i)
      {
        shape_ptrs.emplace_back(&ellipsoids[i]);
      }

      g_3es->create(tes::MultiShape(shape_ptrs.data(), shape_ptrs.size()));
      ellipsoids.clear();
    }
  };

  uint32_t next_id = static_cast<uint32_t>((size_t)this);
  const tes::Colour c = tes::Colour::Colours[tes::Colour::SeaGreen];
  for (auto iter = imp_->map->begin(); iter != imp_->map->end(); ++iter)
  {
    const auto voxel = *iter;
    if (voxel.isOccupied())
    {
      const NdtVoxel &ndt_voxel = *voxel.layerContent<const NdtVoxel *>(imp_->covariance_layer_index);
      glm::dvec3 evals;
      glm::dmat3 evecs;
      if (!eigenDecomposition(ndt_voxel, &evals, &evecs))
      {
        continue;
      }

#if OHM_NDT_UNPACKED_MEAN
      const glm::dvec3 voxel_mean(ndt_voxel.mean[0], ndt_voxel.mean[1], ndt_voxel.mean[2]);
#else  // OHM_NDT_UNPACKED_MEAN
      const glm::dvec3 voxel_mean = voxel.position();
#endif // OHM_NDT_UNPACKED_MEAN
      tes::Sphere ellipsoid(next_id, glm::value_ptr(voxel_mean));
      glm::dquat q(evecs);
      ellipsoid.setRotation(tes::Quaterniond(q.x, q.y, q.z, q.w));
      ellipsoid.setScale(2.0 * tes::Vector3d(evals[0], evals[1], evals[2]));
      ellipsoid.setColour(c);
      ellipsoids.emplace_back(ellipsoid);

      if (ellipsoids.size() >= tes::MultiShape::ShapeCountLimit)
      {
        send();
        ++next_id;
      }
    }
  }

  send();

  TES_SERVER_UPDATE(ohm::g_3es, 0.0f);
#endif  // TES_ENABLE
}
