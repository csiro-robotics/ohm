// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include "NdtMap.h"

#include "private/NdtMapDetail.h"

#include "OccupancyMap.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "MapProbability.h"
#include "Voxel.h"
#include "VoxelLayout.h"
#include "VoxelMean.h"

#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#include "CovarianceVoxel.h"

#include <3esservermacros.h>
#ifdef TES_ENABLE
#include <3esserver.h>
#include <shapes/3esshapes.h>
#endif  // TES_ENABLE

#include <cassert>

using namespace ohm;

NdtMap::NdtMap(OccupancyMap *map, bool borrowed_map)
  : imp_(new NdtMapDetail)
{
  imp_->map = map;
  imp_->borrowed_map = borrowed_map;
  enableNdt(map);
  updateMapInfo();
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


void NdtMap::setSensorNoise(float noise_range)
{
  imp_->sensor_noise = noise_range;
  updateMapInfo();
}


float NdtMap::sensorNoise() const
{
  return imp_->sensor_noise;
}


void NdtMap::setNdtSampleThreshold(unsigned sample_count)
{
  imp_->sample_threshold = sample_count;
  updateMapInfo();
}


unsigned NdtMap::ndtSampleThreshold()
{
  return imp_->sample_threshold;
}


void NdtMap::setReinitialiseCovarianceTheshold(float threshold)
{
  imp_->reinitialise_covariance_theshold = threshold;
  updateMapInfo();
}


float NdtMap::reinitialiseCovarianceTheshold() const
{
  return imp_->reinitialise_covariance_theshold;
}


void NdtMap::setReinitialiseCovariancePointCount(unsigned count)
{
  imp_->reinitialise_covariance_point_count = count;
  updateMapInfo();
}


unsigned NdtMap::reinitialiseCovariancePointCount() const
{
  return imp_->reinitialise_covariance_point_count;
}


void NdtMap::setTrace(bool trace)
{
  imp_->trace = trace;
}


bool NdtMap::trace() const
{
  return imp_->trace;
}


void NdtMap::integrateHit(Voxel &voxel, const glm::dvec3 & /*sensor*/, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;

  assert(map.layout().meanLayer() >= 0);
  assert(map.layout().covarianceLayer() >= 0);
  const int covariance_layer_index = map.layout().covarianceLayer();
  CovarianceVoxel *cov_voxel = voxel.layerContent<CovarianceVoxel *>(covariance_layer_index);
  VoxelMean *voxel_mean_info = voxel.layerContent<VoxelMean *>(map.layout().meanLayer());

  const glm::dvec3 voxel_centre = voxel.centreGlobal();
  glm::dvec3 voxel_mean;
  voxel_mean = subVoxelToLocalCoord<glm::dvec3>(voxel_mean_info->coord, map.resolution()) + voxel_centre;

  float voxel_value = voxel.value();
  if (calculateHitWithCovariance(cov_voxel, &voxel_value, sample, voxel_mean, voxel_mean_info->count, map.hitValue(),
                                 voxel::invalidMarkerValue(), float(map.resolution()),
                                 imp_->reinitialise_covariance_theshold, imp_->reinitialise_covariance_point_count))
  {
    // Covariance matrix has reset. Reset the point count to clear the mean value.
    voxel_mean_info->count = 0;
  }

  // NDT probably value update is the same as for the basic occupancy map.
  voxel.setValue(voxel_value);

  // Update the voxel mean.
  voxel_mean_info->coord =
    subVoxelUpdate(voxel_mean_info->coord, voxel_mean_info->count, sample - voxel_centre, map.resolution());
  ++voxel_mean_info->count;
}


void NdtMap::integrateMiss(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;

  const float initial_value = voxel.value();
  float voxel_value = initial_value;
  const int covariance_layer_index = map.layout().covarianceLayer();
  CovarianceVoxel *cov_voxel = voxel.layerContent<CovarianceVoxel *>(covariance_layer_index);

  assert(map.layout().meanLayer() >= 0);
  VoxelMean *voxel_mean_info = voxel.layerContent<VoxelMean *>(map.layout().meanLayer());
  const glm::dvec3 voxel_mean(subVoxelToLocalCoord<glm::dvec3>(voxel_mean_info->coord, map.resolution()) +
                              voxel.centreGlobal());

#ifdef TES_ENABLE
  const glm::dvec3 voxel_maximum_likelihood =
#endif  // TES_ENABLE
    calculateMissNdt(cov_voxel, &voxel_value, sensor, sample, voxel_mean, voxel_mean_info->count,
                     voxel::invalidMarkerValue(), map.missValue(), imp_->sensor_noise, imp_->sample_threshold);
  voxel.setValue(voxel_value);

  TES_IF(imp_->trace)
  {
    TES_BOX_W(g_3es, TES_COLOUR(OrangeRed),
              TES_PTR_ID(voxel.layerContent<const CovarianceVoxel *>(covariance_layer_index)),
              glm::value_ptr(voxel.centreGlobal()), glm::value_ptr(glm::dvec3(imp_->map->resolution())));
    TES_SERVER_UPDATE(g_3es, 0.0f);
  }

#ifdef TES_ENABLE
  TES_IF(imp_->trace)
  {
    bool drew_surfel = false;
    glm::dquat rot;
    glm::dvec3 scale;
    if (covarianceUnitSphereTransformation(cov_voxel, &rot, &scale))
    {
      TES_SPHERE(g_3es, TES_COLOUR(SeaGreen), TES_PTR_ID(cov_voxel), glm::value_ptr(voxel_mean), glm::value_ptr(scale),
                 tes::Quaterniond(rot.x, rot.y, rot.z, rot.w));
      drew_surfel = true;
    }

    // Trace the voxel mean, maximum likelihood point and the ellipsoid.
    // Mean
    TES_SPHERE(g_3es, TES_COLOUR(OrangeRed), TES_PTR_ID(&voxel_mean), glm::value_ptr(voxel_mean), 0.05f);
    // Maximum likelihood
    TES_SPHERE_W(g_3es, TES_COLOUR(PowderBlue), TES_PTR_ID(&voxel_maximum_likelihood),
                 glm::value_ptr(voxel_maximum_likelihood), 0.1f);

    glm::dvec3 pos = voxel.centreGlobal();
    char text[64];
    sprintf(text, "P %.3f", ohm::valueToProbability(voxel_value - initial_value));
    TES_TEXT2D_WORLD(g_3es, TES_COLOUR(White), text, glm::value_ptr(pos));

    TES_SERVER_UPDATE(g_3es, 0.0f);
    TES_BOX_END(g_3es, TES_PTR_ID(cov_voxel));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_mean));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_maximum_likelihood));
    if (drew_surfel)
    {
      TES_SPHERE_END(g_3es, TES_PTR_ID(cov_voxel));
    }
  }
#endif  // TES_ENABLE
}


void NdtMap::debugDraw() const
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
  const int covariance_layer_index = imp_->map->layout().covarianceLayer();
  for (auto iter = imp_->map->begin(); iter != imp_->map->end(); ++iter)
  {
    const auto voxel = *iter;
    if (voxel.isOccupied())
    {
      const CovarianceVoxel &cov_voxel = *voxel.layerContent<const CovarianceVoxel *>(covariance_layer_index);

      glm::dquat rot;
      glm::dvec3 scale;
      if (!covarianceUnitSphereTransformation(&cov_voxel, &rot, &scale))
      {
        continue;
      }

      const glm::dvec3 voxel_mean = voxel.position();

      tes::Sphere ellipsoid(next_id, glm::value_ptr(voxel_mean));
      ellipsoid.setRotation(tes::Quaterniond(rot.x, rot.y, rot.z, rot.w));
      ellipsoid.setScale(2.0 * tes::Vector3d(scale.x, scale.y, scale.z));
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


void NdtMap::updateMapInfo()
{
  if (!imp_->map)
  {
    return;
  }
  MapInfo &info = imp_->map->mapInfo();
  info.set(MapValue("Ndt sensor noise", imp_->sensor_noise));
  info.set(MapValue("Ndt sample threshold", imp_->sample_threshold));
  info.set(MapValue("Ndt reinitialisation threshold", imp_->reinitialise_covariance_theshold));
  info.set(MapValue("Ndt reinitialisation threshold (probability)",
                    valueToProbability(imp_->reinitialise_covariance_theshold)));
  info.set(MapValue("Ndt reinitialisation point count", imp_->reinitialise_covariance_point_count));
}


int NdtMap::enableNdt(OccupancyMap *map)
{
  // Prepare layout for update.
  MapLayout new_layout = map->layout();

  addVoxelMean(new_layout);
  // Cache the layer index.
  int layer_index = addCovariance(new_layout)->layerIndex();

  // Update the map.
  map->updateLayout(new_layout);

  return layer_index;
}
