// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include "NdtMap.h"

#include "private/NdtMapDetail.h"

#include "DefaultLayer.h"
#include "OccupancyMap.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "MapProbability.h"
#include "VoxelData.h"

#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>

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
  // Much lower hit prob for NDT is better.
  map->setHitProbability(0.55f);
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


void NdtMap::setAdaptationRate(float rate)
{
  imp_->adaptation_rate = rate;
  updateMapInfo();
}


float NdtMap::adaptationRate() const
{
  return imp_->adaptation_rate;
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
  Voxel<const float> occupancy(imp_->map, imp_->map->layout().occupancyLayer());
  Voxel<const CovarianceVoxel> cov(imp_->map, imp_->map->layout().covarianceLayer());
  Voxel<const VoxelMean> mean(imp_->map, imp_->map->layout().meanLayer());
  for (auto iter = imp_->map->begin(); iter != imp_->map->end(); ++iter)
  {
    occupancy.setKey(*iter);
    const auto voxel = *iter;
    if (isOccupied(occupancy.data()))
    {
      cov.setKey(occupancy);

      glm::dquat rot;
      glm::dvec3 scale;
      if (!covarianceUnitSphereTransformation(cov.dataPtr(), &rot, &scale))
      {
        continue;
      }

      mean.setKey(occupancy);

      const glm::dvec3 voxel_mean = position(mean);

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
  info.set(MapValue("Ndt adaptation rate", imp_->adaptation_rate));
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
