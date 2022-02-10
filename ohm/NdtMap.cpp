// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include "NdtMap.h"

#include "private/NdtMapDetail.h"

#include "DefaultLayer.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "MapProbability.h"
#include "OccupancyMap.h"
#include "VoxelData.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/vec4.hpp>

#include <iostream>

#include <3esservermacros.h>
#ifdef TES_ENABLE
#include <3esserver.h>
#include <shapes/3esshapes.h>
#endif  // TES_ENABLE

#include <cassert>

namespace ohm
{
NdtMap::NdtMap(OccupancyMap *map, bool borrowed_map, NdtMode mode)
  : imp_(new NdtMapDetail)
{
  imp_->map = map;
  imp_->borrowed_map = borrowed_map;
  imp_->mode = mode;
  if (imp_->adaptation_rate <= 0)
  {
    imp_->adaptation_rate = ndtAdaptationRateFromMissProbability(map->missProbability());
  }
  enableNdt(map, mode);
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


NdtMode NdtMap::mode() const
{
  return imp_->mode;
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


void NdtMap::setReinitialiseCovarianceThreshold(float threshold)
{
  imp_->reinitialise_covariance_threshold = threshold;
  updateMapInfo();
}


float NdtMap::reinitialiseCovarianceThreshold() const
{
  return imp_->reinitialise_covariance_threshold;
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


void NdtMap::setInitialIntensityCovariance(float initial_intensity_covariance)
{
  imp_->initial_intensity_covariance = initial_intensity_covariance;
  updateMapInfo();
}


float NdtMap::initialIntensityCovariance() const
{
  return imp_->initial_intensity_covariance;
}


void NdtMap::setTrace(bool trace)
{
  imp_->trace = trace;
}


bool NdtMap::trace() const
{
  return imp_->trace;
}


void NdtMap::updateMapInfo()
{
  if (!imp_->map)
  {
    return;
  }
  MapInfo &info = imp_->map->mapInfo();
  info.set(MapValue("Ndt mode", int(imp_->mode)));
  info.set(MapValue("Ndt mode name", ndtModeToString(imp_->mode).c_str()));
  info.set(MapValue("Ndt adaptation rate", imp_->adaptation_rate));
  info.set(MapValue("Ndt sensor noise", imp_->sensor_noise));
  info.set(MapValue("Ndt sample threshold", imp_->sample_threshold));
  info.set(MapValue("Ndt reinitialisation threshold", imp_->reinitialise_covariance_threshold));
  info.set(MapValue("Ndt reinitialisation threshold (probability)",
                    valueToProbability(imp_->reinitialise_covariance_threshold)));
  info.set(MapValue("Ndt reinitialisation point count", imp_->reinitialise_covariance_point_count));
}


int NdtMap::enableNdt(OccupancyMap *map, NdtMode ndt_mode)
{
  // Prepare layout for update.
  MapLayout new_layout = map->layout();

  addVoxelMean(new_layout);
  // Cache the layer index.
  int layer_index = int(addCovariance(new_layout)->layerIndex());

  if (ndt_mode == NdtMode::kTraversability)
  {
    addIntensity(new_layout);
    addHitMissCount(new_layout);
  }

  // Update the map.
  map->updateLayout(new_layout);

  return layer_index;
}
}  // namespace ohm
