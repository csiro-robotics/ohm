// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancymapdetail.h"

#include "maplayer.h"
#include "maplayout.h"
#include "ohmvoxellayout.h"
#include "occupancymap.h"

#include "gpucache.h"

#include <algorithm>

using namespace ohm;

OccupancyMapDetail::~OccupancyMapDetail()
{
  delete gpuCache;
}


void OccupancyMapDetail::setDefaultLayout()
{
  // Setup the default layers
  layout.clear();

  MapLayer *layer;
  VoxelLayout voxel;
  size_t clearValue = 0;

  const float invalidMarkerValue = NodeBase::invalidMarkerValue();

  clearValue = 0;
  memcpy(&clearValue, &invalidMarkerValue, std::min(sizeof(invalidMarkerValue), sizeof(clearValue)));
  layer = layout.addLayer("occupancy", 0);
  voxel = layer->voxelLayout();
  voxel.addMember("occupancy", DataType::Float, clearValue);

  const float defaultClearance = -1.0f;
  clearValue = 0;
  memcpy(&clearValue, &defaultClearance, std::min(sizeof(defaultClearance), sizeof(clearValue)));
  layer = layout.addLayer("clearance", 0);
  voxel = layer->voxelLayout();
  voxel.addMember("clearance", DataType::Float, clearValue);

  clearValue = 0;
  memcpy(&clearValue, &defaultClearance, std::min(sizeof(defaultClearance), sizeof(clearValue)));
  layer = layout.addLayer("coarseClearance", 1);
  voxel = layer->voxelLayout();
  voxel.addMember("coarseClearance", DataType::Float, clearValue);
}


void OccupancyMapDetail::copyFrom(const OccupancyMapDetail &other)
{
  origin = other.origin;
  regionSpatialDimensions = other.regionSpatialDimensions;
  regionVoxelDimensions = other.regionVoxelDimensions;
  resolution = other.resolution;
  stamp = other.stamp;
  occupancyThresholdValue = other.occupancyThresholdValue;
  occupancyThresholdProbability = other.occupancyThresholdProbability;
  hitValue = other.hitValue;
  hitProbability = other.hitProbability;
  missValue = other.missValue;
  missProbability = other.missProbability;
  minNodeValue = other.minNodeValue;
  maxNodeValue = other.maxNodeValue;
  saturateAtMinValue = other.saturateAtMinValue;
  saturateAtMaxValue = other.saturateAtMaxValue;
  layout = MapLayout(other.layout);
}
