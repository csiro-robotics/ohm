// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OccupancyMapDetail.h"

#include "MapLayer.h"
#include "MapLayout.h"
#include "VoxelLayout.h"
#include "OccupancyMap.h"

#include "GpuCache.h"

#include <algorithm>

using namespace ohm;

OccupancyMapDetail::~OccupancyMapDetail()
{
  delete gpu_cache;
}


void OccupancyMapDetail::setDefaultLayout()
{
  // Setup the default layers
  layout.clear();

  MapLayer *layer;
  VoxelLayout voxel;
  size_t clear_value;

  const float invalid_marker_value = voxel::invalidMarkerValue();

  clear_value = 0;
  memcpy(&clear_value, &invalid_marker_value, std::min(sizeof(invalid_marker_value), sizeof(clear_value)));
  layer = layout.addLayer("occupancy", 0);
  voxel = layer->voxelLayout();
  voxel.addMember("occupancy", DataType::kFloat, clear_value);

  const float default_clearance = -1.0f;
  clear_value = 0;
  memcpy(&clear_value, &default_clearance, std::min(sizeof(default_clearance), sizeof(clear_value)));
  layer = layout.addLayer("clearance", 0);
  voxel = layer->voxelLayout();
  voxel.addMember("clearance", DataType::kFloat, clear_value);

  clear_value = 0;
  memcpy(&clear_value, &default_clearance, std::min(sizeof(default_clearance), sizeof(clear_value)));
  layer = layout.addLayer("coarseClearance", 1);
  voxel = layer->voxelLayout();
  voxel.addMember("coarseClearance", DataType::kFloat, clear_value);
}


void OccupancyMapDetail::copyFrom(const OccupancyMapDetail &other)
{
  origin = other.origin;
  region_spatial_dimensions = other.region_spatial_dimensions;
  region_voxel_dimensions = other.region_voxel_dimensions;
  resolution = other.resolution;
  stamp = other.stamp;
  occupancy_threshold_value = other.occupancy_threshold_value;
  occupancy_threshold_probability = other.occupancy_threshold_probability;
  hit_value = other.hit_value;
  hit_probability = other.hit_probability;
  miss_value = other.miss_value;
  miss_probability = other.miss_probability;
  min_voxel_value = other.min_voxel_value;
  max_voxel_value = other.max_voxel_value;
  saturate_at_min_value = other.saturate_at_min_value;
  saturate_at_max_value = other.saturate_at_max_value;
  layout = MapLayout(other.layout);
}
