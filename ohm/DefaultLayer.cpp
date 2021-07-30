// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "DefaultLayer.h"

#include "MapLayer.h"
#include "MapLayout.h"
#include "VoxelMean.h"

#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "CovarianceVoxel.h"

#include <stdexcept>

namespace ohm
{
namespace default_layer
{
const char *occupancyLayerName()
{
  return "occupancy";
}
const char *meanLayerName()
{
  return "mean";
}
const char *traversalLayerName()
{
  return "traversal";
}
const char *covarianceLayerName()
{
  return "covariance";
}
const char *clearanceLayerName()
{
  return "clearance";
}
const char *intensityLayerName()
{
  return "intensity";
}
const char *hitMissCountLayerName()
{
  return "hit_miss_count";
}
const char *touchTimeLayerName()
{
  return "touch_time";
}
const char *incidentNormalLayerName()
{
  return "incident_normal";
}
}  // namespace default_layer


MapLayer *addOccupancy(MapLayout &layout)
{
  int layer_index = layout.occupancyLayer();
  if (layer_index != -1)
  {
    // Already present.
    return layout.layerPtr(layer_index);
  }

  MapLayer *layer = layout.addLayer(default_layer::occupancyLayerName(), 0);

  const float invalid_marker_value = unobservedOccupancyValue();
  size_t clear_value = 0;
  memcpy(&clear_value, &invalid_marker_value, sizeof(invalid_marker_value));

  layer->voxelLayout().addMember(default_layer::occupancyLayerName(), DataType::kFloat, clear_value);

  return layer;
}


MapLayer *addVoxelMean(MapLayout &layout)
{
  int layer_index = layout.meanLayer();
  if (layer_index != -1)
  {
    // Already present.
    return layout.layerPtr(layer_index);
  }

  // Add the mean layer.
  MapLayer *layer = layout.addLayer(default_layer::meanLayerName());

  const size_t clear_value = 0u;
  layer->voxelLayout().addMember("coord", DataType::kUInt32, clear_value);
  layer->voxelLayout().addMember("count", DataType::kUInt32, clear_value);

  if (layer->voxelByteSize() != sizeof(VoxelMean))
  {
    throw std::runtime_error("VoxelMean layer size mismatch");
  }

  return layer;
}


MapLayer *addTraversal(MapLayout &layout)
{
  int layer_index = layout.traversalLayer();
  if (layer_index != -1)
  {
    // Already present.
    return layout.layerPtr(layer_index);
  }

  // Add the mean layer.
  MapLayer *layer = layout.addLayer(default_layer::traversalLayerName());

  const size_t clear_value = 0u;
  layer->voxelLayout().addMember("traversal", DataType::kFloat, clear_value);

  if (layer->voxelByteSize() != sizeof(float))
  {
    throw std::runtime_error("Traversal ayer size mismatch");
  }

  return layer;
}


MapLayer *addCovariance(MapLayout &layout)
{
  if (const MapLayer *layer = layout.layer(default_layer::covarianceLayerName()))
  {
    // Already present.
    // Oddities below as we can only retrieve const layers by name.
    return layout.layerPtr(layer->layerIndex());
  }

  MapLayer *layer = layout.addLayer(default_layer::covarianceLayerName());
  VoxelLayout voxel = layer->voxelLayout();
  // Add members to represent an upper triangular covariance matrix.
  voxel.addMember("P00", DataType::kFloat, 0);
  voxel.addMember("P01", DataType::kFloat, 0);
  voxel.addMember("P11", DataType::kFloat, 0);
  voxel.addMember("P02", DataType::kFloat, 0);
  voxel.addMember("P12", DataType::kFloat, 0);
  voxel.addMember("P22", DataType::kFloat, 0);

  if (layer->voxelByteSize() != sizeof(CovarianceVoxel))
  {
    throw std::runtime_error("CovarianceVoxel layer size mismatch");
  }

  return layer;
}


MapLayer *addClearance(MapLayout &layout)
{
  int layer_index = layout.clearanceLayer();
  if (layer_index != -1)
  {
    // Already present.
    return layout.layerPtr(layer_index);
  }

  const float default_clearance = -1.0f;
  size_t clear_value = 0;
  memcpy(&clear_value, &default_clearance, std::min(sizeof(default_clearance), sizeof(clear_value)));
  MapLayer *layer = layout.addLayer(default_layer::clearanceLayerName(), 0);
  VoxelLayout voxel = layer->voxelLayout();
  voxel.addMember(default_layer::clearanceLayerName(), DataType::kFloat, clear_value);

  if (layer->voxelByteSize() != sizeof(float))
  {
    throw std::runtime_error("Clearance layer size mismatch");
  }

  return layer;
}

MapLayer *addIntensity(MapLayout &layout)
{
  if (const MapLayer *layer = layout.layer(default_layer::intensityLayerName()))
  {
    // Already present.
    // Oddities below as we can only retrieve const layers by name.
    return layout.layerPtr(layer->layerIndex());
  }

  MapLayer *layer = layout.addLayer(default_layer::intensityLayerName());
  VoxelLayout voxel = layer->voxelLayout();
  // Add members to represent mean and covariance of points in voxel
  voxel.addMember("mean", DataType::kFloat, 0);
  voxel.addMember("cov", DataType::kFloat, 0);

  if (layer->voxelByteSize() != sizeof(IntensityMeanCov))
  {
    throw std::runtime_error("Intensity layer size mismatch");
  }

  return layer;
}

MapLayer *addHitMissCount(MapLayout &layout)
{
  if (const MapLayer *layer = layout.layer(default_layer::hitMissCountLayerName()))
  {
    // Already present.
    // Oddities below as we can only retrieve const layers by name.
    return layout.layerPtr(layer->layerIndex());
  }

  MapLayer *layer = layout.addLayer(default_layer::hitMissCountLayerName());
  VoxelLayout voxel = layer->voxelLayout();
  // Add members to represent hit and miss count in voxel, according to NDT-TM
  voxel.addMember("hit_count", DataType::kUInt32, 0);
  voxel.addMember("miss_count", DataType::kUInt32, 0);

  if (layer->voxelByteSize() != sizeof(HitMissCount))
  {
    throw std::runtime_error("HitMissCount layer size mismatch");
  }

  return layer;
}

MapLayer *addTouchTime(MapLayout &layout)
{
  if (const MapLayer *layer = layout.layer(default_layer::touchTimeLayerName()))
  {
    // Already present.
    // Oddities below as we can only retrieve const layers by name.
    return layout.layerPtr(layer->layerIndex());
  }

  MapLayer *layer = layout.addLayer(default_layer::touchTimeLayerName());
  VoxelLayout voxel = layer->voxelLayout();
  // Add members to represent hit and miss count in voxel, according to NDT-TM
  voxel.addMember("touch", DataType::kUInt32, 0);

  if (layer->voxelByteSize() != sizeof(uint32_t))
  {
    throw std::runtime_error("Touch time layer size mismatch");
  }

  return layer;
}

MapLayer *addIncidentNormal(MapLayout &layout)
{
  if (const MapLayer *layer = layout.layer(default_layer::incidentNormalLayerName()))
  {
    // Already present.
    // Oddities below as we can only retrieve const layers by name.
    return layout.layerPtr(layer->layerIndex());
  }

  MapLayer *layer = layout.addLayer(default_layer::incidentNormalLayerName());
  VoxelLayout voxel = layer->voxelLayout();
  // Add members to represent hit and miss count in voxel, according to NDT-TM
  voxel.addMember("packed_normal", DataType::kUInt32, 0);

  if (layer->voxelByteSize() != sizeof(uint32_t))
  {
    throw std::runtime_error("Incident normal layer size mismatch");
  }

  return layer;
}
}  // namespace ohm
