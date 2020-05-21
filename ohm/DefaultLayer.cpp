// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "DefaultLayer.h"

#include "MapLayer.h"
#include "MapLayout.h"

#include <algorithm>

namespace ohm
{
  namespace default_layer
  {
    const char *occupancyLayerName() { return "occupancy"; }
    const char *meanLayerName() { return "mean"; }
    const char *covarianceLayerName() { return "covariance"; }
    const char *clearanceLayerName() { return "clearance"; }
  }  // namespace default_layer


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
    layer_index = layer->layerIndex();

    const size_t clear_value = 0u;
    layer->voxelLayout().addMember("coord", DataType::kUInt32, clear_value);
    layer->voxelLayout().addMember("count", DataType::kUInt32, clear_value);

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
    // Add members to represent a diagonal of the covariance matrix. This is an approximation of the full matrix
    // but it greatly reduces the per voxel memory usage.
    voxel.addMember("P00", DataType::kFloat, 0);
    voxel.addMember("P11", DataType::kFloat, 0);
    voxel.addMember("P22", DataType::kFloat, 0);
    voxel.addMember("P33", DataType::kFloat, 0);
    voxel.addMember("P44", DataType::kFloat, 0);
    voxel.addMember("P55", DataType::kFloat, 0);

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

    return layer;
  }
}  // namespace ohm
