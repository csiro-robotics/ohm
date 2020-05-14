// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "DefaultLayer.h"

#include "MapLayer.h"
#include "MapLayout.h"

namespace ohm
{
  namespace default_layer
  {
    const char *occupancyLayerName() { return "occupancy"; }
    const char *meanLayerName() { return "mean"; }
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
}  // namespace ohm
