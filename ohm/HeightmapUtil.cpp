// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapUtil.h"

#include "DefaultLayer.h"
#include "HeightmapVoxel.h"
#include "MapInfo.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "VoxelLayout.h"

#include "private/HeightmapDetail.h"

namespace ohm
{
namespace heightmap
{
int setupHeightmap(ohm::OccupancyMap &heightmap, HeightmapDetail &detail)
{
  // Setup the heightmap voxel layout.
  MapLayout &layout = heightmap.layout();

  layout.filterLayers({ default_layer::occupancyLayerName(), default_layer::meanLayerName() });

  MapLayer *layer;
  VoxelLayout voxels;

  const float max_clearance = std::numeric_limits<float>::max();
  int max_clearance_int = 0;
  static_assert(sizeof(max_clearance) == sizeof(max_clearance_int), "size mismatch");

  memcpy(&max_clearance_int, &max_clearance, sizeof(max_clearance));

  size_t clear_value = 0;
  // Initialise the data structure to have both ranges at float max.
  memset(&clear_value, max_clearance_int, sizeof(clear_value));
  layer = layout.addLayer(HeightmapVoxel::kHeightmapLayer, 0);
  int layer_index = static_cast<int>(layer->layerIndex());
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);
  voxels.addMember("normal_x", DataType::kFloat, 0);
  voxels.addMember("normal_y", DataType::kFloat, 0);
  voxels.addMember("normal_z", DataType::kFloat, 0);
  voxels.addMember("flags", DataType::kUInt8, 0);
  voxels.addMember("reserved0", DataType::kUInt8, 0);
  voxels.addMember("reserved1", DataType::kUInt8, 0);
  voxels.addMember("reserved2", DataType::kUInt8, 0);

  detail.toMapInfo(heightmap.mapInfo());

  return layer_index;
}

UpAxis queryHeightmapAxis(const MapInfo &info)
{
  const MapValue value = info.get("heightmap-axis");
  if (value.isValid())
  {
    return UpAxis(int(value));
  }

  return UpAxis::kZ;
}
}  // namespace heightmap
}  // namespace ohm
