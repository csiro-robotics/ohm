// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapUtil.h"

#include "private/HeightmapDetail.h"

#include "HeightmapVoxel.h"

#include <ohm/DefaultLayer.h>
#include <ohm/MapInfo.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelLayout.h>

#include <cassert>
#include <sstream>

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

  layer = layout.addLayer(HeightmapVoxel::kHeightmapLayer, 0);
  int layer_index = static_cast<int>(layer->layerIndex());
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);
  voxels.addMember("normal_x", DataType::kFloat, 0);
  voxels.addMember("normal_y", DataType::kFloat, 0);
  voxels.addMember("normal_z", DataType::kFloat, 0);
  voxels.addMember("layer", DataType::kUInt8, 0);
  voxels.addMember("reserved", DataType::kUInt8, 0);
  voxels.addMember("contributing_samples", DataType::kUInt16, 0);
  int r = 0;
  while (voxels.voxelByteSize() < sizeof(HeightmapVoxel))
  {
    std::ostringstream name_stream;
    name_stream << "reserved" << r++;
    const auto name = name_stream.str();
    voxels.addMember(name.c_str(), DataType::kUInt8, 0);
  }
  assert(voxels.voxelByteSize() == sizeof(HeightmapVoxel));

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

double queryHeightmapClearance(const MapInfo &info)
{
  const MapValue value = info.get("heightmap-clearance");
  if (value.isValid())
  {
    return double(value);
  }

  return 0.0;
}

std::array<int, 3> ohmheightmap_API heightmapAxisIndices(UpAxis up_axis)
{
  std::array<int, 3> axis_indices;
  switch (up_axis)
  {
  case UpAxis::kX:
    /* fallthrough */
  case UpAxis::kNegX:
    axis_indices[0] = 1;
    axis_indices[1] = 2;
    axis_indices[2] = 0;
    break;
  case UpAxis::kY:
    /* fallthrough */
  case UpAxis::kNegY:
    axis_indices[0] = 0;
    axis_indices[1] = 2;
    axis_indices[2] = 1;
    break;
  default:
  case UpAxis::kZ:
    /* fallthrough */
  case UpAxis::kNegZ:
    axis_indices[0] = 0;
    axis_indices[1] = 1;
    axis_indices[2] = 2;
    break;
  }

  return axis_indices;
}
}  // namespace heightmap
}  // namespace ohm
