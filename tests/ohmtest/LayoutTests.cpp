// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelLayout.h>

#include <ohmutil/OhmUtil.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include "OhmTestUtil.h"
#include <gtest/gtest.h>

using namespace ohm;
// using namespace ohmutil;

TEST(Layout, Default)
{
  OccupancyMap map(1.0);

  const MapLayout &layout = map.layout();

  EXPECT_EQ(layout.layerCount(), 2);
  const MapLayer *occupancy_layer = layout.layer(default_layer::occupancyLayerName());
  const MapLayer *clearance_layer = layout.layer(default_layer::clearanceLayerName());
  ASSERT_NE(occupancy_layer, nullptr);
  ASSERT_NE(clearance_layer, nullptr);

  EXPECT_EQ(occupancy_layer->layerIndex(), 0);
  EXPECT_EQ(clearance_layer->layerIndex(), 1);

  EXPECT_EQ(occupancy_layer->voxelByteSize(), sizeof(float));
  EXPECT_EQ(clearance_layer->voxelByteSize(), sizeof(float));

  VoxelLayoutConst occupancy_voxel = occupancy_layer->voxelLayout();
  VoxelLayoutConst clearance_voxel = clearance_layer->voxelLayout();

  EXPECT_EQ(occupancy_voxel.memberCount(), 1);
  EXPECT_STREQ(occupancy_voxel.memberName(0), default_layer::occupancyLayerName());
  EXPECT_EQ(occupancy_voxel.memberOffset(0), 0);
  EXPECT_EQ(occupancy_voxel.memberSize(0), sizeof(float));

  EXPECT_EQ(clearance_voxel.memberCount(), 1);
  EXPECT_STREQ(clearance_voxel.memberName(0), default_layer::clearanceLayerName());
  EXPECT_EQ(clearance_voxel.memberOffset(0), 0);
  EXPECT_EQ(clearance_voxel.memberSize(0), sizeof(float));
}


TEST(Layout, Filter)
{
  OccupancyMap map(1.0);

  MapLayout &layout = map.layout();

  EXPECT_EQ(layout.layerCount(), 2);
  const MapLayer *occupancy_layer = layout.layer(default_layer::occupancyLayerName());
  ASSERT_NE(occupancy_layer, nullptr);

  // Remove the occupancy layer.
  layout.filterLayers({default_layer::clearanceLayerName()});

  EXPECT_EQ(layout.layerCount(), 1);
  occupancy_layer = layout.layer(default_layer::occupancyLayerName());
  const MapLayer *clearance_layer = layout.layer(default_layer::clearanceLayerName());
  ASSERT_EQ(occupancy_layer, nullptr);
  ASSERT_NE(clearance_layer, nullptr);

  EXPECT_EQ(clearance_layer->layerIndex(), 0);

  EXPECT_EQ(clearance_layer->voxelByteSize(), sizeof(float));

  VoxelLayoutConst clearance_voxel = clearance_layer->voxelLayout();

  EXPECT_EQ(clearance_voxel.memberCount(), 1);
  EXPECT_STREQ(clearance_voxel.memberName(0), default_layer::clearanceLayerName());
  EXPECT_EQ(clearance_voxel.memberOffset(0), 0);
  EXPECT_EQ(clearance_voxel.memberSize(0), sizeof(float));

  layout.clear();
  EXPECT_EQ(layout.layerCount(), 0);
}
