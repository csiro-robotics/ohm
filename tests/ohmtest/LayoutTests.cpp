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

#include <gtest/gtest.h>
#include "ohmtestcommon/OhmTestUtil.h"

using namespace ohm;

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
  layout.filterLayers({ default_layer::clearanceLayerName() });

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


TEST(Layout, Structure)
{
  // Structure with variable packing and alignment.
  // We are simulating the following structure
  // struct LayoutTestStruct
  // {
  //   uint8_t m1_0;
  //   uint8_t m1_1;
  //   uint16_t m2_3;
  //   uint16_t m2_4;
  //   uint32_t m4_5;
  //   uint64_t m8_6;
  //   uint8_t m1_7;
  // };

  struct MemberInfo
  {
    const char *name;
    ohm::DataType::Type type;
    unsigned expected_offset;
    unsigned expected_cumulative_size;
  };

  static const MemberInfo members[] =  //
    {
      { "0", ohm::DataType::kUInt8, 0, 4 },     //
      { "1", ohm::DataType::kUInt8, 1, 4 },     //
      { "2", ohm::DataType::kUInt16, 2, 4 },    //
      { "3", ohm::DataType::kUInt16, 4, 8 },    //
      { "4", ohm::DataType::kUInt32, 8, 16 },   //
      { "5", ohm::DataType::kUInt64, 16, 24 },  //
      { "6", ohm::DataType::kUInt8, 24, 32 },   //
    };

  MapLayout layout;
  MapLayer *layer = layout.addLayer("structured");
  VoxelLayout voxel = layer->voxelLayout();

  size_t clear_value = 0u;

  size_t i = 0;
  for (const MemberInfo &member : members)
  {
    std::cout << "member " << member.name << std::endl;
    voxel.addMember(member.name, member.type, clear_value);
    EXPECT_EQ(voxel.memberOffset(i), member.expected_offset);
    EXPECT_EQ(voxel.voxelByteSize(), member.expected_cumulative_size);
    ++i;
  }
}
