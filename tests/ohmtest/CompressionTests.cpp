// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelBlock.h>
#include <ohm/VoxelBlockCompressionQueue.h>

TEST(Compression, Managed)
{
  ohm::VoxelBlockCompressionQueue compressor(true);  // Instantiate in test mode
  // Create a map in order to use the layout. DO NOT SET kCompressed. That would start a new compression object.
  ohm::OccupancyMap map(1.0, ohm::MapFlag::kNone);
  std::vector<ohm::VoxelBlock::Ptr> blocks;
  std::vector<uint8_t> compression_buffer;

  const size_t block_count = 10;
  // Create a set of blocks which we can register with the compression object.
  const ohm::MapLayer &layer = map.layout().layer(map.layout().occupancyLayer());
  for (size_t i = 0; i < block_count; ++i)
  {
    blocks.emplace_back();
    blocks[i].reset(new ohm::VoxelBlock(map.detail(), layer));
    compressor.push(blocks[i].get());
  }

  // Ensure the blocks are releases.
  blocks.clear();
  compressor.__tick(compression_buffer);
}
