// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelData.h>

#include <ohmgpu/GpuMap.h>

#include <3esservermacros.h>

#include <cstdio>
#include <vector>

#include <glm/ext.hpp>

#include <gtest/gtest.h>
#include "ohm/ClearingPattern.h"
#include "ohm/OccupancyMap.h"

using namespace ohm;

namespace raypattern
{
TEST(RayPattern, Clearing)
{
  // Build a map of a solid line of voxels.
  const unsigned voxel_count = 20;
  ohm::OccupancyMap map;
  ohm::GpuMap gpu_map(&map, true);

  // Ensure a single miss erases a single hit.
  map.setHitProbability(0.51f);
  map.setMissProbability(0.0f);

  ohm::Key key(0, 0, 0, 0, 0, 0);
  {
    ohm::Voxel<float> voxel_write(&map, map.layout().occupancyLayer());
    ASSERT_TRUE(voxel_write.isLayerValid());
    for (unsigned i = 0; i < voxel_count; ++i)
    {
      voxel_write.setKey(key);
      ASSERT_TRUE(voxel_write.isValid());
      ohm::integrateHit(voxel_write);
      ASSERT_TRUE(ohm::isOccupied(voxel_write));
      map.moveKey(key, 1, 0, 0);
    }
  }

  // Now create a clearing pattern of a single ray large enough to erase all the voxels.
  // We build the line along Y and rotate it to X with a quaternion.
  RayPattern line_pattern;
  line_pattern.addPoint(glm::dvec3(0, map.resolution() * voxel_count, 0));
  ClearingPattern clearing(&line_pattern, false);

  // Set the key to the voxel we want to check.
  key = ohm::Key(0, 0, 0, 0, 0, 0);
  // Translate the ray pattern to start at the centre of the voxel at key.
  const glm::dvec3 pattern_translate = map.voxelCentreGlobal(key);
  // Setup the quaternion to rotate from Y to X axis.
  const glm::dquat rotation = glm::angleAxis(-0.5 * M_PI, glm::dvec3(0, 0, 1));
  ohm::Voxel<const float> voxel_read(&map, map.layout().occupancyLayer());
  ASSERT_TRUE(voxel_read.isLayerValid());
  for (unsigned i = 0; i < voxel_count; ++i)
  {
    // Validate we have an occupied voxel at key.
    voxel_read.setKey(key);
    ASSERT_TRUE(voxel_read.isValid());
    ASSERT_TRUE(ohm::isOccupied(voxel_read));

    // Apply the pattern.
    clearing.apply(&gpu_map, pattern_translate, rotation);
    gpu_map.syncVoxels();

    // Validate we have removed a voxel.
    ASSERT_FALSE(ohm::isOccupied(voxel_read));

    // Next key.
    map.moveKey(key, 1, 0, 0);
  }
  voxel_read.reset();
}
}  // namespace raypattern
