// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/Key.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperNdt.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>

#include <ohmtestcommon/TraversalTest.h>

#include <glm/vec3.hpp>

namespace traversal
{
TEST(Traversal, Through)
{
  // For this test, we want to ensure that the traversal accumulates correctly. To do that we select a set of rays
  // which pass through a voxel perpendicular to various faces of that voxel. In this way we know that the traversal
  // should accumulate at approximately the voxel resolution per ray.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::RayMapperOccupancy mapper(&map);
  ohmtestcommon::traversal::testThrough(map, mapper);
}

TEST(Traversal, ThroughNdt)
{
  // As Through, but using the NDT mapper.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::NdtMap ndt(&map, true);
  ohm::RayMapperNdt mapper(&ndt);
  ohmtestcommon::traversal::testThrough(map, mapper);
}

TEST(Traversal, Into)
{
  // For this test, we want to ensure that the traversal accumulates correctly in the final voxel. To do that we select
  // a set of rays which end in a voxel a voxel entering perpendicular to various faces of that voxel. In this way we
  // know that the traversal should accumulate at approximately half the voxel resolution per ray.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::RayMapperOccupancy mapper(&map);
  ohmtestcommon::traversal::testInto(map, mapper);
}

TEST(Traversal, IntoNdt)
{
  // As Into, but using the NDT mapper.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::NdtMap ndt(&map, true);
  ohm::RayMapperNdt mapper(&ndt);
  ohmtestcommon::traversal::testInto(map, mapper);
}
}  // namespace traversal
