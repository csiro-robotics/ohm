// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Key.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperNdt.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>
#include <ohmgpu/GpuNdtMap.h>

#include <ohmtestcommon/TraversalTest.h>

#include <glm/vec3.hpp>

namespace traversaltests
{
TEST(Traversal, Through)
{
  // For this test, we want to ensure that the traversal accumulates correctly. To do that we select a set of rays
  // which pass through a voxel perpendicular to various faces of that voxel. In this way we know that the traversal
  // should accumulate at approximately the voxel resolution per ray.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::GpuMap mapper(&map, true);
  ohmtestcommon::traversal::testThrough(map, mapper, [&mapper] { mapper.syncVoxels(); });
}

TEST(Traversal, ThroughNoMean)
{
  // Same as Through, but without sub-voxel positioning. Realistically this isn't useful information as the traversal
  // model needs the number of hits on a voxel which is stored in the subvoxel layer. However, it is a valid code path
  // and we want to test that.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kTraversal);
  ohm::GpuMap mapper(&map, true);
  ohmtestcommon::traversal::testThrough(map, mapper, [&mapper] { mapper.syncVoxels(); });
}

TEST(Traversal, ThroughNdt)
{
  // As Through, but using the NDT mapper.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::GpuNdtMap mapper(&map, true);
  ohmtestcommon::traversal::testThrough(map, mapper, [&mapper] { mapper.syncVoxels(); });
}

TEST(Traversal, ThroughNdtTm)
{
  // As Through, but using the NDT mapper in TM mode. Explicitly tested because it used a different kernel invocation.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::GpuNdtMap mapper(&map, true, ohm::NdtMode::kTraversability);
  ohmtestcommon::traversal::testThrough(map, mapper, [&mapper] { mapper.syncVoxels(); });
}

TEST(Traversal, Into)
{
  // For this test, we want to ensure that the traversal accumulates correctly in the final voxel. To do that we select
  // a set of rays which end in a voxel a voxel entering perpendicular to various faces of that voxel. In this way we
  // know that the traversal should accumulate at approximately half the voxel resolution per ray.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::GpuMap mapper(&map, true);
  ohmtestcommon::traversal::testInto(map, mapper, [&mapper] { mapper.syncVoxels(); });
}

TEST(Traversal, IntoNoMean)
{
  // Same as Into without voxel mean. See ThroughNoMean for justification.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kTraversal);
  ohm::GpuMap mapper(&map, true);
  ohmtestcommon::traversal::testInto(map, mapper, [&mapper] { mapper.syncVoxels(); });
}

TEST(Traversal, IntoNdt)
{
  // As Into, but using the NDT mapper.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::GpuNdtMap mapper(&map, true);
  ohmtestcommon::traversal::testInto(map, mapper, [&mapper] { mapper.syncVoxels(); });
}

TEST(Traversal, IntoNdtTm)
{
  // As Into, but using the NDT mapper in TM mode. Explicitly tested because it used a different kernel invocation.
  ohm::OccupancyMap map(0.1f, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal);
  ohm::GpuNdtMap mapper(&map, true, ohm::NdtMode::kTraversability);
  ohmtestcommon::traversal::testInto(map, mapper, [&mapper] { mapper.syncVoxels(); });
}
}  // namespace traversaltests
