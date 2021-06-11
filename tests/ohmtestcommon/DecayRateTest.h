// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohm/OhmConfig.h>

#include <functional>

namespace ohm
{
class OccupancyMap;
class RayMapper;
}  // namespace ohm

namespace ohmtestcommon
{
namespace decay
{
/// Test ray trace accumulation for rays traced into a target voxel.
/// @param map The map object to trace into. The map should be empty/clear.
/// @param mapper The ray tracing algorithm.
/// @param pre_validation Function to call before performing validation. For example, allows GPU to CPU sync.
void testInto(ohm::OccupancyMap &map, ohm::RayMapper &mapper, std::function<void()> pre_validation = {});
/// Test ray trace accumulation for rays traced through a target voxel.
/// @param map The map object to trace into. The map should be empty/clear.
/// @param mapper The ray tracing algorithm.
/// @param pre_validation Function to call before performing validation. For example, allows GPU to CPU sync.
void testThrough(ohm::OccupancyMap &map, ohm::RayMapper &mapper, std::function<void()> pre_validation = {});
}  // namespace decay
}  // namespace ohmtestcommon
