// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmtestcommon/OhmTestUtil.h"

#include <ohm/DefaultLayer.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/MapSerialise.h>
#include <ohm/Mapper.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohmgpu/ClearanceProcess.h>
#include <ohmgpu/GpuCache.h>
#include <ohmgpu/GpuMap.h>
#include <ohmgpu/OhmGpu.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

using namespace ohm;

namespace mappertests
{
// NOTE: This test has been broken for some time. This has been hidden by the fact that the validation has been using
// exactly the same, incorrect results to compare against.
TEST(Mapper, Clearance)
{
  // This is a very rudimentary test for the ClearanceProcess. We build a map, generate the clearance values, clone
  // the map and generate it's clearance values and ensure the results match.
  // This test is only partly maintained and the ClearanceProcess is an experimental feature.

  Profile profile;
  const double map_extents = 10.0;
  const double noise = 1.0;
  const double resolution = 0.25;
  const unsigned ray_count = 1024 * 128;
  const unsigned batch_size = 1024 * 2;
  const glm::u8vec3 region_size(32);
  const float clearance_range = 3.0f;
  // const unsigned clearance_flags = kQfGpuEvaluate | kQfUnknownAsOccupied;
  const unsigned clearance_flags = kQfGpuEvaluate;

  // Make some rays.
  std::mt19937 rand_engine;
  std::uniform_real_distribution<double> unit_rand(-1, 1);
  std::uniform_real_distribution<double> length_rand(map_extents - noise, map_extents);
  std::vector<glm::dvec3> rays;

  // Build rays to create a generally spherical shell.
  while (rays.size() < ray_count * 2)
  {
    rays.push_back(glm::dvec3(0.05));
    glm::dvec3 ray_end(unit_rand(rand_engine), unit_rand(rand_engine), unit_rand(rand_engine));
    ray_end = glm::normalize(ray_end);
    ray_end *= length_rand(rand_engine);
    rays.push_back(ray_end);
  }

  // Setup the map and mapper
  // Test basic map populate using GPU and ensure it matches CPU (close enough).
  OccupancyMap map(resolution, region_size, MapFlag::kNone);
  Mapper mapper(&map);

  ClearanceProcess *clearance_process = new ClearanceProcess(clearance_range, clearance_flags);
  mapper.addProcess(clearance_process);
  clearance_process->ensureClearanceLayer(map);
  // clearance_process = nullptr;  // Ownership lost.

  // Insantiate the GPU map after the clearance layer has been created. Saves resetting the GPU cache.
  GpuMap gpu_map(&map, true, unsigned(batch_size * 2));  // Borrow pointer.

  ASSERT_TRUE(gpu_map.gpuOk());

  std::cout << "Integrating " << rays.size() / 2 << " rays.\n";

  ProfileMarker populate_marker("populate", false);
  ProfileMarker mapper_marker("mapper", false);
  for (size_t i = 0; i < rays.size(); i += batch_size * 2)
  {
    populate_marker.restart();
    const unsigned point_count = unsigned(std::min<size_t>(batch_size * 2, rays.size() - i));
    gpu_map.integrateRays(rays.data() + i, point_count);
    populate_marker.end();
    mapper_marker.restart();
    // FIXME(KS): As part of this tests lack of maintenance, I've commented the update here in favour of a full update
    // later. This update introduces some non-determinism and error in the final values.
    // // Partial update unless this is the last update. Full update for the last update (below).
    // mapper.update(0.01);
    mapper_marker.end();
  }
  populate_marker.end();

  {
    std::cout << "Finalising clearance" << std::endl;
    gpu_map.syncVoxels();
    ProfileMarker mapper_finalise_marker("mapper-finalise");
    mapper.update(0.0);
  }
  {
    ProfileMarker sync_marker("sync");
    std::cout << "Syncing occupancy" << std::endl;
    gpu_map.syncVoxels();
  }

  // Clone the map and calculate clearance values accross the whole map for verification.
  std::cout << "Cloning map" << std::endl;
  std::unique_ptr<OccupancyMap> clone_map(map.clone());
  {
    std::cout << "Calculating cloned clearance" << std::endl;
    ClearanceProcess clone_clearance(clearance_range, clearance_flags);
    ProfileMarker clearance_only_marker("clearanceOnly");
    // Update via explicit extents to force recalculation.
    glm::dvec3 min_ext{};
    glm::dvec3 max_ext{};
    clone_map->calculateExtents(&min_ext, &max_ext);
    clone_clearance.calculateForExtents(*clone_map, min_ext, max_ext);
  }

  std::cout << "Saving maps" << std::endl;
  ohm::save("mapper-map.ohm", map);
  ohm::save("mapper-clone.ohm", *clone_map);
  ohmtools::saveCloud("mapper-map.ply", map);
  ohmtools::saveCloud("mapper-clone.ply", *clone_map);
  ohmtools::saveClearanceCloud("mapper-map-clearance.ply", map, glm::dvec3(-10), glm::dvec3(10), clearance_range);
  ohmtools::saveClearanceCloud("mapper-clone-clearance.ply", *clone_map, glm::dvec3(-10), glm::dvec3(10),
                               clearance_range);

  // Compare maps.
  std::cout << "Comparing maps" << std::endl;
  ohmtestutil::compareMaps(map, *clone_map, ohmtestutil::kCfCompareExtended | ohmtestutil::kCfExpectClearance);
}
}  // namespace mappertests
