// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <ohm/Aabb.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/LineWalk.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/Trace.h>
#include <ohm/VoxelData.h>

#include <ohm/CalculateSegmentKeys.h>
#include <ohm/OccupancyUtil.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>
#include <ohmutil/GlmStream.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>

#include <3esservermacros.h>
#ifdef TES_ENABLE
#include <shapes/3esmutablemesh.h>
#endif  // TES_ENABLE

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

using namespace ohm;

namespace linewalktests
{
void testWalk(const ohm::OccupancyMap &map, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
              bool include_end_point, bool log = false)
{
  // Validate the following conditions:
  // - First voxel is the start key.
  // - First voxel contains the start point.
  // - Each step is orthogonal to the last voxel.
  // - Each step is closer to the goal.
  // - End voxel is the end key.
  // - End voxel is contains the end point.

  ohm::Key start_key = map.voxelKey(start_point);
  ohm::Key end_key = map.voxelKey(end_point);

  TES_SERVER_UPDATE(ohm::g_tes, 0.0f);
  TES_LINE(ohm::g_tes, TES_COLOUR(Yellow), tes::Id(1), tes::Vector3d(glm::value_ptr(start_point)),
           tes::Vector3d(glm::value_ptr(end_point)));

  TES_STMT(unsigned id_offset = 0);
  TES_BOX_W(g_tes, TES_COLOUR(Green), tes::Id(++id_offset),
            tes::Spherical(tes::Vector3d(glm::value_ptr(map.voxelCentreGlobal(start_key))), 0.9 * map.resolution()));
  TES_BOX_W(g_tes, TES_COLOUR(Orange), tes::Id(++id_offset),
            tes::Spherical(tes::Vector3d(glm::value_ptr(map.voxelCentreGlobal(end_key))), 0.9 * map.resolution()));

  if (log)
  {
    std::cout << "walk: " << start_key << " -> " << end_key << std::endl;
  }

  std::string fail_info;
  {
    std::ostringstream str;
    str << "trace: " << start_point << " -> " << end_point << (include_end_point ? " include end" : " exclude end")
        << std::flush;
    fail_info = str.str();
  }

  bool is_first = true;
  ohm::Key last_key(nullptr);
  double last_distance = -1.0;
  unsigned voxel_count = 0;
  bool abort = false;

  const auto &visit_function = [&](const ohm::Key &current_key, double enter_range, double exit_range) -> bool  //
  {
    const glm::ivec3 key_delta = map.rangeBetween(last_key, current_key);
    const glm::ivec3 key_delta_to_end = map.rangeBetween(current_key, end_key);

    const glm::dvec3 voxel_pos = map.voxelCentreGlobal(current_key);

    if (log)
    {
      std::cout << voxel_count << ": " << current_key;
      if (!is_first)
      {
        std::cout << " step ";
        for (int j = 0; j < 3; ++j)
        {
          if (key_delta[j])
          {
            std::cout << j;
          }
        }
      }
      else
      {
        std::cout << "       ";
      }
      std::cout << " enter: " << enter_range << " exit: " << exit_range;
      std::cout << " to end " << key_delta_to_end;
      std::cout << std::endl;
    }
    if (!is_first)
    {
      EXPECT_NE(current_key, start_key) << fail_info;
      // Workout the delta.
      // Validate we have made one, orthogonal voxel step.
      EXPECT_NEAR(glm::length(glm::dvec3(key_delta)), 1.0, 1e-6) << fail_info;

      const double distance_to_end = glm::length(glm::dvec3(key_delta_to_end));
      // Valiate we are closer.
      EXPECT_LT(distance_to_end, last_distance) << fail_info;
      abort = distance_to_end >= last_distance;  // Prevent an infinite loop when it's not working.
      last_distance = distance_to_end;
    }
    else
    {
      const glm::dvec3 expected_point = start_point;
      EXPECT_EQ(current_key, start_key) << fail_info;
      EXPECT_EQ(current_key, map.voxelKey(expected_point)) << fail_info;
      last_distance = glm::length(glm::dvec3(key_delta_to_end));
      is_first = false;
    }

    // Make sure that the ray actually intersects the reported voxel. We use an epsilon to account for floating point
    // error near voxel boundaries.
    const double aabb_padding = 1e-3;
    const Aabb voxel_box(voxel_pos - 0.5 * glm::dvec3(map.resolution() + aabb_padding),
                         voxel_pos + 0.5 * glm::dvec3(map.resolution() + aabb_padding));
    const bool ray_intersect = voxel_box.rayIntersect(start_point, glm::normalize(end_point - start_point));
    if (!ray_intersect)
    {
      EXPECT_TRUE(ray_intersect) << fail_info;
    }

    TES_BOX_W(
      g_tes,
      (current_key == end_key) ?
        TES_COLOUR(Orange) :
        ((current_key == start_key) ? TES_COLOUR(Green) : (ray_intersect ? TES_COLOUR(White) : TES_COLOUR(Red))),
      tes::Id(voxel_count + 1 + id_offset), tes::Spherical(tes::Vector3d(glm::value_ptr(voxel_pos)), map.resolution()));
    std::ostringstream str;
    str << voxel_count << std::flush;
    TES_TEXT2D_SCREEN(g_tes, TES_COLOUR(Yellow), str.str().c_str(), tes::Id(),
                      tes::Spherical(tes::Vector3d(0.1, 0.1, 0)));

    last_key = current_key;
    ++voxel_count;
    TES_SERVER_UPDATE(ohm::g_tes, 0.0f);

    return !abort;
  };

  // Walk the ray.
  walkSegmentKeys(LineWalkContext(map, visit_function), start_point, end_point,
                  (!include_end_point ? kExcludeEndVoxel : 0u));

  const ohm::Key expected_key = end_key;
  // Validate last key.
  if (include_end_point)
  {
    EXPECT_EQ(last_key, expected_key) << fail_info;
    EXPECT_EQ(last_key, end_key) << fail_info;
  }
  else if (start_key != end_key)
  {
    // Ensure we are orthogonal to the end voxel.
    const glm::ivec3 key_delta_to_end = map.rangeBetween(last_key, end_key);
    const double distance_to_end = glm::length(glm::dvec3(key_delta_to_end));
    EXPECT_NEAR(distance_to_end, 1.0, 1e-6) << fail_info;
  }
  else
  {
    // Should not have made any iterations. last_distance should be < 0
    EXPECT_LT(last_distance, 0.0) << fail_info;
    EXPECT_EQ(last_key, ohm::Key(nullptr)) << fail_info;
  }
  TES_LINES_END(ohm::g_tes, tes::Id(1));
#ifdef TES_ENABLE
  for (unsigned i = 0; i < voxel_count + id_offset; ++i)
  {
    TES_BOX_END(ohm::g_tes, tes::Id(i + 1));
  }
#endif  // TES_ENABLE
}


TEST(LineWalk, Random)
{
  const unsigned test_count = 1000;
  ohm::Trace tes_trace("line_walk_random.3es", true);
  glm::dvec3 start;
  glm::dvec3 end;
  OccupancyMap map(0.1);
  std::vector<glm::dvec3> rays;
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  bool log = false;

  const auto restore_precision = std::cout.precision();
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

  // Try with different voxel offsets.
  const std::array<glm::dvec3, 2> map_origins = { glm::dvec3(0.0), glm::dvec3(0.5 * map.resolution()) };
  for (unsigned i = 0; i < test_count; ++i)
  {
    for (const auto &origin : map_origins)
    {
      map.setOrigin(origin);
      start = glm::dvec3(uniform(rng), uniform(rng), uniform(rng));
      end = glm::dvec3(uniform(rng), uniform(rng), uniform(rng));
      // std::cout << start << " -> " << end << std::endl;
      testWalk(map, start, end, true, log);
      testWalk(map, start, end, false, log);
    }
  }
  std::cout << std::setprecision(restore_precision);
}


TEST(LineWalk, Walk)
{
  ohm::Trace tes_trace("line_walk.3es", true);
  const int scale_iterations = 10;  // Test at different ranges.
  glm::dvec3 start(0.0, 0, 0);
  OccupancyMap map(0.1);
  bool log = false;

  const auto restore_precision = std::cout.precision();
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

#if 0
  const glm::dvec3 end(0, 0, 0);
  log = true;
  start = glm::dvec3(-1, 0, 1);
  // map.setOrigin(glm::dvec3(0.05));
  std::cout << "include end" << std::endl;
  testWalk(map, start, end, true, log);
  std::cout << "exclude end" << std::endl;
  testWalk(map, start, end, false, log);
  return;
#endif  // #

  // Try with different voxel offsets.
  const std::array<glm::dvec3, 2> map_origins = { glm::dvec3(0.0), glm::dvec3(0.5 * map.resolution()) };
  for (const auto &origin : map_origins)
  {
    map.setOrigin(origin);
    // Test at different voxel scales.
    for (int s = 1; s <= scale_iterations; ++s)
    {
      const double scale = double(s) / double(scale_iterations);
      std::cout << "Scale: " << s << std::endl;
      for (int z = -1; z <= 1; ++z)
      {
        for (int y = -1; y <= 1; ++y)
        {
          for (int x = -1; x <= 1; ++x)
          {
            const auto step_coord = glm::ivec3(x, y, z);
            glm::dvec3 end = glm::dvec3(step_coord);
            // Make sure we do not scale at s == 100 so as not to introduce new floating point errors.
            // We are testing exact voxel boundary walking and we don't want to perturb that.
            if (s != scale_iterations)
            {
              end *= scale;
            }
            // std::cout << start << " -> " << end << " << include end " << std::endl;
            testWalk(map, start, end, true, log);
            // std::cout << start << " -> " << end << " exclude end" << std::endl;
            testWalk(map, start, end, false, log);
          }
        }
      }
    }
  }
  std::cout << std::setprecision(restore_precision);
}
}  // namespace linewalktests
