// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmCloud.h"

#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/Query.h>
#include <ohm/VoxelData.h>

#include <ohmutil/PlyMesh.h>

#include <algorithm>

using namespace ohm;

namespace ohmtools
{
void saveCloud(const char *file_name, const ohm::OccupancyMap &map, const ProgressCallback &prog)
{
  ohm::PlyMesh ply;
  glm::vec3 v;
  const size_t region_count = map.regionCount();
  size_t processed_region_count = 0;
  glm::i16vec3 last_region = map.begin().key().regionKey();

  Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  Voxel<const VoxelMean> mean(&map, map.layout().meanLayer());

  for (auto iter = map.begin(); iter != map.end(); ++iter)
  {
    mean.setKey(occupancy.setKey(*iter));
    if (last_region != iter.key().regionKey())
    {
      ++processed_region_count;
      if (prog)
      {
        prog(processed_region_count, region_count);
      }
      last_region = iter.key().regionKey();
    }
    if (isOccupied(occupancy))
    {
      v = positionSafe(mean) - map.origin();
      ply.addVertex(v);
    }
  }

  ply.save(file_name, true);
}


void saveQueryCloud(const char *file_name, const ohm::OccupancyMap &map, const ohm::Query &query, float colour_range,
                    const ProgressCallback &prog)
{
  const size_t result_count = query.numberOfResults();
  const ohm::Key *keys = query.intersectedVoxels();
  const float *ranges = query.ranges();
  glm::dvec3 voxel_pos;

  ohm::PlyMesh ply;
  for (size_t i = 0; i < result_count; ++i)
  {
    const ohm::Key &key = keys[i];
    uint8_t c = 255;
    if (colour_range > 0 && ranges)
    {
      float range_value = ranges[i];
      if (range_value < 0)
      {
        range_value = colour_range;
      }
      c = uint8_t(255 * std::max(0.0f, (colour_range - range_value) / colour_range));
    }
    voxel_pos = map.voxelCentreGlobal(key);
    ply.addVertex(voxel_pos, Colour(c, 128, 0));
    if (prog)
    {
      prog(i + 1, result_count);
    }
  }

  ply.save(file_name, true);
}


size_t saveClearanceCloud(const char *file_name, const ohm::OccupancyMap &map, const glm::dvec3 &min_extents,
                          const glm::dvec3 &max_extents, float colour_range, int export_type,
                          const ProgressCallback &prog)
{
  const size_t region_count = map.regionCount();
  size_t processed_region_count = 0;
  glm::dvec3 v;
  glm::i16vec3 last_region = map.begin().key().regionKey();
  ohm::PlyMesh ply;
  size_t point_count = 0;

  glm::i16vec3 min_region = map.regionKey(min_extents);
  glm::i16vec3 max_region = map.regionKey(max_extents);

  Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  Voxel<const float> clearance(&map, map.layout().clearanceLayer());

  const float colour_scale = colour_range;
  const auto map_end_iter = map.end();
  for (auto iter = map.begin(); iter != map_end_iter; ++iter)
  {
    clearance.setKey(occupancy.setKey(*iter));
    if (last_region != iter.key().regionKey())
    {
      ++processed_region_count;
      if (prog)
      {
        prog(processed_region_count, region_count);
      }
      last_region = iter.key().regionKey();
    }

    // Ensure the voxel is in a region we have calculated data for.
    if (min_region.x <= last_region.x && last_region.x <= max_region.x &&  //
        min_region.y <= last_region.y && last_region.y <= max_region.y &&  //
        min_region.z <= last_region.z && last_region.z <= max_region.z)
    {
      const bool export_match = !occupancy.isNull() && occupancyType(occupancy) >= export_type;
      if (export_match)
      {
        float range_value;
        clearance.read(&range_value);
        if (range_value < 0)
        {
          range_value = colour_range;
        }
        if (range_value >= 0)
        {
          uint8_t c = uint8_t(255 * std::max(0.0f, (colour_scale - range_value) / colour_scale));
          v = map.voxelCentreLocal(*iter);
          ply.addVertex(v, Colour(c, 128, 0));
          ++point_count;
        }
      }
    }
  }

  ply.save(file_name, true);

  return point_count;
}
}  // namespace ohmtools
