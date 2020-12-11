// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmCloud.h"

#include <ohm/Heightmap.h>
#include <ohm/HeightmapUtil.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/Query.h>
#include <ohm/VoxelData.h>

#include <ohmutil/Colour.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/PlyPointStream.h>

#include <algorithm>
#include <fstream>
#include <limits>

namespace
{
ohm::PlyPointStream setupPlyStream(
  bool enable_colour, const std::initializer_list<ohm::PlyPointStream::Property> &additional_properties = {})
{
  std::vector<ohm::PlyPointStream::Property> ply_properties = {
    ohm::PlyPointStream::Property{ "x", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "y", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "z", ohm::PlyPointStream::Type::kFloat64 }
  };

  if (enable_colour)
  {
    ply_properties.emplace_back(ohm::PlyPointStream::Property{ "r", ohm::PlyPointStream::Type::kUInt8 });
    ply_properties.emplace_back(ohm::PlyPointStream::Property{ "g", ohm::PlyPointStream::Type::kUInt8 });
    ply_properties.emplace_back(ohm::PlyPointStream::Property{ "b", ohm::PlyPointStream::Type::kUInt8 });
  }

  for (const auto &additional_property : additional_properties)
  {
    ply_properties.emplace_back(additional_property);
  }

  return ohm::PlyPointStream(ply_properties);
}
}  // namespace

namespace ohmtools
{
const ohm::Colour ColourByHeight::s_default_from(128, 255, 0);
const ohm::Colour ColourByHeight::s_default_to(120, 0, 255);

ColourByHeight::ColourByHeight(const ohm::OccupancyMap &map)
  : ColourByHeight(map, s_default_from, s_default_to)
{}


ColourByHeight::ColourByHeight(const ohm::OccupancyMap &map, const ohm::Colour &from, const ohm::Colour &to)
  : colours({ from, to })
{
  map.calculateExtents(nullptr, nullptr, &range_);
}


ColourByHeight::ColourByHeight(const ohm::KeyRange &extents)
  : ColourByHeight(extents, s_default_from, s_default_to)
{}


ColourByHeight::ColourByHeight(const ohm::KeyRange &extents, const ohm::Colour &from, const ohm::Colour &to)
  : colours({ from, to })
  , range_(extents)
{}


ohm::Colour ColourByHeight::select(const ohm::Key &key) const
{
  const double vertical_range = std::max(1.0, double(range_.range()[up_axis_]));
  const double offset = std::max(
    0.0, std::min(double(ohm::OccupancyMap::rangeBetween(range_.minKey(), key, range_.regionDimensions())[up_axis_]),
                  vertical_range));
  const float factor = float(offset / vertical_range);
  return ohm::Colour::lerp(colours[0], colours[1], factor);
}


ohm::Colour ColourByHeight::select(const ohm::Voxel<const float> &occupancy) const
{
  return select(occupancy.key());
}


ColourHeightmapType::ColourHeightmapType(const ohm::OccupancyMap &map)
{
  heightmap_layer_ = map.layout().layerIndex(ohm::HeightmapVoxel::kHeightmapLayer);
}


ohm::Colour ColourHeightmapType::select(const ohm::Voxel<const float> &occupancy) const
{
  if (heightmap_layer_ >= 0 && occupancy.isValid())
  {
    ohm::Voxel<const ohm::HeightmapVoxel> heightmap_voxel(occupancy.map(), heightmap_layer_);
    heightmap_voxel.setKey(occupancy);
    if (heightmap_voxel.isValid())
    {
      const float occ_value = occupancy.data();
      if (occ_value == ohm::Heightmap::kHeightmapSurfaceValue)
      {
        return surface_colour;
      }
      else if (occ_value == ohm::Heightmap::kHeightmapVirtualSurfaceValue)
      {
        return virtual_colour;
      }
    }
  }

  return ohm::Colour(0, 0, 0, 255);
}


const ohm::Colour ColourByHeightmapClearance::s_default_low(255, 128, 0);
const ohm::Colour ColourByHeightmapClearance::s_default_high(255, 0, 0);

ColourByHeightmapClearance::ColourByHeightmapClearance(const ohm::OccupancyMap &map, double clearance_scale)
  : ColourByHeightmapClearance(map, s_default_low, s_default_high, clearance_scale)
{}


ColourByHeightmapClearance::ColourByHeightmapClearance(const ohm::OccupancyMap &map, const ohm::Colour &low,
                                                       const ohm::Colour &high, double clearance_scale)
{
  colours[0] = low;
  colours[1] = high;
  min_clearance_ = ohm::heightmap::queryHeightmapClearance(map.mapInfo());
  max_clearance_ = min_clearance_ * clearance_scale;
  heightmap_layer_ = map.layout().layerIndex(ohm::HeightmapVoxel::kHeightmapLayer);
}


ohm::Colour ColourByHeightmapClearance::select(const ohm::Voxel<const float> &occupancy) const
{
  if (heightmap_layer_ >= 0 && occupancy.isValid())
  {
    ohm::Voxel<const ohm::HeightmapVoxel> heightmap_voxel(occupancy.map(), heightmap_layer_);
    heightmap_voxel.setKey(occupancy);
    if (heightmap_voxel.isValid())
    {
      const double clearance = heightmap_voxel.data().clearance;
      const double lerp_range = (max_clearance_ - min_clearance_);
      const double lerp_factor =
        (lerp_range > std::numeric_limits<double>::epsilon()) ? (clearance - min_clearance_) / lerp_range : 1.0;
      return ohm::Colour::lerp(colours[0], colours[1], lerp_factor);
    }
  }

  return ohm::Colour(255, 255, 255, 255);
}


uint64_t saveCloud(const char *file_name, const ohm::OccupancyMap &map, const SaveCloudOptions &opt,
                   const ProgressCallback &prog)
{
  std::ofstream out(file_name, std::ios::binary);

  if (!out.is_open())
  {
    return 0;
  }

  // Work out if we need colour.
  auto colour_select = opt.colour_select;
  std::unique_ptr<ColourByHeight> colour_by_height;
  if (!colour_select && opt.allow_default_colour_selection)
  {
    colour_by_height = std::make_unique<ColourByHeight>(map);
    colour_select = [&colour_by_height](const ohm::Voxel<const float> &occupancy) {
      return colour_by_height->select(occupancy);
    };
  }

  // Setup the Ply stream.
  ohm::PlyPointStream ply = setupPlyStream(static_cast<bool>(colour_select));
  ply.open(out);

  glm::vec3 pos;
  const size_t region_count = map.regionCount();
  size_t processed_region_count = 0;
  glm::i16vec3 last_region = map.begin().key().regionKey();

  ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  auto mean = (opt.ignore_voxel_mean) ? ohm::Voxel<const ohm::VoxelMean>() :
                                        ohm::Voxel<const ohm::VoxelMean>(&map, map.layout().meanLayer());

  uint64_t point_count = 0;
  for (auto iter = map.begin(); iter != map.end(); ++iter)
  {
    ohm::setVoxelKey(iter, occupancy, mean);
    // Progress update.
    if (last_region != iter.key().regionKey())
    {
      ++processed_region_count;
      if (prog)
      {
        prog(processed_region_count, region_count);
      }
      last_region = iter.key().regionKey();
    }

    if (isOccupied(occupancy) || opt.export_free && isFree(occupancy))
    {
      pos = (mean.isLayerValid()) ? positionSafe(mean) : map.voxelCentreGlobal(*iter);
      ply.setPointPosition(pos);
      if (colour_select)
      {
        const ohm::Colour c = colour_select(occupancy);
        ply.setProperty("r", c.r());
        ply.setProperty("g", c.g());
        ply.setProperty("b", c.b());
      }

      ply.writePoint();
      ++point_count;
    }
  }

  ply.close();
  out.close();

  return point_count;
}


uint64_t saveHeightmapCloud(const char *file_name, const ohm::OccupancyMap &map, const SaveCloudOptions &opt,
                            const ProgressCallback &prog)
{
  std::ofstream out(file_name, std::ios::binary);

  if (!out.is_open())
  {
    return 0;
  }

  auto colour_select = opt.colour_select;
  std::unique_ptr<ColourByHeightmapClearance> colour_by_height;
  if (!colour_select && opt.allow_default_colour_selection)
  {
    colour_by_height = std::make_unique<ColourByHeightmapClearance>(map);
    colour_select = [&colour_by_height](const ohm::Voxel<const float> &occupancy) {
      return colour_by_height->select(occupancy);
    };
  }

  ohm::PlyPointStream ply = setupPlyStream(static_cast<bool>(colour_select));
  ply.open(out);

  glm::vec3 pos;
  const size_t region_count = map.regionCount();
  size_t processed_region_count = 0;
  glm::i16vec3 last_region = map.begin().key().regionKey();

  ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  auto mean = (opt.ignore_voxel_mean) ? ohm::Voxel<const ohm::VoxelMean>() :
                                        ohm::Voxel<const ohm::VoxelMean>(&map, map.layout().meanLayer());
  ohm::Voxel<const ohm::HeightmapVoxel> heightmap_voxel(&map,
                                                        map.layout().layerIndex(ohm::HeightmapVoxel::kHeightmapLayer));

  if (!heightmap_voxel.isLayerValid())
  {
    // Invalid format.
    return 0;
  }

  // Resolve the heightmap axis from the mapInfo if relevant.
  int heightmap_axis = 2;
  float height_flip = 1.0f;  // Set to -1 if we need to negate the height value to get a coordinate out of it.
  ohm::UpAxis up_axis = ohm::heightmap::queryHeightmapAxis(map.mapInfo());
  if (int(up_axis) >= 0)
  {
    heightmap_axis = int(up_axis);
  }
  else
  {
    heightmap_axis = 1 - int(up_axis);
    height_flip = -1.0f;
  }

  uint64_t point_count = 0;
  for (auto iter = map.begin(); iter != map.end(); ++iter)
  {
    ohm::setVoxelKey(iter, occupancy, mean, heightmap_voxel);
    if (last_region != iter.key().regionKey())
    {
      ++processed_region_count;
      if (prog)
      {
        prog(processed_region_count, region_count);
      }
      last_region = iter.key().regionKey();
    }
    if (isOccupied(occupancy) || opt.export_free && isFree(occupancy))
    {
      pos = (mean.isLayerValid()) ? positionSafe(mean) : map.voxelCentreGlobal(*iter);
      if (heightmap_voxel.isValid())
      {
        pos[heightmap_axis] += height_flip * heightmap_voxel.data().height;
      }

      ply.setPointPosition(pos);

      if (colour_select)
      {
        const ohm::Colour c = colour_select(occupancy);
        ply.setProperty("r", c.r());
        ply.setProperty("g", c.g());
        ply.setProperty("b", c.b());
      }

      ply.writePoint();
      ++point_count;
    }
  }

  ply.close();
  out.close();

  return point_count;
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
    uint8_t c = std::numeric_limits<uint8_t>::max();
    if (colour_range > 0 && ranges)
    {
      float range_value = ranges[i];
      if (range_value < 0)
      {
        range_value = colour_range;
      }
      c = uint8_t(std::numeric_limits<uint8_t>::max() * std::max(0.0f, (colour_range - range_value) / colour_range));
    }
    voxel_pos = map.voxelCentreGlobal(key);
    ply.addVertex(voxel_pos, ohm::Colour(c, std::numeric_limits<uint8_t>::max() / 2, 0));
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

  ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  ohm::Voxel<const float> clearance(&map, map.layout().clearanceLayer());

  if (!clearance.isLayerValid())
  {
    // No clearance layer.
    return 0;
  }

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
        assert(clearance.isValid());  // More for clang-tidy. We've already checked the layer validity.
        clearance.read(&range_value);
        if (range_value < 0)
        {
          range_value = colour_range;
        }
        if (range_value >= 0)
        {
          uint8_t c =
            uint8_t(std::numeric_limits<uint8_t>::max() * std::max(0.0f, (colour_scale - range_value) / colour_scale));
          v = map.voxelCentreLocal(*iter);
          ply.addVertex(v, ohm::Colour(c, std::numeric_limits<uint8_t>::max() / 2, 0));
          ++point_count;
        }
      }
    }
  }

  ply.save(file_name, true);

  return point_count;
}
}  // namespace ohmtools
