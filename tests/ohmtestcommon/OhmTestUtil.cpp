// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestUtil.h"

#include <ohm/MapChunk.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelData.h>

#include <gtest/gtest.h>

#include <stdio.h> /* defines FILENAME_MAX */
#ifdef WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

using namespace ohm;

namespace ohmtestutil
{
std::string app_dir;

const char *applicationDir()
{
  return (!app_dir.empty()) ? app_dir.c_str() : "";
}

void setApplicationPath(const char *path)
{
  app_dir = path;
  // Strip the executable name
  auto lastSeparator = app_dir.find_last_of("/\\");
  if (lastSeparator != std::string::npos)
  {
    app_dir = app_dir.substr(0, lastSeparator + 1);
  }
  else
  {
    // Current dir.
    app_dir.clear();
  }
}

bool compareLayout(const OccupancyMap &map, const OccupancyMap &reference_map)
{
  const MapLayout &layout = map.layout();
  const MapLayout &ref_layout = reference_map.layout();
  bool layout_matches = layout.layerCount() == ref_layout.layerCount();
  EXPECT_EQ(layout.layerCount(), ref_layout.layerCount());

  for (unsigned i = 0; layout_matches && i < layout.layerCount(); ++i)
  {
    const MapLayer &layer = layout.layer(i);
    const MapLayer &ref_layer = ref_layout.layer(i);

    layout_matches = layout_matches && strcmp(layer.name(), ref_layer.name()) == 0;
    EXPECT_STREQ(layer.name(), ref_layer.name());
    layout_matches = layout_matches && layer.layerIndex() == ref_layer.layerIndex();
    EXPECT_EQ(layer.layerIndex(), ref_layer.layerIndex());
    layout_matches = layout_matches && layer.subsampling() == ref_layer.subsampling();
    EXPECT_EQ(layer.subsampling(), ref_layer.subsampling());
    layout_matches = layout_matches && layer.flags() == ref_layer.flags();
    EXPECT_EQ(layer.flags(), ref_layer.flags());
    layout_matches = layout_matches && layer.dimensions(map.regionVoxelDimensions()) ==
                                         ref_layer.dimensions(reference_map.regionVoxelDimensions());
    EXPECT_EQ(layer.dimensions(map.regionVoxelDimensions()),
              ref_layer.dimensions(reference_map.regionVoxelDimensions()));
    layout_matches = layout_matches && layer.layerByteSize(map.regionVoxelDimensions()) ==
                                         ref_layer.layerByteSize(reference_map.regionVoxelDimensions());
    EXPECT_EQ(layer.layerByteSize(map.regionVoxelDimensions()),
              ref_layer.layerByteSize(reference_map.regionVoxelDimensions()));

    VoxelLayoutConst voxel_layout = layer.voxelLayout();
    VoxelLayoutConst ref_voxel_layout = ref_layer.voxelLayout();

    layout_matches = layout_matches && voxel_layout.memberCount() == ref_voxel_layout.memberCount();
    EXPECT_EQ(voxel_layout.memberCount(), ref_voxel_layout.memberCount());
    layout_matches = layout_matches && voxel_layout.voxelByteSize() == ref_voxel_layout.voxelByteSize();
    EXPECT_EQ(voxel_layout.voxelByteSize(), ref_voxel_layout.voxelByteSize());

    for (unsigned v = 0; layout_matches && v < voxel_layout.memberCount(); ++v)
    {
      layout_matches = layout_matches && strcmp(voxel_layout.memberName(v), ref_voxel_layout.memberName(v)) == 0;
      EXPECT_STREQ(voxel_layout.memberName(v), ref_voxel_layout.memberName(v));
      layout_matches = layout_matches && voxel_layout.memberType(v) == ref_voxel_layout.memberType(v);
      EXPECT_EQ(voxel_layout.memberType(v), ref_voxel_layout.memberType(v));
      layout_matches = layout_matches && voxel_layout.memberClearValue(v) == ref_voxel_layout.memberClearValue(v);
      EXPECT_EQ(voxel_layout.memberClearValue(v), ref_voxel_layout.memberClearValue(v));
    }
  }

  return layout_matches;
}


void compareMaps(const OccupancyMap &map, const OccupancyMap &reference_map, unsigned compare_flags)
{
  compareMaps(map, reference_map, glm::dvec3(-std::numeric_limits<double>::infinity()),
              glm::dvec3(std::numeric_limits<double>::infinity()), compare_flags);
}


void compareMaps(const OccupancyMap &map, const OccupancyMap &reference_map, const glm::dvec3 &min_ext,
                 const glm::dvec3 &max_ext, unsigned compare_flags)
{
  const bool full_extents = glm::all(glm::equal(min_ext, glm::dvec3(-std::numeric_limits<double>::infinity()))) &&
                            glm::all(glm::equal(max_ext, glm::dvec3(std::numeric_limits<double>::infinity())));

  if (compare_flags & kCfGeneral)
  {
    EXPECT_EQ(map.resolution(), reference_map.resolution());
    EXPECT_EQ(map.regionSpatialResolution(), reference_map.regionSpatialResolution());
    EXPECT_EQ(map.regionVoxelDimensions(), reference_map.regionVoxelDimensions());
    EXPECT_EQ(map.regionVoxelVolume(), reference_map.regionVoxelVolume());
    EXPECT_EQ(map.origin(), reference_map.origin());
    if (full_extents)
    {
      EXPECT_EQ(map.regionCount(), reference_map.regionCount());
    }
    EXPECT_EQ(map.hitValue(), reference_map.hitValue());
    EXPECT_EQ(map.missValue(), reference_map.missValue());
    EXPECT_EQ(map.occupancyThresholdValue(), reference_map.occupancyThresholdValue());
  }

  // Layout
  // Start with layout not compared. Important so later we can assume layout is OK.
  bool layout_matches = false;
  if (compare_flags & kCfLayout)
  {
    layout_matches = compareLayout(map, reference_map);
    EXPECT_TRUE(layout_matches);
  }

  // Chunks
  if (compare_flags & kCfChunksGeneral)
  {
    std::vector<const MapChunk *> ref_chunks;
    reference_map.enumerateRegions(ref_chunks);

    for (const MapChunk *ref_chunk : ref_chunks)
    {
      const MapChunk *chunk = map.region(ref_chunk->region.coord);

      if (full_extents || ref_chunk->overlapsExtents(min_ext, max_ext))
      {
        EXPECT_TRUE(chunk != nullptr);
      }

      if (!chunk)
      {
        continue;
      }

      EXPECT_EQ(chunk->region.centre, ref_chunk->region.centre);
      EXPECT_EQ(chunk->region.coord, ref_chunk->region.coord);

      if (compare_flags & kCfChunksFine)
      {
        EXPECT_EQ(chunk->first_valid_index, ref_chunk->first_valid_index);
        EXPECT_EQ(chunk->touched_time, ref_chunk->touched_time);
        EXPECT_EQ(chunk->dirty_stamp, ref_chunk->dirty_stamp);
        for (unsigned i = 0; i < chunk->layout().layerCount(); ++i)
        {
          EXPECT_EQ(chunk->touched_stamps[i], ref_chunk->touched_stamps[i]);
        }
        EXPECT_EQ(chunk->flags, ref_chunk->flags);
      }
    }
  }

  // Voxels
  if (compare_flags & (kCfOccupancy | kCfClearance))
  {
    bool have_valid_clerance = false;
    Voxel<const float> map_occupancy(&map, map.layout().occupancyLayer());
    Voxel<const float> map_clearance(&map, map.layout().clearanceLayer());
    Voxel<const float> ref_occupancy(&reference_map, reference_map.layout().occupancyLayer());
    Voxel<const float> ref_clearance(&reference_map, reference_map.layout().clearanceLayer());

    ASSERT_EQ(map_occupancy.isLayerValid(), ref_occupancy.isLayerValid());
    if (compare_flags & kCfClearance)
    {
      ASSERT_EQ(map_clearance.isLayerValid(), ref_clearance.isLayerValid());
    }

    for (auto iter = map.begin(); iter != map.end(); ++iter)
    {
      map_clearance.setKey(map_occupancy.setKey(iter));
      ref_clearance.setKey(ref_occupancy.setKey(iter.key()));

      ASSERT_TRUE(map_occupancy.isValid());

      if (full_extents || map_occupancy.chunk()->overlapsExtents(min_ext, max_ext))
      {
        ASSERT_TRUE(ref_occupancy.isValid());
      }

      if (!ref_occupancy.isValid())
      {
        continue;
      }

      ASSERT_EQ(ref_clearance.key(), map_occupancy.key());
      if (compare_flags & kCfOccupancy)
      {
        float ref_value, map_value;
        ASSERT_EQ(ref_occupancy.read(&ref_value), map_occupancy.read(&map_value));
      }

      if (compare_flags & kCfClearance)
      {
        if (map_clearance.isValid())
        {
          ASSERT_TRUE(ref_clearance.isValid());
          float ref_clearance_value, map_clearance_value;
          ref_clearance.read(&ref_clearance_value);
          map_clearance.read(&map_clearance_value);
          ASSERT_EQ(ref_clearance_value, map_clearance_value);
          have_valid_clerance = have_valid_clerance || ref_clearance_value >= 0;
        }
      }
    }

    if (compare_flags & kCfExpectClearance)
    {
      EXPECT_TRUE(have_valid_clerance);
    }
  }
}
}  // namespace ohmtestutil
