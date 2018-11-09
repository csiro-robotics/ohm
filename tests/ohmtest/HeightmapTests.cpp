// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/MapCache.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelLayout.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/Profile.h>

using namespace ohm;
using namespace ohmutil;

namespace
{
  void heightmapBoxTest(const std::string &prefix, Heightmap::Axis axis, int blur, std::shared_ptr<Heightmap> *map_out = nullptr)
  {
    Profile profile;
    const float boundary_distance = 2.5f;
    OccupancyMap map(0.2);

    // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
    ohmgen::boxRoom(map, glm::dvec3(-boundary_distance), glm::dvec3(boundary_distance));

    std::shared_ptr<Heightmap> heightmap(new Heightmap(0.2, 1.0, axis));
    if (map_out)
    {
      *map_out = heightmap;
    }
    heightmap->setOccupancyMap(&map);
    heightmap->setBlurLevel(blur);

    ProfileMarker mark_heightmap("heightmap", &profile);
    heightmap->update();
    mark_heightmap.end();

    // Verify output. Boundaries should be at ~ +boundary_distance (top of walls). All other voxels should be at
    // ~ -boundary_distance. Only approximage due to quantisation.

    const Key min_key = heightmap->heightmap().voxelKey(glm::dvec3(-boundary_distance));
    const Key max_key = heightmap->heightmap().voxelKey(glm::dvec3(boundary_distance));

    // Convert axis reference into an index [0, 2].
    const int axis_index = (axis >= 0) ? axis : -axis - 1;

    const double ground_height =
      (axis >= 0) ? map.voxelCentreGlobal(map.voxelKey(glm::dvec3(-boundary_distance)))[axis_index] :
                    -1.0 * map.voxelCentreGlobal(map.voxelKey(glm::dvec3(boundary_distance)))[axis_index];
    const double top_of_wall_height =
      (axis >= 0) ? map.voxelCentreGlobal(map.voxelKey(glm::dvec3(boundary_distance)))[axis_index] :
                    -1.0 * map.voxelCentreGlobal(map.voxelKey(glm::dvec3(-boundary_distance)))[axis_index];

    std::string filename;
    if (!prefix.empty())
    {
      filename = prefix + "-source.ohm";
      ohm::save(filename.c_str(), map);
      filename = prefix + ".ohm";
      ohm::save(filename.c_str(), heightmap->heightmap());

      filename = prefix + "-source.ply";
      ohmtools::saveCloud(filename.c_str(), map);
      // filename = prefix + "-2d.ply";
      // ohmtools::saveCloud(filename.c_str(), heightmap->heightmap());

      // Save the 2.5d heightmap.
      PlyMesh heightmapCloud;
      glm::dvec3 coord;
      for (auto iter = heightmap->heightmap().begin(); iter != heightmap->heightmap().end(); ++iter)
      {
        if (iter->isOccupied())
        {
          const HeightmapVoxel *voxel = iter->layerContent<const HeightmapVoxel *>(heightmap->heightmapVoxelLayer());
          // Get the coordinate of the voxel.
          coord = heightmap->heightmap().voxelCentreGlobal(iter.key());
          // Adjust the height to the plane height and offset.
          // For now assume a horizontal plane through the origin..
          coord[axis_index] = voxel->height;
          // Add to the cloud.
          heightmapCloud.addVertex(coord);
        }
      }

      filename = prefix + ".ply";
      heightmapCloud.save(filename.c_str(), true);
    }

    // Helper function to work out if we are at the top of the box or not.
    // Basically, at the limits of the box extents, we should report the top of the wall.
    // This also accounts for the blur level.
    const auto is_top_of_wall = [](const OccupancyMap &map, const Key &key, const Key &min_key, const Key &max_key,
                                   int *test_axes, int blur_level) {
      if (blur_level == 0)
      {
        if (key.axisMatches(test_axes[0], min_key) || key.axisMatches(test_axes[0], max_key) ||
            key.axisMatches(test_axes[1], min_key) || key.axisMatches(test_axes[1], max_key))
        {
          return true;
        }
      }
      else
      {
        // Have blur. Add edge tolerance.
        for (int i = 0; i <= blur_level; ++i)
        {
          Key test_key = key;
          map.moveKeyAlongAxis(test_key, test_axes[0], -i);
          if (test_key.axisMatches(test_axes[0], min_key))
          {
            return true;
          }

          test_key = key;
          map.moveKeyAlongAxis(test_key, test_axes[0], +i);
          if (test_key.axisMatches(test_axes[0], max_key))
          {
            return true;
          }

          test_key = key;
          map.moveKeyAlongAxis(test_key, test_axes[1], -i);
          if (test_key.axisMatches(test_axes[1], min_key))
          {
            return true;
          }

          test_key = key;
          map.moveKeyAlongAxis(test_key, test_axes[1], +i);
          if (test_key.axisMatches(test_axes[1], max_key))
          {
            return true;
          }
        }
      }

      return false;
    };

    MapCache cache;
    Key key = min_key;

    // Tricky stuff to resolve indexing the plane axes.
    int axis_indices[3];
    switch (heightmap->upAxisIndex())
    {
    case Heightmap::AxisX:
      axis_indices[0] = 1;
      axis_indices[1] = 2;
      axis_indices[2] = 0;
      break;
    case Heightmap::AxisY:
      axis_indices[0] = 0;
      axis_indices[1] = 2;
      axis_indices[2] = 1;
      break;
    default:
    case Heightmap::AxisZ:
      axis_indices[0] = 0;
      axis_indices[1] = 1;
      axis_indices[2] = 2;
      break;
    }

    // Walk the plane.
    key.setRegionAxis(axis_indices[2], 0);
    key.setLocalAxis(axis_indices[2], 0);
    for (; key.isBounded(axis_indices[1], min_key, max_key); heightmap->heightmap().stepKey(key, axis_indices[1], 1))
    {
      for (key.setAxisFrom(axis_indices[0], min_key); key.isBounded(axis_indices[0], min_key, max_key);
           heightmap->heightmap().stepKey(key, axis_indices[0], 1))
      {
        // Get the plane voxel and validate.
        VoxelConst voxel = heightmap->heightmap().voxel(key, false, &cache);

        double expected_height = ground_height;
        if (is_top_of_wall(heightmap->heightmap(), key, min_key, max_key, axis_indices, blur))
        {
          expected_height = top_of_wall_height;
        }

        ASSERT_TRUE(voxel.isValid());

        const HeightmapVoxel *voxel_content =
          voxel.layerContent<const HeightmapVoxel *>(heightmap->heightmapVoxelLayer());
        ASSERT_NE(voxel_content, nullptr);
        ASSERT_NEAR(voxel_content->height, expected_height, 1e-9);
      }
    }
  }
}  // namespace


TEST(Heightmap, Box)
{
  heightmapBoxTest("heightmap-box", Heightmap::AxisZ, 0);
}


TEST(Heightmap, Blur)
{
  // heightmapBoxTest("heightmap-blur-1", Heightmap::AxisZ, 1);
  heightmapBoxTest("heightmap-blur-2", Heightmap::AxisZ, 2);
}


TEST(Heightmap, AxisX)
{
  heightmapBoxTest("heightmap-x", Heightmap::AxisX, 0);
  heightmapBoxTest("heightmap-x", Heightmap::AxisX, 1);
}


TEST(Heightmap, AxisY)
{
  heightmapBoxTest("heightmap-y", Heightmap::AxisY, 0);
  heightmapBoxTest("heightmap-y", Heightmap::AxisY, 1);
}


TEST(Heightmap, AxisZ)
{
  heightmapBoxTest("heightmap-z", Heightmap::AxisNegZ, 0);
  heightmapBoxTest("heightmap-z", Heightmap::AxisNegZ, 1);
}


TEST(Heightmap, AxisNegX)
{
  heightmapBoxTest("heightmap-negx", Heightmap::AxisX, 0);
  heightmapBoxTest("heightmap-negx", Heightmap::AxisX, 1);
}


TEST(Heightmap, AxisNegY)
{
  heightmapBoxTest("heightmap-negy", Heightmap::AxisY, 0);
  heightmapBoxTest("heightmap-negy", Heightmap::AxisY, 1);
}


TEST(Heightmap, AxisNegZ)
{
  heightmapBoxTest("heightmap-negz", Heightmap::AxisNegZ, 0);
  heightmapBoxTest("heightmap-negz", Heightmap::AxisNegZ, 1);
}


TEST(Heightmap, Layout)
{
  std::shared_ptr<Heightmap> heightmap;
  heightmapBoxTest("", Heightmap::AxisZ, 0, &heightmap);
  const MapLayout &layout = heightmap->heightmap().layout();

  EXPECT_EQ(layout.layerCount(), 2);
  const MapLayer *occupancy_layer = layout.layer(defaultLayerName(kDlOccupancy));
  const MapLayer *heightmap_layer = layout.layer(HeightmapVoxel::kHeightmapLayer);
  ASSERT_NE(occupancy_layer, nullptr);
  ASSERT_NE(heightmap_layer, nullptr);

  EXPECT_EQ(occupancy_layer->layerIndex(), 0);
  EXPECT_EQ(heightmap_layer->layerIndex(), 1);

  EXPECT_EQ(occupancy_layer->voxelByteSize(), sizeof(float));
  EXPECT_EQ(heightmap_layer->voxelByteSize(), sizeof(HeightmapVoxel));

  VoxelLayoutConst occupancy_voxel = occupancy_layer->voxelLayout();
  VoxelLayoutConst heightmap_voxel = heightmap_layer->voxelLayout();

  EXPECT_EQ(occupancy_voxel.memberCount(), 1);
  EXPECT_STREQ(occupancy_voxel.memberName(0), defaultLayerName(kDlOccupancy));
  EXPECT_EQ(occupancy_voxel.memberOffset(0), 0);
  EXPECT_EQ(occupancy_voxel.memberSize(0), sizeof(float));

  EXPECT_EQ(heightmap_voxel.memberCount(), 2);
  EXPECT_STREQ(heightmap_voxel.memberName(0), "height");
  EXPECT_STREQ(heightmap_voxel.memberName(1), "clearance");
  EXPECT_EQ(heightmap_voxel.memberOffset(0), 0);
  EXPECT_EQ(heightmap_voxel.memberSize(0), sizeof(float));
  EXPECT_EQ(heightmap_voxel.memberOffset(1), sizeof(float));
  EXPECT_EQ(heightmap_voxel.memberSize(1), sizeof(float));
}
