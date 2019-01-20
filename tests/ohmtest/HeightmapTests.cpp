// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Heightmap.h>
#include <ohm/HeightmapMesh.h>
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
  struct TestParams
  {
    double floor = 0;
    double ceiling = 0;
  };

  const double kBoxHalfExtents = 2.5;

  void heightmapBoxTest(const std::string &prefix, Heightmap::Axis axis, int blur,
                        std::shared_ptr<Heightmap> *map_out = nullptr, const TestParams *params = nullptr)
  {
    Profile profile;
    const float boundary_distance = float(kBoxHalfExtents);
    const double base_height = 0;
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

    if (params)
    {
      heightmap->setFloor(params->floor);
      heightmap->setCeiling(params->ceiling);
    }

    ProfileMarker mark_heightmap("heightmap", &profile);
    heightmap->update(base_height);
    mark_heightmap.end();

    // Verify output. Boundaries should be at ~ +boundary_distance (top of walls). All other voxels should be at
    // ~ -boundary_distance. Only approximage due to quantisation.

    const Key min_key = heightmap->heightmap().voxelKey(glm::dvec3(-boundary_distance));
    const Key max_key = heightmap->heightmap().voxelKey(glm::dvec3(boundary_distance));

    // Convert axis reference into an index [0, 2].
    const int axis_index = (axis >= 0) ? axis : -axis - 1;

    double ground_height = (axis >= 0) ?
                             map.voxelCentreGlobal(map.voxelKey(glm::dvec3(-boundary_distance)))[axis_index] :
                             -1.0 * map.voxelCentreGlobal(map.voxelKey(glm::dvec3(boundary_distance)))[axis_index];
    double top_of_wall_height =
      (axis >= 0) ? map.voxelCentreGlobal(map.voxelKey(glm::dvec3(boundary_distance)))[axis_index] :
                    -1.0 * map.voxelCentreGlobal(map.voxelKey(glm::dvec3(-boundary_distance)))[axis_index];

    if (params)
    {
      double adjusted_ceiling = top_of_wall_height;
      double adjusted_ground = ground_height;
      if (params->ceiling > 0)
      {
        adjusted_ceiling = std::min(adjusted_ceiling, base_height + params->ceiling);
      }
      if (params->floor > 0)
      {
        adjusted_ground = std::max(adjusted_ground, base_height - params->floor);
        if (adjusted_ground > ground_height)
        {
          adjusted_ground = top_of_wall_height;

          if (adjusted_ground > adjusted_ceiling)
          {
            // Ground is clipped at both ends. Remove completely.
            adjusted_ground = std::numeric_limits<double>::infinity();
          }
        }
      }

      ground_height = adjusted_ground;
      top_of_wall_height = adjusted_ceiling;
    }

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
          coord += double(voxel->height) * Heightmap::upAxisNormal(axis_index);
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
        bool wall = false;
        if (is_top_of_wall(heightmap->heightmap(), key, min_key, max_key, axis_indices, blur))
        {
          expected_height = top_of_wall_height;
          wall = true;
        }

        ASSERT_TRUE(voxel.isValid());

        const HeightmapVoxel *voxel_content =
          voxel.layerContent<const HeightmapVoxel *>(heightmap->heightmapVoxelLayer());
        ASSERT_NE(voxel_content, nullptr) << (wall ? "top" : "floor");
        // Need the equality to handle when both values are inf.
        if (voxel_content->height + base_height != expected_height)
        {
          ASSERT_NEAR(voxel_content->height + base_height, expected_height, 1e-9) << (wall ? "top" : "floor");
        }
      }
    }
  }
}  // namespace


TEST(Heightmap, Box)
{
  heightmapBoxTest("heightmap-box", Heightmap::AxisZ, 0);
}


TEST(Heightmap, Ceiling)
{
  TestParams params;
  params.ceiling = 0.5;
  heightmapBoxTest("heightmap-ceiling", Heightmap::AxisZ, 0, nullptr, &params);
}


TEST(Heightmap, Floor)
{
  TestParams params;
  params.floor = 0.5;
  heightmapBoxTest("heightmap-floor", Heightmap::AxisZ, 0, nullptr, &params);
}


TEST(Heightmap, FloorAndCeiling)
{
  TestParams params;
  params.floor = 0.3;
  params.ceiling = 0.5;
  heightmapBoxTest("heightmap-ceiling-floor", Heightmap::AxisZ, 0, nullptr, &params);
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

  EXPECT_EQ(layout.layerCount(), 3);
  const MapLayer *occupancy_layer = layout.layer(default_layer::occupancyLayerName());
  const MapLayer *heightmap_layer = layout.layer(HeightmapVoxel::kHeightmapLayer);
  const MapLayer *heightmap_build_layer = layout.layer(HeightmapVoxel::kHeightmapBuildLayer);
  ASSERT_NE(occupancy_layer, nullptr);
  ASSERT_NE(heightmap_layer, nullptr);
  ASSERT_NE(heightmap_build_layer, nullptr);

  EXPECT_EQ(occupancy_layer->layerIndex(), 0);
  EXPECT_EQ(heightmap_layer->layerIndex(), 1);
  EXPECT_EQ(heightmap_build_layer->layerIndex(), 2);

  EXPECT_EQ(occupancy_layer->voxelByteSize(), sizeof(float));
  EXPECT_EQ(heightmap_layer->voxelByteSize(), sizeof(HeightmapVoxel));
  EXPECT_EQ(heightmap_build_layer->voxelByteSize(), sizeof(HeightmapVoxel));

  VoxelLayoutConst occupancy_voxel = occupancy_layer->voxelLayout();
  VoxelLayoutConst heightmap_voxel = heightmap_layer->voxelLayout();
  VoxelLayoutConst heightmap_build_voxel = heightmap_layer->voxelLayout();

  EXPECT_EQ(occupancy_voxel.memberCount(), 1);
  EXPECT_STREQ(occupancy_voxel.memberName(0), default_layer::occupancyLayerName());
  EXPECT_EQ(occupancy_voxel.memberOffset(0), 0);
  EXPECT_EQ(occupancy_voxel.memberSize(0), sizeof(float));

  const auto validate_heightmap_voxel_layout = [](const VoxelLayoutConst &layout) {
    EXPECT_EQ(layout.memberCount(), 2);
    EXPECT_STREQ(layout.memberName(0), "height");
    EXPECT_STREQ(layout.memberName(1), "clearance");
    EXPECT_EQ(layout.memberOffset(0), 0);
    EXPECT_EQ(layout.memberSize(0), sizeof(float));
    EXPECT_EQ(layout.memberOffset(1), sizeof(float));
    EXPECT_EQ(layout.memberSize(1), sizeof(float));
  };

  validate_heightmap_voxel_layout(heightmap_voxel);
  validate_heightmap_voxel_layout(heightmap_build_voxel);
}


TEST(Heightmap, Mesh)
{
  std::shared_ptr<Heightmap> heightmap;
  heightmapBoxTest("", Heightmap::AxisZ, 0, &heightmap);
  HeightmapMesh mesh;

  bool ok = mesh.buildMesh(*heightmap);
  ASSERT_TRUE(ok);

  PlyMesh ply;
  mesh.extractPlyMesh(ply);
  ply.save("hmm.ply", true);

  // Since we have generated a heightmap from a box, we can assume and verify the following characteristics:
  // - Verify all neighbour information.
  // - By inspecting triangle neighbours, we can validate triangle normals.

  const size_t vertex_count = mesh.vertexCount();
  const size_t triangle_count = mesh.triangleCount();
  const glm::dvec3 up = heightmap->upAxisNormal();

  const double height_low = -kBoxHalfExtents;
  const double height_high = kBoxHalfExtents;
  for (size_t t = 0; t < triangle_count; ++t)
  {
    const TriangleNeighbours &neighbours = mesh.triangleNeighbours()[t];

    // Check the triangle's neighbours.
    // Get the height of the vertices. All should be either height_low or height_height.
    for (int i = 3; i < 3; ++i)
    {
      glm::dvec3 vertex = mesh.vertices()[mesh.triangles()[t * 3 + i]];
      double vertex_height = glm::dot(up, vertex);
      if (vertex_height < 0)
      {
        EXPECT_NEAR(vertex_height, height_low, 1e-4);
      }
      else
      {
        EXPECT_NEAR(vertex_height, height_high, 1e-4);
      }
    }

    // Confirm the neighbour indices match.
    for (int i = 0; i < 3; ++i)
    {
      const unsigned nt = neighbours.neighbours[i];

      if (nt == ~0u)
      {
        // No neighbour.
        continue;
      }

      int e0, e1;
      int ne0, ne1;
      unsigned v0, v1;
      unsigned nv0, nv1;

      e0 = i;
      e1 = (e0 + 1) % 3;

      // Get the triangle edge vertex indices.
      v0 = mesh.triangles()[t * 3 + e0];
      v1 = mesh.triangles()[t * 3 + e1];

      ASSERT_LT(v0, vertex_count);
      ASSERT_LT(v1, vertex_count);

      // Define edge indices in the neighbour.
      ne0 = neighbours.neighbour_edge_indices[i];
      ASSERT_GE(ne0, 0);
      ASSERT_LT(ne0, 3);

      ne1 = (ne0 + 1) % 3;

      // Get the neighbour edge vertex indices.
      nv0 = mesh.triangles()[nt * 3 + ne0];
      nv1 = mesh.triangles()[nt * 3 + ne1];

      ASSERT_LT(ne0, vertex_count);
      ASSERT_LT(ne1, vertex_count);

      // Validate the edge indices match.
      // Index order may be reversed due to winding, so make both have lower value index come first.
      if (v0 > v1)
      {
        std::swap(v0, v1);
      }
      if (nv0 > nv1)
      {
        std::swap(nv0, nv1);
      }

      EXPECT_EQ(v0, nv0) << "vertex mismatch between triangles: " << t << "," << nt;
      EXPECT_EQ(v1, nv1) << "vertex mismatch between triangles: " << t << "," << nt;
    }
  }
}
