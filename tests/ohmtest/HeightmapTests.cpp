// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/Heightmap.h>
#include <ohm/HeightmapMesh.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/TriangleNeighbours.h>
#include <ohm/VoxelData.h>
#include <ohm/VoxelLayout.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/Profile.h>

#include <sstream>

using namespace ohm;

namespace
{
const double kBoxHalfExtents = 2.5;

void heightmapBoxTest(const std::string &prefix, UpAxis axis, std::shared_ptr<Heightmap> *map_out = nullptr)
{
  Profile profile;
  const float boundary_distance = float(kBoxHalfExtents);
  OccupancyMap map(0.2);

  // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
  ohmgen::boxRoom(map, glm::dvec3(-boundary_distance), glm::dvec3(boundary_distance));

  std::shared_ptr<Heightmap> heightmap(new Heightmap(0.2, 1.0, axis));
  if (map_out)
  {
    *map_out = heightmap;
  }
  heightmap->setOccupancyMap(&map);

  ProfileMarker mark_heightmap("heightmap", &profile);
  const double base_height = 0.0;
  heightmap->buildHeightmap(base_height * heightmap->upAxisNormal());
  mark_heightmap.end();

  // Verify output. Boundaries should be at ~ +boundary_distance (top of walls). All other voxels should be at
  // ~ -boundary_distance. Only approximage due to quantisation.

  const Key min_key = heightmap->heightmap().voxelKey(glm::dvec3(-boundary_distance));
  const Key max_key = heightmap->heightmap().voxelKey(glm::dvec3(boundary_distance));

  // Convert axis reference into an index [0, 2].
  const int axis_index = (int(axis) >= 0) ? int(axis) : -int(axis) - 1;

  double ground_height = (int(axis) >= 0) ?
                           map.voxelCentreGlobal(map.voxelKey(glm::dvec3(-boundary_distance)))[axis_index] :
                           -1.0 * map.voxelCentreGlobal(map.voxelKey(glm::dvec3(boundary_distance)))[axis_index];
  double top_of_wall_height = (int(axis) >= 0) ?
                                map.voxelCentreGlobal(map.voxelKey(glm::dvec3(boundary_distance)))[axis_index] :
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
    Voxel<const float> occupancy(&heightmap->heightmap(), heightmap->heightmap().layout().occupancyLayer());
    Voxel<const HeightmapVoxel> heigthmap_voxel(&heightmap->heightmap(), heightmap->heightmapVoxelLayer());
    for (auto iter = heightmap->heightmap().begin(); iter != heightmap->heightmap().end(); ++iter)
    {
      occupancy.setKey(*iter);
      if (isOccupied(occupancy))
      {
        heigthmap_voxel.setKey(occupancy);
        HeightmapVoxel voxel;
        heigthmap_voxel.read(&voxel);
        // Get the coordinate of the voxel.
        coord = heightmap->heightmap().voxelCentreGlobal(*iter);
        coord += double(voxel.height) * Heightmap::upAxisNormal(axis);
        // Add to the cloud.
        heightmapCloud.addVertex(coord);
      }
    }

    filename = prefix + ".ply";
    heightmapCloud.save(filename.c_str(), true);
  }

  // Helper function to work out if we are at the top of the box or not.
  // Basically, at the limits of the box extents, we should report the top of the wall.
  const auto is_top_of_wall = [](const Key &key, const Key &min_key, const Key &max_key, int *test_axes) {
    if (key.axisMatches(test_axes[0], min_key) || key.axisMatches(test_axes[0], max_key) ||
        key.axisMatches(test_axes[1], min_key) || key.axisMatches(test_axes[1], max_key))
    {
      return true;
    }

    return false;
  };

  Key key = min_key;

  // Tricky stuff to resolve indexing the plane axes.
  int axis_indices[3];
  switch (UpAxis(heightmap->upAxisIndex()))
  {
  case UpAxis::kX:
    axis_indices[0] = 1;
    axis_indices[1] = 2;
    axis_indices[2] = 0;
    break;
  case UpAxis::kY:
    axis_indices[0] = 0;
    axis_indices[1] = 2;
    axis_indices[2] = 1;
    break;
  default:
  case UpAxis::kZ:
    axis_indices[0] = 0;
    axis_indices[1] = 1;
    axis_indices[2] = 2;
    break;
  }

  // Walk the plane.
  key.setRegionAxis(axis_indices[2], 0);
  key.setLocalAxis(axis_indices[2], 0);
  Voxel<const HeightmapVoxel> voxel(&heightmap->heightmap(), heightmap->heightmapVoxelLayer());
  for (; key.isBounded(axis_indices[1], min_key, max_key); heightmap->heightmap().stepKey(key, axis_indices[1], 1))
  {
    for (key.setAxisFrom(axis_indices[0], min_key); key.isBounded(axis_indices[0], min_key, max_key);
         heightmap->heightmap().stepKey(key, axis_indices[0], 1))
    {
      // Get the plane voxel and validate.
      voxel.setKey(key);

      double expected_height = ground_height;
      bool wall = false;
      if (is_top_of_wall(key, min_key, max_key, axis_indices))
      {
        expected_height = top_of_wall_height;
        wall = true;
      }

      if (int(axis) < 0)
      {
        expected_height *= -1.0;
      }

      ASSERT_TRUE(voxel.isValid()) << (wall ? "top" : "floor");
      ;

      HeightmapVoxel voxel_content;
      voxel.read(&voxel_content);
      // Need the equality to handle when both values are inf.
      if (voxel_content.height + base_height != expected_height)
      {
        ASSERT_NEAR(voxel_content.height + base_height, expected_height, 1e-9) << (wall ? "top" : "floor");
      }
    }
  }
}
}  // namespace


TEST(Heightmap, Box)
{
  heightmapBoxTest("heightmap-box", UpAxis::kZ);
}


TEST(Heightmap, AxisX)
{
  heightmapBoxTest("heightmap-x", UpAxis::kX);
}


TEST(Heightmap, AxisY)
{
  heightmapBoxTest("heightmap-y", UpAxis::kY);
}


TEST(Heightmap, AxisZ)
{
  heightmapBoxTest("heightmap-z", UpAxis::kZ);
}


TEST(Heightmap, AxisNegX)
{
  heightmapBoxTest("heightmap-negx", UpAxis::kNegX);
}


TEST(Heightmap, AxisNegY)
{
  heightmapBoxTest("heightmap-negy", UpAxis::kNegY);
}


TEST(Heightmap, AxisNegZ)
{
  heightmapBoxTest("heightmap-negz", UpAxis::kNegZ);
}


TEST(Heightmap, Layout)
{
  std::shared_ptr<Heightmap> heightmap;
  heightmapBoxTest("", UpAxis::kZ, &heightmap);
  const MapLayout &layout = heightmap->heightmap().layout();

  EXPECT_EQ(layout.layerCount(), 2);
  const MapLayer *occupancy_layer = layout.layer(default_layer::occupancyLayerName());
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
  EXPECT_STREQ(occupancy_voxel.memberName(0), default_layer::occupancyLayerName());
  EXPECT_EQ(occupancy_voxel.memberOffset(0), 0);
  EXPECT_EQ(occupancy_voxel.memberSize(0), sizeof(float));

  const auto validate_heightmap_voxel_layout = [](const VoxelLayoutConst &layout) {
    EXPECT_EQ(layout.memberCount(), 6);
    EXPECT_STREQ(layout.memberName(0), "height");
    EXPECT_STREQ(layout.memberName(1), "clearance");
    EXPECT_STREQ(layout.memberName(2), "normal_x");
    EXPECT_STREQ(layout.memberName(3), "normal_y");
    EXPECT_STREQ(layout.memberName(4), "normal_z");
    EXPECT_STREQ(layout.memberName(5), "reserved");
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_EQ(layout.memberOffset(i), i * sizeof(float));
      EXPECT_EQ(layout.memberSize(i), sizeof(float));
    }
  };

  validate_heightmap_voxel_layout(heightmap_voxel);
}


TEST(Heightmap, Mesh)
{
  std::shared_ptr<Heightmap> heightmap;
  heightmapBoxTest("", UpAxis::kZ, &heightmap);
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


namespace
{
enum class SurfaceType
{
  kVoid,
  kVirtual,
  kReal
};

const char *kSurfaceName[] = { "void", "virtual", "real" };

enum class HeightmapSelect
{
  kNull,  // Should only be used to indicate the expected result is not to choise a voxel.
  kBelow,
  kAbove
};

enum class SurfaceMode
{
  kNone,
  kVirtual,
  kPromoteVirtual
};

const char *kModeName[] = { "no-virtual", "virtual", "promoted-virtual" };

struct HeightmapSelectTest
{
  SurfaceType type_below;
  SurfaceType type_above;
  HeightmapSelect closer;

  void fillContext(std::ostream &o) const
  {
    const char *mark_below = (closer == HeightmapSelect::kBelow) ? "*" : " ";
    const char *mark_above = (closer != HeightmapSelect::kBelow) ? "*" : " ";
    o << " below" << mark_below << "<" << kSurfaceName[(int)type_below] << ">";
    o << " above" << mark_above << "<" << kSurfaceName[(int)type_above] << ">";
  }
};

struct HeightmapTestResult
{
  HeightmapVoxelType surface;
  HeightmapSelect select;
};

void addVirtualHeightmapVoxel(ohm::OccupancyMap *map, double range)
{
  const glm::dvec3 pos = glm::dvec3(0, 0, range);
  ohm::integrateMiss(*map, map->voxelKey(pos));
}

void addRealHeightmapVoxel(ohm::OccupancyMap *map, double range)
{
  const glm::dvec3 pos = glm::dvec3(0, 0, range);
  ohm::integrateHit(*map, pos);
}

void testSurface(SurfaceMode surface_mode, const HeightmapSelectTest &test_data, const HeightmapTestResult &expected,
                 const std::string &context)
{
  // Build a constrained region map. Ensures we have to step over void regions.
  ohm::OccupancyMap map(1.0, glm::u8vec3(8, 8, 2));
  // Offset the map so that 0, 0, 0 is at the centre of a voxel.
  map.setOrigin(glm::dvec3(-0.5 * map.resolution()));
  // Set ranges based on map resolution
  const double selected_voxel_range = 5.0 * map.resolution();
  const double other_voxel_range = selected_voxel_range + 2.0 * map.resolution();

  // Set ranges for above/below based on select
  const double range_below = (test_data.closer == HeightmapSelect::kBelow) ? selected_voxel_range : other_voxel_range;
  const double range_above = (test_data.closer != HeightmapSelect::kBelow) ? selected_voxel_range : other_voxel_range;
  const double expected_height = (expected.select == HeightmapSelect::kBelow) ? -range_below : range_above;

  // Add the voxels
  // Add below
  if (test_data.type_below == SurfaceType::kVirtual)
  {
    addVirtualHeightmapVoxel(&map, -range_below);
  }
  else if (test_data.type_below == SurfaceType::kReal)
  {
    addRealHeightmapVoxel(&map, -range_below);
  }
  // Add above.
  if (test_data.type_above == SurfaceType::kVirtual)
  {
    addVirtualHeightmapVoxel(&map, range_above);
  }
  else if (test_data.type_above == SurfaceType::kReal)
  {
    addRealHeightmapVoxel(&map, range_above);
  }

  // Now generate the heightmap.
  ohm::Heightmap heightmap(map.resolution(), 0.0);  // Ignore clearance
  heightmap.setOccupancyMap(&map);
  // Match map origins.
  heightmap.heightmap().setOrigin(map.origin());
  // Setup virtul surface support.
  heightmap.setGenerateVirtualSurface(surface_mode != SurfaceMode::kNone);
  heightmap.setPromoteVirtualBelow(surface_mode == SurfaceMode::kPromoteVirtual);
  // Create a AABB which limits the search space to right near the origin column.
  ohm::Aabb clip;
  clip.setMaxExtents(glm::dvec3(0.5 * map.resolution(), 0.5 * map.resolution(), 2 * other_voxel_range));
  clip.setMinExtents(-clip.maxExtents());
  // Build
  heightmap.buildHeightmap(glm::dvec3(0), clip);

  // Validate the selected voxel.
  ohm::Key hm_key = heightmap.heightmap().voxelKey(glm::dvec3(0));
  // ensure zero layer.
  hm_key.setRegionAxis(2, 0);
  hm_key.setLocalAxis(2, 0);

  // Validate the height of the voxel.
  glm::dvec3 pos{};
  ohm::HeightmapVoxel info{};
  HeightmapVoxelType voxel_type = heightmap.getHeightmapVoxelInfo(hm_key, &pos, &info);
  EXPECT_EQ((int)voxel_type, (int)expected.surface) << context;
  if (expected.surface != HeightmapVoxelType::kVacant && expected.surface != HeightmapVoxelType::kUnknown)
  {
    EXPECT_EQ(pos.z, expected_height) << context;
  }
}
}  // namespace

TEST(Heightmap, SurfaceSelection)
{
  // Test virtual surface generation.
  // We have the following voxel combinations to test:
  // - Real below, real above, below closer
  // - Real below, real above, above closer
  // - Real below, virtual above, below closer
  // - Real below, virtual above, above closer
  // - Virtual below, real above, below closer
  // - Virtual below, real above, above closer
  // - Virtual below, virtual above, below closer
  // - Virtual below, virtual above, above closer
  // - Real below, void above
  // - Void below, real above
  // - Virtual below, void above
  // - Void below, virtual above
  // - Void below, void above
  //
  // We run all test configurations in three surface modes: i. no virtual surfaces, ii.allow virtual surfaces preferring
  // real, iii. promote virtual surfaces below over real above.

  const HeightmapSelectTest surface_tests[] =  //
    {                                          // No virtual surface
      HeightmapSelectTest{ SurfaceType::kReal, SurfaceType::kReal, HeightmapSelect::kBelow },
      HeightmapSelectTest{ SurfaceType::kReal, SurfaceType::kReal, HeightmapSelect::kAbove },
      HeightmapSelectTest{ SurfaceType::kReal, SurfaceType::kVirtual, HeightmapSelect::kBelow },
      HeightmapSelectTest{ SurfaceType::kReal, SurfaceType::kVirtual, HeightmapSelect::kAbove },
      HeightmapSelectTest{ SurfaceType::kVirtual, SurfaceType::kReal, HeightmapSelect::kBelow },
      HeightmapSelectTest{ SurfaceType::kVirtual, SurfaceType::kReal, HeightmapSelect::kAbove },
      HeightmapSelectTest{ SurfaceType::kVirtual, SurfaceType::kVirtual, HeightmapSelect::kBelow },
      HeightmapSelectTest{ SurfaceType::kVirtual, SurfaceType::kVirtual, HeightmapSelect::kAbove },
      HeightmapSelectTest{ SurfaceType::kReal, SurfaceType::kVoid, HeightmapSelect::kAbove },
      HeightmapSelectTest{ SurfaceType::kVoid, SurfaceType::kReal, HeightmapSelect::kBelow },
      HeightmapSelectTest{ SurfaceType::kVirtual, SurfaceType::kVoid, HeightmapSelect::kAbove },
      HeightmapSelectTest{ SurfaceType::kVoid, SurfaceType::kVirtual, HeightmapSelect::kBelow },
      HeightmapSelectTest{ SurfaceType::kVoid, SurfaceType::kVoid, HeightmapSelect::kAbove }
    };
  const size_t test_count = sizeof(surface_tests) / sizeof(surface_tests[0]);

  // Build expected results for the 3 different modes.
  const HeightmapTestResult test_results[3][test_count] =  //
    {                                                      // No virtual surfaces.
      {
        //
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kUnknown, HeightmapSelect::kNull },
        HeightmapTestResult{ HeightmapVoxelType::kUnknown, HeightmapSelect::kNull },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kUnknown, HeightmapSelect::kNull },
        HeightmapTestResult{ HeightmapVoxelType::kUnknown, HeightmapSelect::kNull },
        HeightmapTestResult{ HeightmapVoxelType::kUnknown, HeightmapSelect::kBelow },
      },
      // Allow virtual surfaces.
      {
        //
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kUnknown, HeightmapSelect::kBelow },
      },
      // Promote virtual surfaces.
      {
        //
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kBelow },
        HeightmapTestResult{ HeightmapVoxelType::kVirtualSurface, HeightmapSelect::kAbove },
        HeightmapTestResult{ HeightmapVoxelType::kUnknown, HeightmapSelect::kBelow },
      }
    };

  const int debug_mode_start = 0;
  const int debug_mode_last = 2;
  const size_t debug_test_start = 0;
  const size_t debug_test_last = test_count - 1;
  for (int m = debug_mode_start; m < std::min(3, debug_mode_last + 1); ++m)
  {
    SurfaceMode mode = (SurfaceMode)m;
    for (size_t t = debug_test_start; t < std::min(test_count, debug_test_last + 1); ++t)
    {
      // Build a context string for error reporting.
      std::ostringstream str;
      str << t << ": " << kModeName[m];
      surface_tests[t].fillContext(str);
      // Run test
      testSurface(mode, surface_tests[t], test_results[m][t], str.str());
    }
  }
}