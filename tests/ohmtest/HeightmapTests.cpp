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
#include <ohm/KeyRange.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Trace.h>
#include <ohm/TriangleNeighbours.h>
#include <ohm/VoxelData.h>
#include <ohm/VoxelLayout.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/Profile.h>

#include <sstream>
#include <unordered_set>
#include <utility>

using namespace ohm;

namespace
{
const double kBoxHalfExtents = 2.5;

/// Test heightmap generation parameters.
struct HeightmapParams
{
  /// Half extents of the ground surface.
  double map_half_extents = 6.0;
  /// Half extents of a platform to place above the surface.
  double platform_half_extents = 2.0;
  /// Height of the platform.
  double platform_height = 1.5;
  /// Generate some virtual surfaces in the test map?
  bool generate_virtual_surfaces = false;
  /// Do virtual surfaces occlude visibility of real surfaces below them? This simulates the way in which virtual
  /// surfaces would be generated in real situations where they represent an optimistic best possible slope estimate
  /// observed. Real voxels below our virtual surface voxels are removed as if they have not been observed.
  /// Ignored if @c generate_virtual_surfaces is false.
  bool virtual_surface_occlusion = true;
};

/// Populated with information about the heightmap generated in @c populateMultiLevelMap() to be used for validation.
struct HeightmapGeneratedInfo
{
  /// Set of voxel keys for voxels which represent a potential surface.
  std::unordered_set<ohm::Key, ohm::Key::Hash> surface;
  /// Set of voxel keys for voxels which are potential virtual surfaces.
  std::unordered_set<ohm::Key, ohm::Key::Hash> virtual_surface;
  /// Set of voxel keys for voxels at which the platform joins the floor. This will generally be excluded from a layered
  /// heightmap.
  std::unordered_set<ohm::Key, ohm::Key::Hash> seam;

  /// Default constructor.
  HeightmapGeneratedInfo() = default;
  /// Copy constructor.
  HeightmapGeneratedInfo(const HeightmapGeneratedInfo &other) = default;
  /// Move constructor.
  /// @param other Object to move.
  HeightmapGeneratedInfo(HeightmapGeneratedInfo &&other)
    : surface(std::move(other.surface))
    , virtual_surface(std::move(other.virtual_surface))
  {}
};

/// Try make the given occupancy voxel virtual. This is a helper function for @c populateMultiLevelMap().
void tryMakeVirtual(ohm::Voxel<float> &occupancy, const HeightmapParams &params, HeightmapGeneratedInfo &info)
{
  // Note we don't forcibly set the miss value so as not to unnecessarily erode occupied voxels.
  ohm::integrateMiss(occupancy);
  if (ohm::isFree(occupancy))
  {
    // Potential virtual.
    ohm::Key key = occupancy.key();
    info.virtual_surface.insert(key);
    // Occlude ptential surfacec below by changing them to unobserved.
    if (params.virtual_surface_occlusion)
    {
      // Walk down to the base level eating voxels.
      const ohm::OccupancyMap &map = *occupancy.map();
      do
      {
        map.moveKeyAlongAxis(key, 2, -1);
        occupancy.setKey(key);
        if (ohm::isOccupied(occupancy))
        {
          occupancy.write(ohm::unobservedOccupancyValue());
          info.surface.erase(key);
        }
      } while (map.voxelCentreGlobal(key)[2] > 0);
    }
  }
}

/// Populate an occupancy map with multiple layers. The generated map is a flat square with smaller raised section
/// connected to the lower surface by a ramp on two sides.
///
/// Virtual surfaces may be optionally included in the map as shallower ramps overshadowing one platform ramp and
/// another descending from the platform as a psuedo third ramp.
///
/// @param map The map to populate (assumed empty)
/// @param generate_virtual_surfaces True to include virtual surfaces in the map.
/// @return The height of the platform above the lower surface.
HeightmapGeneratedInfo populateMultiLevelMap(ohm::OccupancyMap &map, const HeightmapParams &params)
{
  HeightmapGeneratedInfo info;

  ohm::Voxel<float> occupancy(&map, map.layout().occupancyLayer());
  // ASSERT_TRUE(occupancy.isLayerValid());

  // Build extents to traverse in keys. This will ensure we touch every voxel correctly.
  ohm::KeyRange key_range(map.voxelKey(glm::dvec3(-params.map_half_extents, -params.map_half_extents, 0)),
                          map.voxelKey(glm::dvec3(params.map_half_extents, params.map_half_extents, 0)), map);

  // Create the floor.
  for (const Key &key : key_range)
  {
    occupancy.setKey(key);
    occupancy.write(map.hitValue());
    info.surface.insert(key);
  }

  key_range = ohm::KeyRange(
    map.voxelKey(glm::dvec3(-params.platform_half_extents, -params.platform_half_extents, params.platform_height)),
    map.voxelKey(glm::dvec3(params.platform_half_extents, params.platform_half_extents, params.platform_height)), map);

  // Create the raised platform.
  for (const Key &key : key_range)
  {
    occupancy.setKey(key);
    occupancy.write(map.hitValue());
    info.surface.insert(key);
  }

  // Create a vertical range to iterate from platform to floor. We'll walk out the X axis as well as we iterate.
  Key side_range_min_key1 = map.voxelKey(glm::dvec3(-params.platform_half_extents, -params.platform_half_extents, 0));
  Key side_range_min_key2 = map.voxelKey(glm::dvec3(params.platform_half_extents, -params.platform_half_extents, 0));
  map.moveKeyAlongAxis(side_range_min_key1, 2, 1);
  map.moveKeyAlongAxis(side_range_min_key2, 2, 1);
  const ohm::KeyRange side_key_ranges[2] = {
    ohm::KeyRange(
      side_range_min_key1,
      map.voxelKey(glm::dvec3(-params.platform_half_extents, -params.platform_half_extents, params.platform_height)),
      map),
    ohm::KeyRange(
      side_range_min_key2,
      map.voxelKey(glm::dvec3(params.platform_half_extents, -params.platform_half_extents, params.platform_height)),
      map)
  };

  int y_range_voxel_count = int((2 * params.platform_half_extents) / map.resolution());
  // Build a ramp either side. We do 2 iterations, one for each size.
  for (int i = 0; i < 2; ++i)
  {
    // Because we will be walking up, the x offset has to start at the maximum value.
    // We walk X in lockstep with the height for a 45 degree slope, so we read the Z axis range.
    int x_offset = -(side_key_ranges[i].range()[2]);

    // First iteration marks the seam between ground and platform.
    bool first_row = true;
    for (const Key &ref_key : side_key_ranges[i])
    {
      Key key = ref_key;
      // Second side used x_offset
      map.moveKeyAlongAxis(key, 0, x_offset * (i == 0 ? 1 : -1));
      // Create a second range to walk along the Y axis.
      ohm::KeyRange y_key_range(key, key, map);
      map.moveKeyAlongAxis(key, 1, y_range_voxel_count);
      y_key_range.setMaxKey(key);

      // Walk the Y axis line.
      for (const Key &key : y_key_range)
      {
        occupancy.setKey(key);
        occupancy.write(map.hitValue());
        info.surface.insert(key);

        if (first_row)
        {
          // Voxels below the first row are the seam.
          ohm::Key seam_key = key;
          map.moveKeyAlongAxis(seam_key, 2, -1);
          info.seam.insert(seam_key);
        }
      }
      // Setup for next iteration.
      ++x_offset;
      first_row = false;
    }
  }

  if (params.generate_virtual_surfaces)
  {
    // Add two virtual surfaces.
    // 1. Create a third, virtual ramp from the platform where there is no real ramp.
    // 2. Hide a ramp at a shallower slope.

    // 1. Virtual ramp
    const ohm::KeyRange virtual_ramp_range = ohm::KeyRange(
      map.voxelKey(glm::dvec3(-params.platform_half_extents, -params.platform_half_extents, 0)),
      map.voxelKey(glm::dvec3(-params.platform_half_extents, -params.platform_half_extents, params.platform_height)),
      map);

    // This loop is the same as above, but swaps X and Y axes. Cut & paste is simpler for this unit test than trying to
    // generalise the code.
    int x_range_voxel_count = int((2 * params.platform_half_extents) / map.resolution());
    // Build a ramp either side. We do 2 iterations, one for each size.
    // Because we will be walking up, the x offset has to start at the maximum value.
    // We walk X in lockstep with the height for a 45 degree slope, so we read the Z axis range.
    int y_offset = -(virtual_ramp_range.range()[2]);
    for (const Key &ref_key : virtual_ramp_range)
    {
      Key key = ref_key;
      // Second side used y_offset
      map.moveKeyAlongAxis(key, 1, y_offset * 1);
      // Create a second range to walk along the Y axis.
      ohm::KeyRange x_key_range(key, key, map);
      map.moveKeyAlongAxis(key, 0, x_range_voxel_count);
      x_key_range.setMaxKey(key);

      // Walk the Y axis line.
      for (const Key &key2 : x_key_range)
      {
        occupancy.setKey(key2);
        tryMakeVirtual(occupancy, params, info);
      }
      // Setup for next iteration.
      ++y_offset;
    }

    // 2. Hide a ramp at a shallower slope.
    y_range_voxel_count = int((2 * params.platform_half_extents) / map.resolution());
    // Because we will be walking up, the x offset has to start at the maximum value.
    // We walk X in lockstep with the height for a 45 degree slope, so we read the Z axis range.
    int x_offset = -2 * (side_key_ranges[0].range()[2]);
    for (const Key &ref_key : side_key_ranges[0])
    {
      // We walk 2 voxels along X for each 1 in Y to create a shallower slope.
      for (int i = 0; i < 2; ++i)
      {
        Key key = ref_key;
        // Second side used x_offset
        map.moveKeyAlongAxis(key, 0, x_offset * 1);
        // Create a second range to walk along the Y axis.
        ohm::KeyRange y_key_range(key, key, map);
        map.moveKeyAlongAxis(key, 1, y_range_voxel_count);

        y_key_range.setMaxKey(key);

        // Walk the Y axis line.
        for (const Key &key2 : y_key_range)
        {
          occupancy.setKey(key2);
          tryMakeVirtual(occupancy, params, info);
        }
        // Setup for next iteration.
        ++x_offset;
      }
    }
  }

  // Validate the virtual surface set: some of the collected voxels will be supported by an occupied voxel. That does
  // not qualify as a virtual surface candidate.
  for (auto iter = info.virtual_surface.begin(); iter != info.virtual_surface.end();)
  {
    Key key = *iter;
    bool valid_virtual_surface = false;
    occupancy.setKey(key);
    if (ohm::isFree(occupancy))
    {
      // It's a valid free voxel. Now check that the voxel below is null or unobserved.
      map.moveKeyAlongAxis(key, 2, -1);  // Move the key to the voxel below.
      occupancy.setKey(key);             // Update the voxel reference to the new key.
      if (ohm::isUnobservedOrNull(occupancy))
      {
        // It's a valid virtual surface.
        valid_virtual_surface = true;
      }
    }

    if (valid_virtual_surface)
    {
      ++iter;
    }
    else
    {
      // Not a valid virtual surface voxel - remove it.
      iter = info.virtual_surface.erase(iter);
    }
  }

  return info;
}

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

      ASSERT_TRUE(voxel.isValid()) << (wall ? "top" : "floor");

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

TEST(Heightmap, Clearance)
{
  ohm::OccupancyMap map(0.1);
  const HeightmapParams params;
  populateMultiLevelMap(map, params);
  ohm::save("clearance-map.ohm", map);
  ohmtools::saveCloud("clearance-map.ply", map);

  // Define a heightmap for which there is no clearance requirement. The map we just generated has a smooth, flat
  // surface with a raised platform above like this (2D section):
  ///       _____
  //       /     \          backslashes in ascii
  //      /       \         are fun
  // ____/_________\____
  //
  // So the reference_map will consist of the lower surface voxels, but also indicate the clearance height for each
  // of these to the surface above. We'll later use this to validate the clearance constrained heigthmap.
  ohm::Heightmap reference_heightmap(map.resolution(), 0);
  reference_heightmap.setOccupancyMap(&map);
  reference_heightmap.buildHeightmap(glm::dvec3(0));

  // Now add a clearance constraint.
  const double clearance_constraint = 0.5 * params.platform_height;
  ohm::Heightmap constrained_heightmap(map.resolution(), clearance_constraint);
  constrained_heightmap.setOccupancyMap(&map);
  constrained_heightmap.setUseFloodFill(true);
  constrained_heightmap.buildHeightmap(glm::dvec3(0));

  ohm::save("clearance-hm.ohm", constrained_heightmap.heightmap());
  ohmtools::saveCloud("clearance-hm.ply", constrained_heightmap.heightmap());

  // Now validate the clearance of each voxel in the constrained heigthmap and validate against the reported clearance
  // from the reference map.
  KeyRange validation_range;
  constrained_heightmap.heightmap().calculateExtents(nullptr, nullptr, &validation_range);

  ohm::Voxel<ohm::HeightmapVoxel> ref_voxel(&reference_heightmap.heightmap(),
                                            reference_heightmap.heightmapVoxelLayer());
  ohm::Voxel<ohm::HeightmapVoxel> constrained_voxel(&constrained_heightmap.heightmap(),
                                                    constrained_heightmap.heightmapVoxelLayer());

  ASSERT_TRUE(ref_voxel.isLayerValid());
  ASSERT_TRUE(constrained_voxel.isLayerValid());

  // Note: Key references in the two heightmaps should exactly match.
  for (const auto &key : validation_range)
  {
    ref_voxel.setKey(key);
    constrained_voxel.setKey(key);

    ASSERT_TRUE(ref_voxel.isValid());
    ASSERT_TRUE(constrained_voxel.isValid());

    const HeightmapVoxel ref = ref_voxel.data();
    const HeightmapVoxel test_value = constrained_voxel.data();

    if (test_value.clearance > 0)
    {
      EXPECT_GE(test_value.clearance, clearance_constraint);
    }

    // Validate that we've chose the correct voxel.
    if (ref.clearance >= clearance_constraint)
    {
      EXPECT_EQ(test_value.height, ref.height);
    }
  }
}

enum class LayeredTestStart
{
  kUnderPlatform,
  kOnPlatform,
  kOutside
};

void heightmapLayeredTest(const std::string &name, LayeredTestStart start_location)
{
  // For this test we use the multi-layered map generation. For validation, we set the target clearance to zero which
  // allows the heightmap to be a precise representation of the original map. We can then convert the heightmap back
  // into a normal occupancy map and compare that against the input map. It should be identical.
  ohm::Trace trace(std::string("heightmap-") + name + ".3es");  // Setup debug trace for 3rd Eye Scene visualisation.
  // Create the input map.
  ohm::OccupancyMap map(0.1);
  const HeightmapParams params;
  const HeightmapGeneratedInfo heightmap_info = populateMultiLevelMap(map, params);
  ohm::save(name + "-map.ohm", map);
  ohmtools::saveCloud(name + "-map.ply", map);

  // Disable any clearance constraint.
  const double clearance_constraint = 0;
  ohm::Heightmap layered_heightmap(map.resolution(), clearance_constraint);
  layered_heightmap.setOccupancyMap(&map);
  layered_heightmap.heightmap().setOrigin(map.origin());

  // Build the layered heightmap.
  layered_heightmap.setMode(ohm::HeightmapMode::kLayeredFillOrdered);

  glm::dvec3 seed_pos(0);
  switch (start_location)
  {
  case LayeredTestStart::kUnderPlatform:
    seed_pos = glm::dvec3(0, 0, 0);
    break;
  case LayeredTestStart::kOnPlatform:
    seed_pos = glm::dvec3(0, 0, params.platform_height);
    break;
  case LayeredTestStart::kOutside:
    // Set a position outside the observed bounds of the map. We'll choose the minimum extents of the map.
    {
      ohm::Key seed_key;
      map.calculateExtents(nullptr, nullptr, &seed_key, nullptr);
      seed_pos = map.voxelCentreGlobal(seed_key);
    }
    break;
  default:
    break;
  }
  layered_heightmap.buildHeightmap(seed_pos);

  ohmtools::SaveCloudOptions save_opt;
  ohmtools::ColourHeightmapType colour_by_type(layered_heightmap.heightmap());
  save_opt.colour_select = [&colour_by_type](const ohm::Voxel<const float> &occupancy) {
    return colour_by_type.select(occupancy);
  };
  save_opt.export_free = true;
  ohm::save(name + "-hm.ohm", layered_heightmap.heightmap());
  ohmtools::saveHeightmapCloud(name + "-hm.ply", layered_heightmap.heightmap(),
                               ohmtools::SaveHeightmapCloudOptions(save_opt));

  // Now convert the heightmap back into a normal occupancy map.
  ohm::OccupancyMap test_map(map.resolution());
  test_map.setOrigin(map.origin());

  // HACK: want to visualise the seam.
  ohm::PlyMesh ply;
  for (const ohm::Key &key : heightmap_info.seam)
  {
    const glm::dvec3 pos = map.voxelCentreGlobal(key);
    ply.addVertex(pos);
  }
  ply.save("seam.ply", true);

  ohm::Voxel<float> new_occupancy(&test_map, test_map.layout().occupancyLayer());
  ASSERT_TRUE(new_occupancy.isLayerValid());
  unsigned test_voxel_count = 0;
  for (auto iter = layered_heightmap.heightmap().begin(); iter != layered_heightmap.heightmap().end(); ++iter)
  {
    glm::dvec3 voxel_pos{};
    auto hm_voxel_type = layered_heightmap.getHeightmapVoxelInfo(*iter, &voxel_pos);
    if (hm_voxel_type == HeightmapVoxelType::kSurface)
    {
      // Get the voxel position from the heightmap.
      new_occupancy.setKey(test_map.voxelKey(voxel_pos));
      ASSERT_TRUE(new_occupancy.isValid());
      ohm::integrateHit(new_occupancy);
      ++test_voxel_count;
    }
  }

  new_occupancy.reset();

  // Ensure we've extracted something.
  ASSERT_GT(test_voxel_count, 0);
  ohm::save(name + "-map-out.ohm", test_map);
  ohmtools::saveCloud(name + "-map-out.ply", test_map);

  // Now validate the extracted test_map against the original map. We should exactly reconstruct the original map.
  ohm::Voxel<const float> ref_occupancy(&map, map.layout().occupancyLayer());
  ohm::Voxel<const float> test_occupancy(&test_map, test_map.layout().occupancyLayer());

  unsigned ref_voxel_count = 0;
  for (auto iter = map.begin(); iter != map.end(); ++iter)
  {
    ref_occupancy.setKey(iter);  // Set key from iterator for efficiency (chunk pointer already available)
    ASSERT_TRUE(ref_occupancy.isValid());

    // Skip seam voxels. These won't be present in the test map.
    if (heightmap_info.seam.find(*iter) != heightmap_info.seam.end())
    {
      continue;
    }

    test_occupancy.setKey(*iter);  // Set from Key as we are referencing a different map object.
    ASSERT_TRUE(test_occupancy.isValid());
    // Check that the occupancy types match. The actual occupancies may differ.
    const auto ref_occupancy_type = ohm::occupancyType(ref_occupancy);
    const auto test_occupancy_type = ohm::occupancyType(test_occupancy);
    EXPECT_EQ(test_occupancy_type, ref_occupancy_type);
    if (ref_occupancy_type == ohm::kOccupied)
    {
      ++ref_voxel_count;
    }
  }

  ASSERT_GT(ref_voxel_count, 0);
  ASSERT_EQ(test_voxel_count, ref_voxel_count);
}

TEST(Heightmap, LayeredAbove)
{
  // For this test we use the multi-layered map generation. For validation, we set the target clearance to zero which
  // allows the heightmap to be a precise representation of the original map. We can then convert the heightmap back
  // into a normal occupancy map and compare that against the input map. It should be identical.
  heightmapLayeredTest("layered-above", LayeredTestStart::kOnPlatform);
}

TEST(Heightmap, LayeredBelow)
{
  heightmapLayeredTest("layered-below", LayeredTestStart::kUnderPlatform);
}

TEST(Heightmap, LayeredExternal)
{
  heightmapLayeredTest("layered-external", LayeredTestStart::kOutside);
}


/// Parameters for @c testHeightmapVirtualSurface()
///
/// Note that we will not find all voxels from the original map. Quirks based on mode are listed below.
///
/// - @c ohm::HeightmapMode::kPlanar misses 2215 voxels below the platform and 41 seam voxels.
/// - @c ohm::HeightmapMode::kSimpleFill also misses 2215, but these are the platform voxels and 41 seam voxels.
/// - @c ohm::HeightmapMode::kLayeredFill[Ordered] misses 41 seam voxels. Seam removal deals with this.
struct VirtualSurfaceTestParams
{
  /// Heightmap generation mode.
  ohm::HeightmapMode mode = ohm::HeightmapMode::kPlanar;
  /// Heightmap floor search distance.
  double floor = 0.5;
  /// Heightmap ceiling search distance. Large to prefer finding the lower surface below the platform.
  double ceiling = 2.0;
  /// Permitted number of surface voxels missed in the heightmap.
  unsigned allowed_missed_surface_count = 0;
};


void testHeightmapVirtualSurface(const std::string &name, const VirtualSurfaceTestParams test_params)
{
  // For this test, we build the multilevel map, then add a virtual surface at an angle over one of the ramp slopes.
  // When we generate a map at the lower level, we should see no impact of the virtual surface.
  // When we generate at the higher level, we should see the virtual surface obscure parts of the ramp.
  // allows the heightmap to be a precise representation of the original map. We can then convert the heightmap back
  // into a normal occupancy map and compare that against the input map. It should be identical.
  ohm::Trace trace("heightmap-virtual-surface-" + name + ".3es");  // Setup debug trace for 3rd Eye Scene visualisation.
  // Create the input map.
  ohm::OccupancyMap map(0.1);
  HeightmapParams params;
  ohmtools::SaveCloudOptions save_opt;
  // Ensure we select the virtual surfaces by removing other options below.
  params.generate_virtual_surfaces = true;
  HeightmapGeneratedInfo heightmap_info = populateMultiLevelMap(map, params);

  ohmtools::ColourByType colour_by_type(map);
  save_opt.colour_select = [&colour_by_type](const ohm::Voxel<const float> &occupancy) {
    return colour_by_type.select(occupancy);
  };
  save_opt.export_free = true;
  ohm::save("virtual-surface-" + name + ".ohm", map);
  ohmtools::saveCloud("virtual-surface-" + name + ".ply", map, save_opt);

  HeightmapGeneratedInfo validation_info = heightmap_info;
  // Disable any clearance constraint.
  const double clearance_constraint = 0.0;
  ohm::Heightmap layered_heightmap(map.resolution(), clearance_constraint);
  layered_heightmap.setOccupancyMap(&map);
  layered_heightmap.heightmap().setOrigin(map.origin());

  // Build the layered heightmap.
  layered_heightmap.setMode(test_params.mode);
  layered_heightmap.setGenerateVirtualSurface(true);
  layered_heightmap.setCeiling(test_params.floor);
  layered_heightmap.setCeiling(test_params.ceiling);
  layered_heightmap.buildHeightmap(glm::dvec3(0, 0, 1.1 * params.platform_height));

  ohmtools::ColourHeightmapType colour_by_heightmap_type(layered_heightmap.heightmap());
  save_opt.colour_select = [&colour_by_heightmap_type](const ohm::Voxel<const float> &occupancy) {
    return colour_by_heightmap_type.select(occupancy);
  };
  save_opt.export_free = true;
  ohm::save("virtual-surface-" + name + "-hm.ohm", layered_heightmap.heightmap());
  ohmtools::saveHeightmapCloud("virtual-surface-" + name + "-hm.ply", layered_heightmap.heightmap(), save_opt);

  // Walk the heightmap extracting all the voxels. With a zero clearance constraint, we should have an exact
  // representation of the original map with the addition of virtual surfaces.
  ohm::Voxel<const float> src_occupancy(&map, map.layout().occupancyLayer());
  glm::dvec3 hm_pos{};
  for (auto iter = layered_heightmap.heightmap().begin(); iter != layered_heightmap.heightmap().end(); ++iter)
  {
    const ohm::HeightmapVoxelType voxel_type = layered_heightmap.getHeightmapVoxelInfo(*iter, &hm_pos, nullptr);
    switch (voxel_type)
    {
    case ohm::HeightmapVoxelType::kSurface:
      // Validate that this corresponds to an occupied voxel in the source map.
      src_occupancy.setKey(map.voxelKey(hm_pos));
      ASSERT_EQ(ohm::occupancyType(src_occupancy), ohm::kOccupied);
      // Remove this from the list of surface keys. Well validate we touched everything below.
      // We must first convert the heightmap position into a key within the source map.
      validation_info.surface.erase(map.voxelKey(hm_pos));
      break;
    case ohm::HeightmapVoxelType::kVirtualSurface:
      // Validate that this corresponds to a free voxel in the source map.
      src_occupancy.setKey(map.voxelKey(hm_pos));
      ASSERT_EQ(ohm::occupancyType(src_occupancy), ohm::kFree);
      // Remove this from the list of virtual surface keys. Well validate we touched everything below.
      // We must first convert the heightmap position into a key within the source map.
      validation_info.virtual_surface.erase(map.voxelKey(hm_pos));
      break;
    case ohm::HeightmapVoxelType::kVacant:
      // Validate that this corresponds to an unobserved or null voxel in the source map.
      ASSERT_TRUE(ohm::isUnobservedOrNull(src_occupancy));
      break;
    default:
      // no op
      break;
    }
  }

  // None of the modes find all voxels in a seam. remove those from the detection set.
  for (auto iter = validation_info.surface.begin(); iter != validation_info.surface.end();)
  {
    // Check for seam voxel.
    auto seam_iter = validation_info.seam.find(*iter);
    if (seam_iter != validation_info.seam.end())
    {
      // Remove the seam voxel.
      iter = validation_info.surface.erase(iter);
    }
    else
    {
      // No removal. Continue iteration.
      ++iter;
    }
  }

  // Ensure we have represented every relevant voxel in the original map.
  EXPECT_EQ(validation_info.surface.size(), test_params.allowed_missed_surface_count);
  EXPECT_EQ(validation_info.virtual_surface.size(), 0);
}


TEST(Heightmap, VirtualSurfacePlanar)
{
  VirtualSurfaceTestParams params;
  params.mode = ohm::HeightmapMode::kPlanar;
  // Allow missing the voxels below the platform.
  params.allowed_missed_surface_count = 2215;
  testHeightmapVirtualSurface("planar", params);
}


TEST(Heightmap, VirtualSurfaceFill)
{
  VirtualSurfaceTestParams params;
  params.mode = ohm::HeightmapMode::kSimpleFill;
  // Allow missing the voxels of the platform.
  params.allowed_missed_surface_count = 2215;
  testHeightmapVirtualSurface("fill", params);
}


TEST(Heightmap, VirtualSurfaceSimpleLayered)
{
  VirtualSurfaceTestParams params;
  params.mode = ohm::HeightmapMode::kLayeredFill;
  params.floor = params.ceiling = 0.5;
  testHeightmapVirtualSurface("simple-layered", params);
}


TEST(Heightmap, VirtualSurfaceLayered)
{
  // For this test, we build the multilevel map, then add a virtual surface at an angle over one of the ramp slopes.
  // When we generate a map at the lower level, we should see no impact of the virtual surface.
  // When we generate at the higher level, we should see the virtual surface obscure parts of the ramp.
  // allows the heightmap to be a precise representation of the original map. We can then convert the heightmap back
  // into a normal occupancy map and compare that against the input map. It should be identical.
  ohm::Trace trace("heightmap-virtual-surface-layered.3es");  // Setup debug trace for 3rd Eye Scene visualisation.
  // Create the input map.
  ohm::OccupancyMap map(0.1);
  HeightmapParams params;
  // Ensure we select the virtual surfaces by removing other options below.
  params.generate_virtual_surfaces = true;
  params.virtual_surface_occlusion = false;
  HeightmapGeneratedInfo validation_info = populateMultiLevelMap(map, params);
  ohm::save("layered-virtual-surface-layered.ohm", map);
  ohmtools::saveCloud("layered-virtual-surface-layered.ply", map);

  // Disable any clearance constraint.
  const double clearance_constraint = 0.0;
  ohm::Heightmap layered_heightmap(map.resolution(), clearance_constraint);
  layered_heightmap.setOccupancyMap(&map);
  layered_heightmap.heightmap().setOrigin(map.origin());

  // Build the layered heightmap.
  layered_heightmap.setMode(ohm::HeightmapMode::kLayeredFillOrdered);
  layered_heightmap.setGenerateVirtualSurface(true);
  // Use tight ceiling/floor constraints to ensure we capture some virtual surfaces.
  layered_heightmap.setCeiling(3 * map.resolution());
  layered_heightmap.setFloor(5 * map.resolution());
  layered_heightmap.buildHeightmap(glm::dvec3(0, 0, 1.1 * params.platform_height));

  ohmtools::SaveCloudOptions save_opt;
  ohmtools::ColourHeightmapType colour_by_type(layered_heightmap.heightmap());
  save_opt.colour_select = [&colour_by_type](const ohm::Voxel<const float> &occupancy) {
    return colour_by_type.select(occupancy);
  };
  save_opt.export_free = true;
  ohm::save("layered-virtual-surface-layered-hm.ohm", layered_heightmap.heightmap());
  ohmtools::saveHeightmapCloud("layered-virtual-surface-layered-hm.ply", layered_heightmap.heightmap(), save_opt);

  // // Walk the heightmap extracting all the voxels. With a zero clearance constraint, we should have an exact
  // // representation of the original map with the addition of virtual surfaces.
  // ohm::Voxel<const float> src_occupancy(&map, map.layout().occupancyLayer());
  // glm::dvec3 hm_pos{};
  // for (auto iter = layered_heightmap.heightmap().begin(); iter != layered_heightmap.heightmap().end(); ++iter)
  // {
  //   const ohm::HeightmapVoxelType voxel_type = layered_heightmap.getHeightmapVoxelInfo(*iter, &hm_pos, nullptr);
  //   switch (voxel_type)
  //   {
  //   case ohm::HeightmapVoxelType::kSurface:
  //     // Validate that this corresponds to an occupied voxel in the source map.
  //     src_occupancy.setKey(map.voxelKey(hm_pos));
  //     ASSERT_EQ(ohm::occupancyType(src_occupancy), ohm::kOccupied);
  //     // Remove this from the list of surface keys. Well validate we touched everything below.
  //     // We must first convert the heightmap position into a key within the source map.
  //     validation_info.surface.erase(map.voxelKey(hm_pos));
  //     break;
  //   case ohm::HeightmapVoxelType::kVirtualSurface:
  //     // Validate that this corresponds to a free voxel in the source map.
  //     src_occupancy.setKey(map.voxelKey(hm_pos));
  //     ASSERT_EQ(ohm::occupancyType(src_occupancy), ohm::kFree);
  //     // Remove this from the list of virtual surface keys. Well validate we touched everything below.
  //     // We must first convert the heightmap position into a key within the source map.
  //     validation_info.virtual_surface.erase(map.voxelKey(hm_pos));
  //     break;
  //   case ohm::HeightmapVoxelType::kVacant:
  //     // Validate that this corresponds to an unobserved or null voxel in the source map.
  //     ASSERT_TRUE(ohm::isUnobservedOrNull(src_occupancy));
  //     break;
  //   default:
  //     // no op
  //     break;
  //   }
  // }

  // // Ensure we have represented every relevant voxel in the original map.
  // EXPECT_TRUE(validation_info.surface.empty());
  // EXPECT_TRUE(validation_info.virtual_surface.empty());
}
