// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/OccupancyMap.h>
#include <ohm/MapCache.h>
#include <ohm/MapSerialise.h>

#include <ohmtools/OhmCloud.h>
#include <ohmtools/OhmGen.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/Profile.h>
#include <ohmutil/PlyMesh.h>

using namespace ohm;
// using namespace ohmutil;

TEST(Heightmap, Simple)
{
  const float boundary_distance = 2.5f;
  OccupancyMap map(0.2);

  // Build a cloud with real samples around a cubic boundary. Does not cover every voxel in the boundary.
  ohmgen::boxRoom(map, glm::dvec3(-boundary_distance), glm::dvec3(boundary_distance));

  Heightmap heightmap(0.2, 1.0);
  heightmap.setOccupancyMap(&map);

  heightmap.update(Heightmap::AxisZ);

  // Verify output. Boundaries should be at ~ +boundary_distance (top of walls). All other voxels should be at
  // ~ -boundary_distance. Only approximage due to quantisation.

  const Key min_key = heightmap.heightmap().voxelKey(glm::dvec3(-boundary_distance));
  const Key max_key = heightmap.heightmap().voxelKey(glm::dvec3( boundary_distance));

  const double ground_height = map.voxelCentreGlobal(map.voxelKey(glm::dvec3(-boundary_distance))).z;
  const double top_of_wall_height = map.voxelCentreGlobal(map.voxelKey(glm::dvec3(boundary_distance))).z;

  ohm::save("heightmap-simple-source.ohm", map);
  ohm::save("heightmap-simple.ohm", heightmap.heightmap());

  ohmtools::saveCloud("heightmap-simple-source.ply", map);
  ohmtools::saveCloud("heightmap-simple-2d.ply", heightmap.heightmap());

  // Save the 2.5d heightmap.
  PlyMesh heightmapCloud;
  glm::dvec3 coord;
  for (auto iter = heightmap.heightmap().begin(); iter != heightmap.heightmap().end(); ++iter)
  {
    if (iter->isOccupied())
    {
      const HeightmapVoxel *voxel = iter->layerContent<const HeightmapVoxel *>(heightmap.heightmapVoxelLayer());
      // Get the coordinate of the voxel.
      coord = heightmap.heightmap().voxelCentreGlobal(iter.key());
      // Adjust the height to the plane height and offset.
      // For now assume a horizontal plane through the origin..
      coord.z = voxel->height;
      // Add to the cloud.
      heightmapCloud.addVertex(coord);
    }
  }

  heightmapCloud.save("heightmap-simple.ply", true);
  MapCache cache;
  Key key = min_key;
  key.setRegionAxis(2, 0);
  key.setLocalAxis(2, 0);
  for (; key.isBoundedY(min_key, max_key); heightmap.heightmap().stepKey(key, 1, 1))
  {
    for (key.setAxisFrom(0, min_key); key.isBoundedX(min_key, max_key); heightmap.heightmap().stepKey(key, 0, 1))
    {
      VoxelConst voxel = heightmap.heightmap().voxel(key, false, &cache);

      double expected_height = ground_height;
      if (key.axisMatches(0, min_key) || key.axisMatches(0, max_key) ||
          key.axisMatches(1, min_key) || key.axisMatches(1, max_key))
      {
        expected_height = top_of_wall_height;
      }

      ASSERT_TRUE(voxel.isValid());

      const HeightmapVoxel *voxel_content = voxel.layerContent<const HeightmapVoxel *>(heightmap.heightmapVoxelLayer());
      ASSERT_NE(voxel_content, nullptr);
      ASSERT_NEAR(voxel_content->height, expected_height, 1e-9);
    }
  }
}
