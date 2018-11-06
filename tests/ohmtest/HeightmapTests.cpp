// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/OccupancyMap.h>
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

  heightmap.update(glm::dvec4(0, 0, 1, 0));

  ohm::save("heightmap-source.ohm", map);
  ohm::save("heightmap.ohm", heightmap.heightmap());

  ohmtools::saveCloud("heightmap-source.ply", map);
  ohmtools::saveCloud("heightmap-2d.ply", heightmap.heightmap());

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
      coord.z = heightmap.plane().w + voxel->min_offset;
      // Add to the cloud.
      heightmapCloud.addVertex(coord);
    }
  }

  heightmapCloud.save("heightmap.ply", true);
}
