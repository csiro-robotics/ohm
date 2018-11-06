// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Heightmap.h"

#include "private/HeightmapDetail.h"

#include "HeightmapVoxel.h"
#include "Key.h"
#include "MapCache.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"

using namespace ohm;

namespace
{
  glm::dvec3 projectOnPlane(const glm::dvec4 &plane, const glm::dvec3 &point, double *signed_distance_to_plane)
  {
    const glm::dvec3 plane_normal(plane);
    *signed_distance_to_plane = glm::dot(point, plane_normal) + plane.w;
    return point - plane_normal * *signed_distance_to_plane;
  }
}  // namespace

Heightmap::Heightmap(double grid_resolution, double min_clearance, unsigned region_size)
  : imp_(new HeightmapDetail)
{
  region_size = region_size ? region_size : kDefaultRegionSize;

  imp_->min_clearance = min_clearance;
  imp_->heightmap_plane = glm::dvec4(0, 0, 1, 0);
  // Use an OccupancyMap to store grid cells. Each region is 1 voxel thick.
  imp_->heightmap.reset(new OccupancyMap(grid_resolution, glm::u8vec3(region_size, region_size, 1)));

  // Setup the heightmap voxel layout.
  MapLayout &layout = imp_->heightmap->layout();

  MapLayer *layer;
  VoxelLayout voxels;

  const float invalid_marker_value = voxel::invalidMarkerValue();
  const float max_clearance = std::numeric_limits<float>::max();
  int max_clearance_int = 0;
  static_assert(sizeof(max_clearance) == sizeof(max_clearance_int), "size mismatch");

  memcpy(&max_clearance_int, &max_clearance, sizeof(max_clearance));

  size_t clear_value = 0;
  memcpy(&clear_value, &invalid_marker_value, std::min(sizeof(invalid_marker_value), sizeof(clear_value)));
  layer = layout.addLayer("occupancy", 0);
  voxels = layer->voxelLayout();
  voxels.addMember("occupancy", DataType::kFloat, clear_value);

  // Initialise the data structure to have both ranges at float max.
  memset(&clear_value, max_clearance_int, sizeof(clear_value));
  layer = layout.addLayer(HeightmapVoxel::kHeightmapLayer, 0);
  imp_->heightmap_layer = layer->layerIndex();
  voxels = layer->voxelLayout();
  voxels.addMember("height", DataType::kFloat, 0);
  voxels.addMember("clearance", DataType::kFloat, 0);
}


Heightmap::~Heightmap()
{}


void Heightmap::setOccupancyMap(OccupancyMap *map)
{
  imp_->occupancy_map = map;
}


OccupancyMap *Heightmap::occupancyMap() const
{
  return imp_->occupancy_map;
}


OccupancyMap &Heightmap::heightmap() const
{
  return *imp_->heightmap;
}


void Heightmap::setMinClearance(double clearance)
{
  imp_->min_clearance = clearance;
}


double Heightmap::minClearance() const
{
  return imp_->min_clearance;
}


const glm::dvec4 &Heightmap::plane() const
{
  return imp_->heightmap_plane;
}


unsigned Heightmap::heightmapVoxelLayer() const
{
  return imp_->heightmap_layer;
}


bool Heightmap::update(const glm::dvec4 &plane)
{
  if (!imp_->occupancy_map)
  {
    return false;
  }

  if (std::abs(plane.x) > 1e-9 || std::abs(plane.y) > 1e-9)
  {
    // Current implementation must have horizontal plane.
    return false;
  }

  // Brute force initial approach.
  const OccupancyMap &src_map = *imp_->occupancy_map;
  OccupancyMap &heightmap = *imp_->heightmap;

  // Clear previous results.
  heightmap.clear();

  // Cache the plane.
  imp_->heightmap_plane = plane;
  const glm::dvec3 plane_normal(plane);

  // 1. Calculate the map extents.
  //  a. Calculate occupancy map extents.
  //  b. Project occupancy map extents onto heightmap plane.
  // 2. Populate heightmap voxels

  glm::dvec3 min_ext, max_ext;
  src_map.calculateExtents(min_ext, max_ext);

  // Generate keys for these extents.
  const Key min_ext_key = heightmap.voxelKey(min_ext);
  const Key max_ext_key = heightmap.voxelKey(max_ext);
  Key target_key = min_ext_key;

  // Collapse the height.
  target_key.setRegionAxis(2, 0);
  target_key.setLocalAxis(2, 0);

  // TODO(KS): address differences in voxel resolution between source and destination maps.
  // Walk the heightmap voxels which are potentially occupied. We only walk the X/Y plane.
  MapCache heightmap_cache;
  MapCache src_cache;
  for (; target_key.isBoundedY(min_ext_key, max_ext_key); heightmap.stepKey(target_key, 1, 1))
  {
    target_key.setLocalAxis(0, min_ext_key.localKey().x);
    target_key.setRegionAxis(0, min_ext_key.regionKey().x);
    for (; target_key.isBoundedX(min_ext_key, max_ext_key); heightmap.stepKey(target_key, 0, 1))
    {
      HeightmapVoxel column_details;
      column_details.height = std::numeric_limits<float>::max();
      column_details.clearance = -1.0;
      // Start walking the voxels in the source map.
      glm::dvec3 column_reference = heightmap.voxelCentreGlobal(target_key);
      // Set to the min Z extents of the source map.
      column_reference.z = min_ext.z;
      const Key src_min_key = src_map.voxelKey(column_reference);
      column_reference.z = max_ext.z;
      const Key src_max_key = src_map.voxelKey(column_reference);

      // Walk the src column up.
      for (Key src_key = src_min_key; src_key.isBoundedZ(src_min_key, src_max_key); src_map.stepKey(src_key, 2, 1))
      {
        VoxelConst src_voxel = src_map.voxel(src_key, &src_cache);
        if (src_voxel.isOccupied())
        {
          // Determine the height offset for src_voxel.
          double height;
          const glm::dvec3 src_voxel_centre = src_map.voxelCentreGlobal(src_key);
          height = glm::dot(src_voxel_centre, plane_normal);
          if (height < column_details.height)
          {
            // First voxel in column.
            column_details.height = height;
          }
          else if (column_details.clearance < 0)
          {
            // No clearance value.
            column_details.clearance = height - column_details.height;
            if (column_details.clearance >= imp_->min_clearance)
            {
              // Found our heightmap voxels.
              break;
            }
            else
            {
              // Insufficient clearance. This becomes our new base voxel; keep looking for clearance.
              column_details.height = height;
              column_details.clearance = -1.0;
            }
          }
        }
      }

      // Commit the voxel if required.
      if (column_details.height < std::numeric_limits<float>::max())
      {
        if (column_details.clearance < 0)
        {
          // No clearance information.
          column_details.clearance = imp_->min_clearance;
        }

        Voxel heightmap_voxel = heightmap.voxel(target_key, true, &heightmap_cache);
        heightmap_voxel.setValue(heightmap.occupancyThresholdValue());
        HeightmapVoxel *voxel_content = heightmap_voxel.layerContent<HeightmapVoxel *>(imp_->heightmap_layer);
        *voxel_content = column_details;
      }
    }
  }

  return true;
}
