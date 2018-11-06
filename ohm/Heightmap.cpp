// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Heightmap.h"

#include "private/HeightmapDetail.h"

#include "HeightmapVoxel.h"
#include "MapCache.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "Voxel.h"

using namespace ohm;

Heightmap::Heightmap(double grid_resolution, ohm::OccupancyMap *map)
  : imp_(new HeightmapDetail)
{
  imp_->occupancy_map = map;
  imp_->heightmap_plane = glm::dvec4(0, 0, 1, 0);
  // Use an OccupancyMap to store grid cells. Each region is 1 voxel thick.
  imp_->heightmap.reset(new OccupancyMap(grid_resolution, glm::u8vec3(128, 128, 1)));

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
  voxels.addMember("min_offset", DataType::kFloat, 0);
  voxels.addMember("clearance_offset", DataType::kFloat, 0);
}


Heightmap::~Heightmap()
{
  delete imp_;
}


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
  // Brute force initial approach.

  // Clear previous results.
  imp_->heightmap->clear();

  // Cache the plane.
  imp_->heightmap_plane = plane;

  // 1. Calculate the map extents.
  //  a. Calculate occupancy map extents.
  //  b. Project occupancy map extents onto heightmap plane.
  // 2. Populate heightmap voxels

  // Get the regions we'll work with.
  std::vector<const MapChunk *> regions;
  OccupancyMap &src_map = *imp_->occupancy_map;
  OccupancyMap &heightmap = *imp_->heightmap;
  src_map.enumerateRegions(regions);

  // Populate heightmap (supports arbitrary heightmap plane):
  // - Foreach source OccupancyMap region overlapping bounds
  //  - Foreach occupied (or unknown) region voxel
  //    - project position onto heightmap plane
  //    - calculate distance to heightmap
  //    - generate key for project position ino heightmap grid
  //    - get heightmap grid cell
  //    - update min_offset with shortest distance.

  const glm::u8vec3 voxel_dimensions = src_map.regionVoxelDimensions();
  Key voxel_key;
  glm::dvec3 voxel_coord;
  const glm::dvec3 plane_normal(plane);
  MapCache cache;
  double signed_distance_to_plane;
  HeightmapVoxel *voxel_content = nullptr;
  for (const MapChunk *region : regions)
  {
    if (region->first_valid_index.x < voxel_dimensions.x && region->first_valid_index.y < voxel_dimensions.y &&
        region->first_valid_index.y < voxel_dimensions.z)
    {
      voxel_key = Key(region->region.coord, region->first_valid_index);
      do
      {
        // Check if the source voxel is occupied.
        const VoxelConst src_voxel(voxel_key, region, src_map.detail());
        if (src_voxel.isOccupied())
        {
          // Generate a spatial point for the voxel.
          voxel_coord = src_map.voxelCentreGlobal(voxel_key);
          signed_distance_to_plane = glm::dot(voxel_coord, plane_normal) + plane.w;
          voxel_coord -= plane_normal * signed_distance_to_plane;

          // Add to the height map.
          Voxel voxel = heightmap.voxel(heightmap.voxelKey(voxel_coord), true, &cache);
          voxel.setValue(heightmap.occupancyThresholdValue());

          voxel_content = voxel.layerContent<HeightmapVoxel *>(imp_->heightmap_layer);
          if (signed_distance_to_plane < voxel_content->min_offset)
          {
            voxel_content->min_offset = signed_distance_to_plane;
          }
        }
        // Project onto the plane.
      } while (nextLocalKey(voxel_key, voxel_dimensions));
    }
  }

  // How to calculate clearance_offset?
  // - It's the next shortest distance after a set of free in voxels of at least distance D.
  // - May need a second pass to calculate the clearance.
  //  - Also update min_offset to the highest of a continuous set of occupied voxels.
  return true;
}
