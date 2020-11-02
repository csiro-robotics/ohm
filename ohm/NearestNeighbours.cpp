// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "NearestNeighbours.h"

#include "private/MapLayoutDetail.h"
#include "private/NearestNeighboursDetail.h"
#include "private/OccupancyMapDetail.h"
#include "private/OccupancyQueryAlg.h"

#include "DefaultLayer.h"
#include "Key.h"
#include "MapChunk.h"
#include "OccupancyMap.h"
#include "QueryFlag.h"
#include "VoxelBlock.h"
#include "VoxelBuffer.h"
#include "VoxelOccupancy.h"

#include <3esservermacros.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <functional>
#include <iostream>

using namespace ohm;

namespace
{
unsigned regionNearestNeighboursCpu(OccupancyMap &map, NearestNeighboursDetail &query, const glm::i16vec3 &region_key,
                                    ClosestResult &closest)
{
  const float invalid_occupancy_value = unobservedOccupancyValue();
  const OccupancyMapDetail &map_data = *map.detail();
  const auto chunk_search = map_data.chunks.find(region_key);
  glm::vec3 query_origin, voxel_vector;
  Key voxel_key(nullptr);
  const MapChunk *chunk = nullptr;
  const uint8_t *occupancy_mem = nullptr;
  float range_squared = 0;
  unsigned added = 0;
  VoxelBuffer<VoxelBlock> voxel_buffer;

  std::function<bool(float, const OccupancyMapDetail &)> voxel_occupied_func;

  query_origin = glm::vec3(query.near_point - map.origin());

  if (chunk_search == map_data.chunks.end())
  {
    // The entire region is unknown space...
    if ((query.query_flags & ohm::kQfUnknownAsOccupied) == 0)
    {
      // ... and unknown space is considered free. No results to add.
      return 0;
    }

    // ... and we have to treat unknown space as occupied.
    chunk = nullptr;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    occupancy_mem = reinterpret_cast<const uint8_t *>(&invalid_occupancy_value);
    // Setup voxel occupancy test function to pass all voxels in this region.
    voxel_occupied_func = [](const float, const OccupancyMapDetail &) -> bool { return true; };
  }
  else
  {
    chunk = chunk_search->second;
    // FIXME: (KS) This is a bit of a mix of legacy direct voxel access and newer VoxelBlock access. Makes things a
    // bit unclear.
    voxel_buffer = VoxelBuffer<VoxelBlock>(chunk->voxel_blocks[chunk->layout().occupancyLayer()]);
    occupancy_mem = voxel_buffer.voxelMemory();
    // Setup the voxel test function to check the occupancy threshold and behaviour flags.
    voxel_occupied_func = [&query](const float voxel, const OccupancyMapDetail &map_data) -> bool {
      if (voxel == unobservedOccupancyValue())
      {
        if (query.query_flags & ohm::kQfUnknownAsOccupied)
        {
          return true;
        }
        return false;
      }
      if (voxel >= map_data.occupancy_threshold_value ||
          (query.query_flags & ohm::kQfUnknownAsOccupied) && voxel == unobservedOccupancyValue())
      {
        return true;
      }
      return false;
    };
  }

  TES_STMT(std::vector<tes::Vector3d> includedOccupied);
  TES_STMT(std::vector<tes::Vector3d> excludedOccupied);
  TES_STMT(std::vector<tes::Vector3d> includedUncertain);
  TES_STMT(std::vector<tes::Vector3d> excludedUncertain);
  // TES_BOX_W(g_tes, TES_COLOUR(LightSeaGreen), 0u,
  //           glm::value_ptr(region_centre), glm::value_ptr(map.regionSpatialResolution()));

  float occupancy;
  for (int z = 0; z < map_data.region_voxel_dimensions.z; ++z)
  {
    for (int y = 0; y < map_data.region_voxel_dimensions.y; ++y)
    {
      for (int x = 0; x < map_data.region_voxel_dimensions.x; ++x)
      {
        memcpy(&occupancy, occupancy_mem, sizeof(float));
        if (voxel_occupied_func(occupancy, map_data))
        {
          // Occupied voxel, or invalid voxel to be treated as occupied.
          // Calculate range to centre.
          voxel_key = Key(region_key, x, y, z);
          voxel_vector = map.voxelCentreLocal(voxel_key);
          voxel_vector -= query_origin;
          range_squared = glm::dot(voxel_vector, voxel_vector);
          if (range_squared <= query.search_radius * query.search_radius)
          {
            query.intersected_voxels.push_back(voxel_key);
            query.ranges.push_back(std::sqrt(range_squared));

            if (range_squared < closest.range)
            {
              closest.index = query.intersected_voxels.size() - 1;
              closest.range = range_squared;
            }

            ++added;
#ifdef TES_ENABLE
            if (occupancy != unobservedOccupancyValue())
            {
              includedOccupied.push_back(tes::Vector3d(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
            }
            else
            {
              includedUncertain.push_back(tes::Vector3d(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
            }
#endif  // TES_ENABLE
          }
#ifdef TES_ENABLE
          else
          {
            if (occupancy != unobservedOccupancyValue())
            {
              excludedOccupied.push_back(tes::Vector3d(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
            }
            else
            {
              excludedUncertain.push_back(tes::Vector3d(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
            }
          }
#endif  // TES_ENABLE
        }

        // Next voxel. Leave pointer as is (pointing to invalid_occupancy_value) if the chunk is invalid.
        occupancy_mem += (chunk != nullptr) ? sizeof(occupancy) : 0;
      }
    }
  }

#ifdef TES_ENABLE
  // Visualise points.
  if (!excludedUncertain.empty())
  {
    TES_POINTS(g_tes, TES_COLOUR(Salmon), tes::Id(), tes::DataBuffer(excludedUncertain));
  }

  if (!includedUncertain.empty())
  {
    TES_POINTS(g_tes, TES_COLOUR(PowderBlue), tes::Id(), tes::DataBuffer(includedUncertain));
  }

  if (!excludedOccupied.empty())
  {
    TES_POINTS(g_tes, TES_COLOUR(Orange), tes::Id(), tes::DataBuffer(excludedOccupied));
  }

  if (!includedOccupied.empty())
  {
    TES_POINTS(g_tes, TES_COLOUR(LightSkyBlue), tes::Id(), tes::DataBuffer(includedOccupied));
  }
#endif  //  TES_ENABLE

  TES_SERVER_UPDATE(g_tes, 0.0f);

  return added;
}
}  // namespace


NearestNeighbours::NearestNeighbours(NearestNeighboursDetail *detail)
  : Query(detail)
  , query_flags_(0)
{
  // Can't initalise GPU until onSetMap() (need a valid map).
}


NearestNeighbours::NearestNeighbours(OccupancyMap &map, const glm::dvec3 &near_point, float search_radius,
                                     unsigned query_flags)
  : Query(new NearestNeighboursDetail)
  , query_flags_(query_flags)
{
  setMap(&map);
  setNearPoint(near_point);
  setSearchRadius(search_radius);
  setQueryFlags(query_flags);
}


NearestNeighbours::~NearestNeighbours()
{
  NearestNeighboursDetail *d = imp();
  delete d;
  imp_ = nullptr;
}


glm::dvec3 NearestNeighbours::nearPoint() const
{
  const NearestNeighboursDetail *d = imp();
  return d->near_point;
}


void NearestNeighbours::setNearPoint(const glm::dvec3 &point)
{
  NearestNeighboursDetail *d = imp();
  d->near_point = point;
}


float NearestNeighbours::searchRadius() const
{
  const NearestNeighboursDetail *d = imp();
  return d->search_radius;
}


void NearestNeighbours::setSearchRadius(float range)
{
  NearestNeighboursDetail *d = imp();
  d->search_radius = range;
}


bool NearestNeighbours::onExecute()
{
  NearestNeighboursDetail *d = imp();

  if (!d->map)
  {
    return false;
  }

  ClosestResult closest;

  const glm::dvec3 min_extents = d->near_point - glm::dvec3(d->search_radius, d->search_radius, d->search_radius);
  const glm::dvec3 max_extents = d->near_point + glm::dvec3(d->search_radius, d->search_radius, d->search_radius);

  // Create debug visualisation objects. We use the map address to persist objects.
  // Search sphere.
  TES_SPHERE_W(g_tes, TES_COLOUR_A(GreenYellow, 128), tes::Id(d->map),
               tes::Spherical(tes::Vector3d(glm::value_ptr(d->near_point)), double(d->search_radius)));
  // Search bounds.
  TES_BOX_W(g_tes, TES_COLOUR(FireBrick), tes::Id(d->map),
            tes::Transform(tes::Vector3d(glm::value_ptr(0.5 * min_extents + max_extents)),
                           tes::Vector3d(glm::value_ptr(max_extents - min_extents))));

  ohm::occupancyQueryRegions(*d->map, *d, closest, min_extents, max_extents, &regionNearestNeighboursCpu);

  TES_SERVER_UPDATE(g_tes, 0.0f);
  // Delete debug objects.
  TES_SPHERE_END(g_tes, uint32_t((size_t)d->map));
  TES_BOX_END(g_tes, uint32_t((size_t)d->map));

  if ((d->query_flags & kQfNearestResult) && !d->intersected_voxels.empty())
  {
    d->intersected_voxels[0] = d->intersected_voxels[closest.index];
    d->intersected_voxels.resize(1);
    d->ranges[0] = d->ranges[closest.index];
    d->number_of_results = 1u;
    d->ranges.resize(1);
  }
  else
  {
    d->number_of_results = d->intersected_voxels.size();
  }

  return true;
}


bool NearestNeighbours::onExecuteAsync()
{
  return false;
}


void NearestNeighbours::onReset(bool /*hard_reset*/)
{
  // NearestNeighboursDetail *d = imp();
}


NearestNeighboursDetail *NearestNeighbours::imp()
{
  return static_cast<NearestNeighboursDetail *>(imp_);
}


const NearestNeighboursDetail *NearestNeighbours::imp() const
{
  return static_cast<const NearestNeighboursDetail *>(imp_);
}
