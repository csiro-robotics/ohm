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

#include <3esservermacros.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <functional>
#include <iostream>

using namespace ohm;

namespace
{
  unsigned regionNearestNeighboursCpu(OccupancyMap &map,               // NOLINT(google-runtime-references)
                                      NearestNeighboursDetail &query,  // NOLINT(google-runtime-references)
                                      const glm::i16vec3 &region_key,
                                      ClosestResult &closest)  // NOLINT(google-runtime-references)
  {
    static const float invalid_occupancy_value = voxel::invalidMarkerValue();
    const OccupancyMapDetail &map_data = *map.detail();
    const auto chunk_search = map_data.chunks.find(region_key);
    glm::vec3 query_origin, voxel_vector;
    Key voxel_key(nullptr);
    const MapChunk *chunk = nullptr;
    const float *occupancy = nullptr;
    float range_squared = 0;
    unsigned added = 0;

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
      occupancy = &invalid_occupancy_value;
      // Setup voxel occupancy test function to pass all voxels in this region.
      voxel_occupied_func = [](const float, const OccupancyMapDetail &) -> bool { return true; };
    }
    else
    {
      chunk = chunk_search->second;
      occupancy = chunk->layout->layer(chunk->layout->occupancyLayer()).voxelsAs<float>(*chunk);
      // Setup the voxel test function to check the occupancy threshold and behaviour flags.
      voxel_occupied_func = [&query](const float voxel, const OccupancyMapDetail &map_data) -> bool {
        if (voxel == voxel::invalidMarkerValue())
        {
          if (query.query_flags & ohm::kQfUnknownAsOccupied)
          {
            return true;
          }
          return false;
        }
        if (voxel >= map_data.occupancy_threshold_value ||
            (query.query_flags & ohm::kQfUnknownAsOccupied) && voxel == voxel::invalidMarkerValue())
        {
          return true;
        }
        return false;
      };
    }

    TES_STMT(std::vector<tes::Vector3f> includedOccupied);
    TES_STMT(std::vector<tes::Vector3f> excludedOccupied);
    TES_STMT(std::vector<tes::Vector3f> includedUncertain);
    TES_STMT(std::vector<tes::Vector3f> excludedUncertain);
    // TES_BOX_W(g_3es, TES_COLOUR(LightSeaGreen), 0u,
    //           glm::value_ptr(region_centre), glm::value_ptr(map.regionSpatialResolution()));

    for (int z = 0; z < map_data.region_voxel_dimensions.z; ++z)
    {
      for (int y = 0; y < map_data.region_voxel_dimensions.y; ++y)
      {
        for (int x = 0; x < map_data.region_voxel_dimensions.x; ++x)
        {
          if (voxel_occupied_func(*occupancy, map_data))
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
              if (occupancy && *occupancy != voxel::invalidMarkerValue())
              {
                includedOccupied.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
              }
              else
              {
                includedUncertain.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
              }
#endif  // TES_ENABLE
            }
#ifdef TES_ENABLE
            else
            {
              if (occupancy && *occupancy != voxel::invalidMarkerValue())
              {
                excludedOccupied.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
              }
              else
              {
                excludedUncertain.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(voxel_key))));
              }
            }
#endif  // TES_ENABLE
          }

          // Next voxel. Will be null for uncertain regions.
          occupancy = (occupancy) ? occupancy + 1 : occupancy;
        }
      }
    }

#ifdef TES_ENABLE
    // Visualise points.
    if (!excludedUncertain.empty())
    {
      TES_POINTS(g_3es, TES_COLOUR(Salmon), &excludedUncertain.data()->x, (unsigned)excludedUncertain.size(),
                 sizeof(tes::Vector3f));
    }

    if (!includedUncertain.empty())
    {
      TES_POINTS(g_3es, TES_COLOUR(PowderBlue), &includedUncertain.data()->x, (unsigned)includedUncertain.size(),
                 sizeof(tes::Vector3f));
    }

    if (!excludedOccupied.empty())
    {
      TES_POINTS(g_3es, TES_COLOUR(Orange), &excludedOccupied.data()->x, (unsigned)excludedOccupied.size(),
                 sizeof(tes::Vector3f));
    }

    if (!includedOccupied.empty())
    {
      TES_POINTS(g_3es, TES_COLOUR(LightSkyBlue), &includedOccupied.data()->x, (unsigned)includedOccupied.size(),
                 sizeof(tes::Vector3f));
    }
#endif  //  TES_ENABLE

    TES_SERVER_UPDATE(g_3es, 0.0f);

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
  TES_SPHERE_W(g_3es, TES_COLOUR_A(GreenYellow, 128), uint32_t((size_t)d->map), glm::value_ptr(d->near_point),
               d->search_radius);
  // Search bounds.
  TES_BOX_W(g_3es, TES_COLOUR(FireBrick), uint32_t((size_t)d->map), glm::value_ptr(0.5 * min_extents + max_extents),
            glm::value_ptr(max_extents - min_extents));

  ohm::occupancyQueryRegions(*d->map, *d, closest, min_extents, max_extents, &regionNearestNeighboursCpu);

  TES_SERVER_UPDATE(g_3es, 0.0f);
  // Delete debug objects.
  TES_SPHERE_END(g_3es, uint32_t((size_t)d->map));
  TES_BOX_END(g_3es, uint32_t((size_t)d->map));

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
