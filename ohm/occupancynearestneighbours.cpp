// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancynearestneighbours.h"

#include "mapchunk.h"
#include "mapregion.h"
#include "occupancykey.h"
#include "occupancymap.h"
#include "occupancyqueryflag.h"
#include "ohmdefaultlayers.h"
#include "private/maplayoutdetail.h"
#include "private/occupancymapdetail.h"
#include "private/occupancynearestneighboursdetail.h"
#include "private/occupancyqueryalg.h"
#include "occupancygpu.h"

#include <3esservermacros.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>

#include <gputil/gpupinnedbuffer.h>

using namespace ohm;

// Prototypes for GPU API specific functions.
int initialiseNNGpuProgram(NearestNeighboursDetail &query, gputil::Device &gpu);
int invokeNNQueryGpu(const OccupancyMapDetail &map, NearestNeighboursDetail &query, NearestNeighboursDetail::GpuData &gpuData);
void releaseNNGpu(NearestNeighboursDetail &query);

namespace
{
  // TODO: base this value on GPU capabilities.
  const size_t GpuBatchSize = 32*32*32;

  int initialiseGpu(const OccupancyMapDetail &map, NearestNeighboursDetail &query)
  {
    if (query.gpuOk)
    {
      return 0;
    }

    NearestNeighboursDetail::GpuData &gpuData = query.gpuData;
    query.gpu = gpuDevice();

    gpuData.queue = query.gpu.defaultQueue();
    gpuData.localRanges.resize(GpuBatchSize);
    gpuData.regionKeys.resize(GpuBatchSize);
    gpuData.localKeys.resize(GpuBatchSize);

    // Use the default device.
    int err = 0;
    err = initialiseNNGpuProgram(query, ohm::gpuDevice());
    if (err)
    {
      return err;
    }

    // Initialise buffers.
    gpuData.gpuNodes = gputil::Buffer(query.gpu, GpuBatchSize * sizeof(float), gputil::BF_ReadHost);
    gpuData.gpuNodeRegionKeys = gputil::Buffer(query.gpu, GpuBatchSize * sizeof(NearestNeighboursDetail::GpuData::short3), gputil::BF_ReadHost);
    gpuData.gpuNodeVoxelKeys = gputil::Buffer(query.gpu, GpuBatchSize * sizeof(NearestNeighboursDetail::GpuData::uchar3), gputil::BF_ReadHost);
    gpuData.gpuRanges = gputil::Buffer(query.gpu, GpuBatchSize * sizeof(float), gputil::BF_WriteHost);
    gpuData.gpuResulRegionKeys = gputil::Buffer(query.gpu, GpuBatchSize * sizeof(NearestNeighboursDetail::GpuData::short3), gputil::BF_WriteHost);
    gpuData.gpuResultNodeKeys = gputil::Buffer(query.gpu, GpuBatchSize * sizeof(NearestNeighboursDetail::GpuData::uchar3), gputil::BF_WriteHost);
    gpuData.gpuResultCount = gputil::Buffer(query.gpu, sizeof(unsigned), gputil::BF_ReadWriteHost);

    query.gpuOk = (err == 0);

    return err;
  }

  unsigned regionNearestNeighboursCpu(OccupancyMap &map, NearestNeighboursDetail &query,
                                      const glm::i16vec3 &regionKey,
                                      ClosestResult &closest
                                     )
  {
    const OccupancyMapDetail &mapData = *map.detail();
    auto chunkSearch = mapData.findRegion(regionKey);
    glm::vec3 queryOrigin, nodeVector, regionCentre;
    OccupancyKey nodeKey(nullptr);
    const MapChunk *chunk = nullptr;
    const float *occupancy = nullptr;
    float rangeSquared = 0;
    unsigned added = 0;

    std::function<bool (const float, const OccupancyMapDetail &)> nodeOccupiedFunc;

    queryOrigin = glm::vec3(query.nearPoint - map.origin());

    if (chunkSearch == mapData.chunks.end())
    {
      // The entire region is unknown space...
      if ((query.queryFlags & ohm::QF_UnknownAsOccupied) == 0)
      {
        // ... and unknown space is considered free. No results to add.
        return 0;
      }

      // ... and we have to treat unknown space as occupied.
      chunk = nullptr;
      occupancy = nullptr;
      regionCentre = map.regionCentreGlobal(regionKey);
      // Setup node occupancy test function to pass all nodes in this region.
      nodeOccupiedFunc = [](const float, const OccupancyMapDetail &) -> bool
      {
        return true;
      };
    }
    else
    {
      chunk = chunkSearch->second;
      occupancy = chunk->layout->layer(DL_Occupancy).voxelsAs<float>(*chunk);
      regionCentre = chunk->region.centre;
      // Setup the node test function to check the occupancy threshold and behaviour flags.
      nodeOccupiedFunc = [&query](const float voxel, const OccupancyMapDetail & mapData) -> bool
      {
        if (voxel == NodeBase::invalidMarkerValue())
        {
          if ((query.queryFlags & ohm::QF_UnknownAsOccupied))
          {
            return true;
          }
          return false;
        }
        if (voxel >= mapData.occupancyThresholdValue ||
            (query.queryFlags & ohm::QF_UnknownAsOccupied) && voxel == NodeBase::invalidMarkerValue())
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
    TES_BOX_W(g_3es, TES_COLOUR(LightSeaGreen), 0u,
              glm::value_ptr(regionCentre), glm::value_ptr(map.regionSpatialResolution()));

    for (int z = 0; z < mapData.regionVoxelDimensions.z; ++z)
    {
      for (int y = 0; y < mapData.regionVoxelDimensions.y; ++y)
      {
        for (int x = 0; x < mapData.regionVoxelDimensions.x; ++x)
        {
          if (nodeOccupiedFunc(*occupancy, mapData))
          {
            // Occupied node, or invalid node to be treated as occupied.
            // Calculate range to centre.
            nodeKey = OccupancyKey(regionKey, x, y, z);
            nodeVector = map.voxelCentreLocal(nodeKey);
            nodeVector -= queryOrigin;
            rangeSquared = glm::dot(nodeVector, nodeVector);
            if (rangeSquared <= query.searchRadius * query.searchRadius)
            {
              query.intersectedVoxels.push_back(nodeKey);
              query.ranges.push_back(std::sqrt(rangeSquared));

              if (rangeSquared < closest.range)
              {
                closest.index = query.intersectedVoxels.size() - 1;
                closest.range = rangeSquared;
              }

              ++added;
#ifdef TES_ENABLE
              if (occupancy && *occupancy != NodeBase::invalidMarkerValue())
              {
                includedOccupied.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(nodeKey))));
              }
              else
              {
                includedUncertain.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(nodeKey))));
              }
#endif // TES_ENABLE
            }
#ifdef TES_ENABLE
            else
            {
              if (occupancy && *occupancy != NodeBase::invalidMarkerValue())
              {
                excludedOccupied.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(nodeKey))));
              }
              else
              {
                excludedUncertain.push_back(tes::V3Arg(glm::value_ptr(map.voxelCentreGlobal(nodeKey))));
              }
            }
#endif // TES_ENABLE
          }

          // Next node. Will be null for uncertain regions.
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
#endif //  TES_ENABLE

    TES_SERVER_UPDATE(g_3es, 0.0f);

    return added;
  }

  // Wait for the given GPU queue to finish and migrate the data into query.
  unsigned finishGpuOperation(NearestNeighboursDetail &query, NearestNeighboursDetail::GpuData &gpuData, ClosestResult &closest)
  {
    unsigned added = 0;

    //---------------------------------
    const unsigned zero = 0u;
    gpuData.gpuResultCount.fill(&zero, sizeof(zero));
    invokeNNQueryGpu(*query.map->detail(), query, gpuData);

    // Block until the currently queued operations complete.
    gpuData.queue.finish();

    gpuData.gpuResultCount.read(&gpuData.resultCount, sizeof(gpuData.resultCount));
    if (gpuData.resultCount)
    {
      gpuData.gpuRanges.readElements(gpuData.localRanges.data(), gpuData.resultCount);
      gpuData.gpuResulRegionKeys.readElements(gpuData.regionKeys.data(), sizeof(*gpuData.regionKeys.data()),
                                              gpuData.resultCount, 0, sizeof(NearestNeighboursDetail::GpuData::short3));
      gpuData.gpuResultNodeKeys.readElements(gpuData.localKeys.data(), sizeof(*gpuData.localKeys.data()),
                                             gpuData.resultCount, 0, sizeof(NearestNeighboursDetail::GpuData::uchar3));

#ifndef VALIDATE_KEYS
      OccupancyKey key;
      // Collate results.
      for (unsigned i = 0; i < gpuData.resultCount; ++i)
      {
        key.setRegionKey(glm::i16vec3(gpuData.regionKeys[i].x, gpuData.regionKeys[i].y, gpuData.regionKeys[i].z));
        key.setLocalAxis(0, gpuData.localKeys[i].x);
        key.setLocalAxis(1, gpuData.localKeys[i].y);
        key.setLocalAxis(2, gpuData.localKeys[i].z);
        query.intersectedVoxels.push_back(key);
        query.ranges.push_back(gpuData.localRanges[i]);

        if (gpuData.localRanges[i] * gpuData.localRanges[i] < closest.range)
        {
          closest.index = query.intersectedVoxels.size();
          closest.range = gpuData.localRanges[i] * gpuData.localRanges[i];
        }
        ++added;
      }
#else  // VALIDATE_KEYS
      bool keyOk = true;
      unsigned nindex = 0;
      OccupancyKey key;
      double range, expectedRange;
      // Collate results.
      for (unsigned i = 0; i < gpuData.resultCount; ++i)
      {
        // Validate keys are as expected when writing all keys out.
        ++added;
        key.setRegionKey(glm::i16vec3(gpuData.regionKeys[i].x, gpuData.regionKeys[i].y, gpuData.regionKeys[i].z));
        key.setLocalAxis(0, gpuData.localKeys[i].x);
        key.setLocalAxis(1, gpuData.localKeys[i].y);
        key.setLocalAxis(2, gpuData.localKeys[i].z);
        nindex = nodeIndex(key, query.map->regionVoxelDimensions());
        query.intersectedVoxels.push_back(key);
        keyOk = nindex == i;
        if (!keyOk)
        {
          std::cout << "Key failure [" << i << "]: R("
                    << int(key.regionKey().x) << ' ' << int(key.regionKey().y) << ' ' << int(key.regionKey().z) << ") V("
                    << int(key.localKey().x) << ' ' << int(key.localKey().y) << ' ' << int(key.localKey().z) << ") : "
                    << gpuData.localRanges[i] << std::endl;
          nindex = nodeIndex(key, query.map->regionVoxelDimensions());
        }
        // Validate the range.
        range = gpuData.localRanges[i];
        expectedRange = glm::distance(query.nearPoint, query.map->voxelCentreGlobal(key));
        if (std::abs(range - expectedRange) > 1e-5)
        {
          std::cout << "  Range deviation. [" << i << "]: R("
                    << int(key.regionKey().x) << ' ' << int(key.regionKey().y) << ' ' << int(key.regionKey().z) << ") V("
                    << int(key.localKey().x) << ' ' << int(key.localKey().y) << ' ' << int(key.localKey().z) << ") : "
                    << "Expected/actual: " << expectedRange << ' ' << range << std::endl;
        }

        query.ranges.push_back(gpuData.localRanges[i]);

        if (gpuData.localRanges[i] * gpuData.localRanges[i] < closest.range)
        {
          closest.index = query.intersectedVoxels.size();
          closest.range = gpuData.localRanges[i] * gpuData.localRanges[i];
        }
      }
#endif // VALIDATE_KEYS
    }

    gpuData.clear();

    return added;
  }


  // Can handle splitting up regions which are larger than GpuBatchSize, but only when the
  // gpuData.queuedNodes size is zero on calling.
  bool regionNearestNeighboursGpu(const OccupancyMap &map, NearestNeighboursDetail &query,
                                  const glm::i16vec3 &regionKey,
                                  ClosestResult &closest,
                                  NearestNeighboursDetail::GpuData &gpuData
                                 )
  {
    const OccupancyMapDetail &data = *map.detail();
    auto chunkSearch = data.findRegion(regionKey);
    glm::vec3 regionCentre;
    const size_t nodesVolume = data.regionVoxelDimensions.x * data.regionVoxelDimensions.y * data.regionVoxelDimensions.z;

    NearestNeighboursDetail::GpuData::short3 gpuRegionKey = { regionKey.x, regionKey.y, regionKey.z };

    if (chunkSearch == data.chunks.end())
    {
      // The entire region is unknown space...
      if ((query.queryFlags & ohm::QF_UnknownAsOccupied) == 0)
      {
        // ... and unknown space is considered free. No results to add.
        return false;
      }
    }

    unsigned pushed = 0;
    int kx, ky, kz;
    bool needNewBatch = false;
    // Used to push voxel keys.
    kx = ky = kz = 0;

    // Handle having voxel regions exceeding the allowed GPU buffer sizes.
    while (pushed < map.regionVoxelVolume())
    {
      unsigned pushSize = (unsigned)std::min(nodesVolume - pushed, GpuBatchSize);
      needNewBatch = false;
      if (chunkSearch == data.chunks.end())
      {
        // The entire region is unknown space...
        // ... and we have to treat unknown space as occupied.
        // Fill the node buffer with zero to mark unknown space.
        const float occupiedNode = data.occupancyThresholdValue;
        gpuData.gpuNodes.fillPartial(&occupiedNode, sizeof(occupiedNode), pushSize * sizeof(occupiedNode), gpuData.queuedNodes * sizeof(occupiedNode));
        gpuData.gpuNodeRegionKeys.fillPartial(&gpuRegionKey, sizeof(gpuRegionKey), pushSize * sizeof(gpuRegionKey), gpuData.queuedNodes * sizeof(gpuRegionKey));
        regionCentre = map.regionCentreGlobal(regionKey);
      }
      else
      {
        const MapChunk *chunk = chunkSearch->second;
        const float *voxels = chunk->layout->layer(DL_Occupancy).voxelsAs<float>(*chunk);
        regionCentre = chunk->region.centre;

        gpuData.gpuNodes.write(voxels + pushed, pushSize * sizeof(*voxels), gpuData.queuedNodes * sizeof(*voxels));
        gpuData.gpuNodeRegionKeys.fillPartial(&gpuRegionKey, sizeof(gpuRegionKey), pushSize * sizeof(gpuRegionKey), gpuData.queuedNodes * sizeof(gpuRegionKey));
      }

      NearestNeighboursDetail::GpuData::uchar3 voxelKey;
      gputil::PinnedBuffer pinned(gpuData.gpuNodeVoxelKeys, gputil::PinWrite);

      kx = pushed % map.regionVoxelDimensions().x;
      ky = (pushed / map.regionVoxelDimensions().x) % map.regionVoxelDimensions().y;
      kz = (pushed / (map.regionVoxelDimensions().x * map.regionVoxelDimensions().x)) % map.regionVoxelDimensions().z;
      for (int z = kz; z < map.regionVoxelDimensions().z; ++z)
      {
        kz = 0;
        voxelKey.z = z;
        for (int y = ky; y < map.regionVoxelDimensions().y; ++y)
        {
          ky = 0;
          voxelKey.y = y;
          for (int x = kx; x < map.regionVoxelDimensions().x; ++x)
          {
            kx = 0;
            voxelKey.x = x;
            pinned.write(&voxelKey, sizeof(voxelKey), gpuData.queuedNodes * sizeof(voxelKey));
            ++gpuData.queuedNodes;
            ++pushed;
            if (gpuData.queuedNodes == GpuBatchSize)
            {
              // Pushed enough to process.
              finishGpuOperation(query, gpuData, closest);
              needNewBatch = true;
              break;
            }
          }

          if (needNewBatch)
          {
            // Pushed enough.
            break;
          }
        }
        if (needNewBatch)
        {
          // Pushed enough.
          break;
        }
      }
    }

    return true;
  }


  unsigned pushNodes(const OccupancyMap &map, NearestNeighboursDetail &query,
                     const glm::i16vec3 &regionKey,
                     ClosestResult &closest)
  {
    unsigned added = 0;
    NearestNeighboursDetail::GpuData &gpuData = query.gpuData;
    const size_t nodesVolume = map.regionVoxelVolume();

#ifdef VALIDATE_KEYS
    // Always complete previous queued operation when validating keys. This is to allow
    // us to infer each voxel index by its position in the results array.
    if (gpuData.queuedNodes)
#else  // VALIDATE_KEYS
    if (gpuData.queuedNodes && gpuData.queuedNodes + nodesVolume > GpuBatchSize)
#endif // VALIDATE_KEYS
    {
      // Complete existing queue.
      added = finishGpuOperation(query, gpuData, closest);
    }

    regionNearestNeighboursGpu(map, query, regionKey, closest, gpuData);
    return added;
  }


  unsigned occupancyNearestNeighboursGpu(OccupancyMap &map, NearestNeighboursDetail &query, ClosestResult &closest,
                                         const glm::dvec3 &minExtents, const glm::dvec3 &maxExtents)
  {
    const size_t nodesVolume = map.regionVoxelVolume();
    unsigned addedNeighbours = 0;
    std::function<unsigned (OccupancyMap &, NearestNeighboursDetail &, const glm::i16vec3 &, ClosestResult &)> rnnGpuFunc;
    rnnGpuFunc = pushNodes;

    addedNeighbours += ohm::occupancyQueryRegions(map, query, closest, minExtents, maxExtents, rnnGpuFunc);

    // Finalise queues.
    NearestNeighboursDetail::GpuData &gpuData = query.gpuData;
    if (gpuData.queuedNodes)
    {
      addedNeighbours += finishGpuOperation(query, gpuData, closest);
    }

    return addedNeighbours;
  }
}


NearestNeighbours::NearestNeighbours(NearestNeighboursDetail *detail)
  : Query(detail)
{
  // Can't initalise GPU until onSetMap() (need a valid map).
}


NearestNeighbours::NearestNeighbours(OccupancyMap &map, const glm::dvec3 &nearPoint,
                                     float searchRadius, unsigned queryFlags)
  : Query(new NearestNeighboursDetail)
{
  setMap(&map);
  setNearPoint(nearPoint);
  setSearchRadius(searchRadius);
  setQueryFlags(queryFlags);

  initialiseGpu(*map.detail(), *imp());
}


NearestNeighbours::~NearestNeighbours()
{
  NearestNeighboursDetail *d = imp();
  if (d)
  {
    releaseNNGpu(*d);
  }
  delete d;
  _imp = nullptr;
}


glm::dvec3 NearestNeighbours::nearPoint() const
{
  const NearestNeighboursDetail *d = imp();
  return d->nearPoint;
}


void NearestNeighbours::setNearPoint(const glm::dvec3 &point)
{
  NearestNeighboursDetail *d = imp();
  d->nearPoint = point;
}


float NearestNeighbours::searchRadius() const
{
  const NearestNeighboursDetail *d = imp();
  return d->searchRadius;
}


void NearestNeighbours::setSearchRadius(float range)
{
  NearestNeighboursDetail *d = imp();
  d->searchRadius = range;
}


void NearestNeighbours::onSetMap()
{
  NearestNeighboursDetail *d = imp();
  if (d->map)
  {
    initialiseGpu(*d->map->detail(), *d);
  }
}


bool NearestNeighbours::onExecute()
{
  NearestNeighboursDetail *d = imp();

  if (!d->map)
  {
    return false;
  }

  ClosestResult closest;

  // Calculate the query extents.
  glm::dvec3 minExtents, maxExtents;

  minExtents = d->nearPoint - glm::dvec3(d->searchRadius, d->searchRadius, d->searchRadius);
  maxExtents = d->nearPoint + glm::dvec3(d->searchRadius, d->searchRadius, d->searchRadius);

  // Create debug visualisation objects. We use the map address to persist objects.
  // Search sphere.
  TES_SPHERE_W(g_3es, TES_COLOUR_A(GreenYellow, 128), uint32_t((size_t)d->map),
                glm::value_ptr(d->nearPoint), d->searchRadius);
  // Search bounds.
  TES_BOX_W(g_3es, TES_COLOUR(FireBrick), uint32_t((size_t)d->map),
            glm::value_ptr(0.5 * minExtents + maxExtents),
            glm::value_ptr(maxExtents - minExtents));

  if (d->gpuOk && (d->queryFlags & QF_GpuEvaluate))
  {
    occupancyNearestNeighboursGpu(*d->map, *d, closest, minExtents, maxExtents);
  }
  else
  {
    ohm::occupancyQueryRegions(*d->map, *d, closest, minExtents, maxExtents, &regionNearestNeighboursCpu);
  }

  TES_SERVER_UPDATE(g_3es, 0.0f);
  // Delete debug objects.
  TES_SPHERE_END(g_3es, uint32_t((size_t)d->map));
  TES_BOX_END(g_3es, uint32_t((size_t)d->map));

  if ((d->queryFlags & QF_NearestResult) && !d->intersectedVoxels.empty())
  {
    d->intersectedVoxels[0] = d->intersectedVoxels[closest.index];
    d->intersectedVoxels.resize(1);
    d->ranges[0] = d->ranges[closest.index];
    d->numberOfResults = 1u;
    d->ranges.resize(1);
  }
  else
  {
    d->numberOfResults = d->intersectedVoxels.size();
  }

  return true;
}


bool NearestNeighbours::onExecuteAsync()
{
  return false;
}


void NearestNeighbours::onReset(bool hardReset)
{
  // NearestNeighboursDetail *d = imp();
}


NearestNeighboursDetail *NearestNeighbours::imp()
{
  return static_cast<NearestNeighboursDetail *>(_imp);
}


const NearestNeighboursDetail *NearestNeighbours::imp() const
{
  return static_cast<const NearestNeighboursDetail *>(_imp);
}
