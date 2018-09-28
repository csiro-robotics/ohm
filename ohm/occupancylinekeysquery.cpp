// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancylinekeysquery.h"

#include "private/occupancylinekeysquerydetail.h"
#include "occupancykeylist.h"
#include "occupancymap.h"
#include "occupancygpu.h"
#include "gpupinnedbuffer.h"
#include "occupancyutil.h"

#include <string.h>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <thread>

using namespace ohm;

// Prototypes for GPU API specific functions.
int initialiseLineKeysGpuProgram(LineKeysQueryDetail &query, gputil::Device &gpu);
int invokeLineKeysQueryGpu(LineKeysQueryDetail &query, LineKeysQueryDetail::GpuData &gpuData, bool (*completionFunc)(LineKeysQueryDetail &));
void releaseLineKeysGpu(LineKeysQueryDetail &query);

namespace
{
  bool readGpuResults(LineKeysQueryDetail &query);

  unsigned nextPow2(unsigned v)
  {
    // compute the next highest power of 2 of 32-bit v
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
  }

  // TODO: Verfy alignment.
  const size_t GpuKeySize = sizeof(GpuKey);

  int initialiseGpu(LineKeysQueryDetail &query)
  {
    if (query.gpuOk)
    {
      return 0;
    }

    LineKeysQueryDetail::GpuData &gpuData = query.gpuData;
    query.gpu = gpuDevice();

    unsigned queueFlags = 0;
//#ifdef OHM_PROFILE
//    queueFlags |= gputil::Queue::Profile;
//#endif // OHM_PROFILE
    gpuData.queue = query.gpu.createQueue(queueFlags);

    // Use the default device.
    int err = 0;
    err = initialiseLineKeysGpuProgram(query, ohm::gpuDevice());
    if (err)
    {
      return err;
    }

    // Initialise buffer to dummy size. We'll resize as required.
    gpuData.linesOut = gputil::Buffer(query.gpu, 1 * GpuKeySize, gputil::BF_ReadWriteHost);
    gpuData.linePoints = gputil::Buffer(query.gpu, 1 * sizeof(LineKeysQueryDetail::GpuData::float3), gputil::BF_ReadHost);
    query.gpuOk = (err == 0);

    return err;
  }


  bool lineKeysQueryGpu(LineKeysQueryDetail &query, bool async)
  {
    // std::cout << "Prime kernel\n" << std::flush;
    // Size the buffers.
    query.gpuData.maxKeysPerLine = 1;
    const double voxelRes = query.map->resolution();
    for (size_t i = 0; i < query.rays.size(); i += 2)
    {
      query.gpuData.maxKeysPerLine = std::max<unsigned>(
        unsigned(std::ceil((glm::length(query.rays[i + 1] - query.rays[i + 0]) / voxelRes) * std::pow(3.0, 0.5)) + 1u),
        query.gpuData.maxKeysPerLine);
    }
    // std::cout << "Worst case key requirement: " << query.gpuData.maxKeysPerLine << std::endl;
    // std::cout << "Occupancy Key size " << sizeof(OccupancyKey) << " GPU Key size: " << GpuKeySize << std::endl;

    size_t requiredSize;
    requiredSize = query.rays.size() / 2 * query.gpuData.maxKeysPerLine * GpuKeySize;
    if (query.gpuData.linesOut.size() < requiredSize)
    {
      // std::cout << "Required bytes " << requiredSize << " for " << query.rays.size() / 2u << " lines" << std::endl;
      query.gpuData.linesOut.resize(requiredSize);
    }
    requiredSize = query.rays.size() * sizeof(LineKeysQueryDetail::GpuData::float3);
    if (query.gpuData.linePoints.size() < requiredSize)
    {
      // std::cout << "linePoints size: " << requiredSize << std::endl;
      query.gpuData.linePoints.resize(requiredSize);
    }

    // Upload rays. Need to write one at a time due to precision change and size differences.
    glm::vec3 pointF;
    gputil::PinnedBuffer linePointsMem(query.gpuData.linePoints, gputil::PinWrite);
    for (size_t i = 0; i < query.rays.size(); ++i)
    {
      pointF = query.rays[i] - query.map->origin();
      linePointsMem.write(glm::value_ptr(pointF), sizeof(pointF), i * sizeof(LineKeysQueryDetail::GpuData::float3));
    }
    linePointsMem.unpin();

    // Execute.
    // std::cout << "Invoke kernel\n" << std::flush;
    int err = invokeLineKeysQueryGpu(query, query.gpuData, (async) ? &readGpuResults : nullptr);
    if (err)
    {
      return false;
    }

    query.gpuData.inflight = true;
    return true;
  }

  bool readGpuResults(LineKeysQueryDetail &query)
  {
    // std::cout << "Reading results\n" << std::flush;
    // Download results.
    gputil::PinnedBuffer gpuMem(query.gpuData.linesOut, gputil::PinRead);

    query.resultIndices.resize(query.rays.size() / 2);
    query.resultCounts.resize(query.rays.size() / 2);

    const size_t rayCount = query.rays.size() / 2;
    size_t readOffsetCount = 0;
    short resultCount = 0;
    for (size_t i = 0; i < rayCount; ++i)
    {
      // Read result count.
      gpuMem.read(&resultCount, sizeof(resultCount), readOffsetCount * GpuKeySize);

      query.resultIndices[i] = query.intersectedVoxels.size();
      query.resultCounts[i] = resultCount;

      // Read keys.
      if (resultCount)
      {
#if 1
        static_assert(sizeof(GpuKey) == sizeof(OccupancyKey), "CPU/GPU key size mismatch.");
        if (query.intersectedVoxels.capacity() < query.intersectedVoxels.size() + resultCount)
        {
          const size_t reserve = nextPow2(unsigned(query.intersectedVoxels.capacity() + resultCount));
          // const size_t reserve = (query.intersectedVoxels.capacity() + resultCount) * 2;
          // std::cout << "will reserve " << reserve << std::endl;
          query.intersectedVoxels.reserve(reserve);
        }
        query.intersectedVoxels.resize(query.intersectedVoxels.size() + resultCount);
        gpuMem.read(query.intersectedVoxels.data() + query.resultIndices[i], GpuKeySize * resultCount, (readOffsetCount + 1) * GpuKeySize);
#else  // #
        GpuKey gpuKey;
        OccupancyKey key;
        for (size_t j = 0; j < resultCount; ++j)
        {
          gpuMem.read(&gpuKey, GpuKeySize, (readOffsetCount + 1 + j) * GpuKeySize);
          key.setRegionKey(glm::i16vec3(gpuKey.region[0], gpuKey.region[1], gpuKey.region[2]));
          key.setLocalKey(glm::u8vec3(gpuKey.voxel[0], gpuKey.voxel[1], gpuKey.voxel[2]));
          query.intersectedVoxels.push_back(key);
        }
#endif // #
      }

      readOffsetCount += query.gpuData.maxKeysPerLine;
    }

    gpuMem.unpin();

    query.numberOfResults = query.rays.size() / 2;

    query.gpuData.inflight = false;
    // std::cout << "Results ready\n" << std::flush;
    return true;
  }
}

LineKeysQuery::LineKeysQuery(LineKeysQueryDetail *detail)
  : Query(detail)
{
}


LineKeysQuery::LineKeysQuery(ohm::OccupancyMap &map, unsigned queryFlags)
  : LineKeysQuery(queryFlags)
{
  setMap(&map);
}


LineKeysQuery::LineKeysQuery(unsigned queryFlags)
  : LineKeysQuery(new LineKeysQueryDetail)
{
  setQueryFlags(queryFlags);
  if (queryFlags & QF_GpuEvaluate)
  {
    LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
    initialiseGpu(*d);
  }
}


LineKeysQuery::~LineKeysQuery()
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  if (d && d->gpuOk)
  {
    releaseLineKeysGpu(*d);
  }
  delete d;
  _imp = nullptr;
}


void LineKeysQuery::setRays(const glm::dvec3 *rays, size_t pointCount)
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  d->rays.resize(pointCount);
  memcpy(d->rays.data(), rays, sizeof(*rays) * pointCount);
}


const glm::dvec3 *LineKeysQuery::rays() const
{
  const LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  return d->rays.data();
}


size_t LineKeysQuery::rayPointCount() const
{
  const LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  return d->rays.size();
}


const size_t *LineKeysQuery::resultIndices() const
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  return d->resultIndices.data();
}


const size_t *LineKeysQuery::resultCounts() const
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  return d->resultCounts.data();
}


bool LineKeysQuery::onExecute()
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);

  if ((d->queryFlags & QF_GpuEvaluate))
  {
    initialiseGpu(*d);

    if (d->gpuOk)
    {
      bool ok = lineKeysQueryGpu(*d, false);
      if (ok)
      {
        d->gpuData.queue.finish();
        ok = readGpuResults(*d);
      }
      return ok;
    }

    static bool once = false;
    if (!once)
    {
      once = true;
      std::cerr << "GPU unavailable for LineKeysQuery. Falling back to CPU\n" << std::flush;
    }
  }

  OccupancyKeyList keyList;
  d->resultIndices.resize(d->rays.size() / 2);
  d->resultCounts.resize(d->rays.size() / 2);
  for (size_t i = 0; i < d->rays.size(); i += 2)
  {
    keyList.clear();
    d->map->calculateSegmentKeys(keyList, d->rays[i + 0], d->rays[i + 1], true);
    d->resultIndices[i / 2] = d->intersectedVoxels.size();
    d->resultCounts[i / 2] = keyList.size();
    for (auto &&key : keyList)
    {
      d->intersectedVoxels.push_back(key);
    }
  }

  d->numberOfResults = d->resultIndices.size();

  return true;
}


bool LineKeysQuery::onExecuteAsync()
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);

  if ((d->queryFlags & QF_GpuEvaluate))
  {
    initialiseGpu(*d);

    if (d->gpuOk)
    {
      bool ok = lineKeysQueryGpu(*d, true);
      if (ok)
      {
        // d->gpuData.queue.flush();
        // ok = readGpuResults(*d);
      }
      return ok;
    }

    static bool once = false;
    if (!once)
    {
      once = true;
      std::cerr << "GPU unavailable for LineKeysQuery. Failing async call.\n" << std::flush;
    }
  }

  return false;
}


void LineKeysQuery::onReset(bool hardReset)
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  d->resultIndices.clear();
  d->resultCounts.clear();
}


bool LineKeysQuery::onWaitAsync(unsigned timeoutMs)
{
  LineKeysQueryDetail *d = static_cast<LineKeysQueryDetail *>(_imp);
  auto sleepInterval = std::chrono::milliseconds(0);
  auto startTime = std::chrono::system_clock::now();
  auto timeout = std::chrono::milliseconds(timeoutMs);
  while (d->gpuData.inflight)
  {
    std::this_thread::sleep_for(sleepInterval);
    if (timeoutMs != ~0u)
    {
      if (std::chrono::system_clock::now() - startTime >= timeout)
      {
        break;
      }
    }
  }

  return !d->gpuData.inflight;
}
