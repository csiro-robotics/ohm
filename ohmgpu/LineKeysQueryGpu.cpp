// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "LineKeysQueryGpu.h"

#include "private/LineKeysQueryDetailGpu.h"

#include "OhmGpu.h"

#include "private/GpuProgramRef.h"

#include <ohm/KeyList.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>

#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "LineKeysResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(calculateLines);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

using namespace ohm;

namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref("LineKeys", GpuProgramRef::kSourceString, LineKeysCode, LineKeysCode_length);
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref("LineKeys", GpuProgramRef::kSourceFile, "LineKeys.cl");
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

  bool readGpuResults(LineKeysQueryDetailGpu &query);

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

  // TODO(KS): Verify alignment.
  const size_t kGpuKeySize = sizeof(GpuKey);

  bool initialiseGpu(LineKeysQueryDetailGpu &query)
  {
    if (query.gpu_ok)
    {
      return true;
    }

    query.gpu = gpuDevice();

    unsigned queue_flags = 0;
    //#ifdef OHM_PROFILE
    //    queueFlags |= gputil::Queue::Profile;
    //#endif // OHM_PROFILE
    query.queue = query.gpu.createQueue(queue_flags);

    if (!program_ref.addReference(query.gpu))
    {
      return false;
    }

    query.line_keys_kernel = GPUTIL_MAKE_KERNEL(program_ref.program(), calculateLines);
    query.line_keys_kernel.calculateOptimalWorkGroupSize();

    if (!query.line_keys_kernel.isValid())
    {
      return false;
    }

    // Initialise buffer to dummy size. We'll resize as required.
    query.lines_out = gputil::Buffer(query.gpu, 1 * kGpuKeySize, gputil::kBfReadWriteHost);
    query.line_points = gputil::Buffer(query.gpu, 1 * sizeof(gputil::float3), gputil::kBfReadHost);
    query.gpu_ok = true;

    return true;
  }


  bool lineKeysQueryGpu(LineKeysQueryDetailGpu &query, bool /*async*/)
  {
    // std::cout << "Prime kernel\n" << std::flush;
    // Size the buffers.
    query.max_keys_per_line = 1;
    const double voxel_res = query.map->resolution();
    for (size_t i = 0; i < query.rays.size(); i += 2)
    {
      query.max_keys_per_line = std::max<unsigned>(
        unsigned(std::ceil((glm::length(query.rays[i + 1] - query.rays[i + 0]) / voxel_res) * std::pow(3.0, 0.5)) + 1u),
        query.max_keys_per_line);
    }
    // std::cout << "Worst case key requirement: " << query.max_keys_per_line << std::endl;
    // std::cout << "Occupancy Key size " << sizeof(Key) << " GPU Key size: " << GpuKeySize << std::endl;

    size_t required_size = query.rays.size() / 2 * query.max_keys_per_line * kGpuKeySize;
    if (query.lines_out.size() < required_size)
    {
      // std::cout << "Required bytes " << requiredSize << " for " << query.rays.size() / 2u << " lines" << std::endl;
      query.lines_out.resize(required_size);
    }
    required_size = query.rays.size() * sizeof(gputil::float3);
    if (query.line_points.size() < required_size)
    {
      // std::cout << "line_points size: " << requiredSize << std::endl;
      query.line_points.resize(required_size);
    }

    // Upload rays. Need to write one at a time due to precision change and size differences.
    glm::vec3 point_f;
    gputil::PinnedBuffer line_points_mem(query.line_points, gputil::kPinWrite);
    for (size_t i = 0; i < query.rays.size(); ++i)
    {
      point_f = query.rays[i] - query.map->origin();
      line_points_mem.write(glm::value_ptr(point_f), sizeof(point_f), i * sizeof(gputil::float3));
    }
    line_points_mem.unpin();

    // Execute.
    const gputil::int3 region_dim = { query.map->regionVoxelDimensions().x, query.map->regionVoxelDimensions().y,
                                      query.map->regionVoxelDimensions().z };

    // std::cout << "Invoke kernel\n" << std::flush;
    gputil::Dim3 global_size(query.rays.size() / 2);
    gputil::Dim3 local_size(std::min<size_t>(query.line_keys_kernel.optimalWorkGroupSize(), query.rays.size() / 2));

    // Ensure all memory transfers have completed.
    query.queue.insertBarrier();
    int err = query.line_keys_kernel(global_size, local_size, &query.queue,
                                     // Kernel args
                                     gputil::BufferArg<GpuKey>(query.lines_out), query.max_keys_per_line,
                                     gputil::BufferArg<gputil::float3>(query.line_points),
                                     gputil::uint(query.rays.size() / 2), region_dim, float(query.map->resolution()));

    if (err)
    {
      return false;
    }

    query.inflight = true;
    return true;
  }

  bool readGpuResults(LineKeysQueryDetailGpu &query)
  {
    // std::cout << "Reading results\n" << std::flush;
    // Download results.
    gputil::PinnedBuffer gpu_mem(query.lines_out, gputil::kPinRead);

    query.result_indices.resize(query.rays.size() / 2);
    query.result_counts.resize(query.rays.size() / 2);

    const size_t ray_count = query.rays.size() / 2;
    size_t read_offset_count = 0;
    short result_count = 0;
    for (size_t i = 0; i < ray_count; ++i)
    {
      // Read result count.
      gpu_mem.read(&result_count, sizeof(result_count), read_offset_count * kGpuKeySize);

      query.result_indices[i] = query.intersected_voxels.size();
      query.result_counts[i] = result_count;

      // Read keys.
      if (result_count)
      {
#if 1
        static_assert(sizeof(GpuKey) == sizeof(Key), "CPU/GPU key size mismatch.");
        if (query.intersected_voxels.capacity() < query.intersected_voxels.size() + result_count)
        {
          const size_t reserve = nextPow2(unsigned(query.intersected_voxels.capacity() + result_count));
          // const size_t reserve = (query.intersected_voxels.capacity() + resultCount) * 2;
          // std::cout << "will reserve " << reserve << std::endl;
          query.intersected_voxels.reserve(reserve);
        }
        query.intersected_voxels.resize(query.intersected_voxels.size() + result_count);
        gpu_mem.read(query.intersected_voxels.data() + query.result_indices[i], kGpuKeySize * result_count,
                     (read_offset_count + 1) * kGpuKeySize);
#else   // #
        GpuKey gpuKey;
        Key key;
        for (size_t j = 0; j < resultCount; ++j)
        {
          gpuMem.read(&gpuKey, GpuKeySize, (readOffsetCount + 1 + j) * GpuKeySize);
          key.setRegionKey(glm::i16vec3(gpuKey.region[0], gpuKey.region[1], gpuKey.region[2]));
          key.setLocalKey(glm::u8vec3(gpuKey.voxel[0], gpuKey.voxel[1], gpuKey.voxel[2]));
          query.intersected_voxels.push_back(key);
        }
#endif  // #
      }

      read_offset_count += query.max_keys_per_line;
    }

    gpu_mem.unpin();

    query.number_of_results = query.rays.size() / 2;

    query.inflight = false;
    // std::cout << "Results ready\n" << std::flush;
    return true;
  }
}  // namespace

LineKeysQueryGpu::LineKeysQueryGpu(LineKeysQueryDetailGpu *detail)
  : LineKeysQuery(detail)
{}


LineKeysQueryGpu::LineKeysQueryGpu(ohm::OccupancyMap &map, unsigned query_flags)
  : LineKeysQueryGpu(query_flags)
{
  setMap(&map);
}


LineKeysQueryGpu::LineKeysQueryGpu(unsigned query_flags)
  : LineKeysQuery(new LineKeysQueryDetailGpu)
{
  setQueryFlags(query_flags);
  LineKeysQueryDetailGpu *d = static_cast<LineKeysQueryDetailGpu *>(imp_);
  initialiseGpu(*d);
}


LineKeysQueryGpu::~LineKeysQueryGpu()
{
  LineKeysQueryDetailGpu *d = static_cast<LineKeysQueryDetailGpu *>(imp_);
  if (d && d->gpu_ok)
  {
    if (d->line_keys_kernel.isValid())
    {
      d->line_keys_kernel = gputil::Kernel();
      program_ref.releaseReference();
    }
  }
  delete d;
  imp_ = nullptr;
}


bool LineKeysQueryGpu::onExecute()
{
  LineKeysQueryDetailGpu *d = static_cast<LineKeysQueryDetailGpu *>(imp_);

  if (!(d->query_flags & kQfGpuEvaluate))
  {
    return LineKeysQuery::onExecute();
  }

  initialiseGpu(*d);

  if (d->gpu_ok)
  {
    bool ok = lineKeysQueryGpu(*d, false);
    if (ok)
    {
      d->queue.finish();
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

  KeyList key_list;
  d->result_indices.resize(d->rays.size() / 2);
  d->result_counts.resize(d->rays.size() / 2);
  for (size_t i = 0; i < d->rays.size(); i += 2)
  {
    key_list.clear();
    d->map->calculateSegmentKeys(key_list, d->rays[i + 0], d->rays[i + 1], true);
    d->result_indices[i / 2] = d->intersected_voxels.size();
    d->result_counts[i / 2] = key_list.size();
    for (auto &&key : key_list)
    {
      d->intersected_voxels.push_back(key);
    }
  }

  d->number_of_results = d->result_indices.size();

  return true;
}


bool LineKeysQueryGpu::onExecuteAsync()
{
  LineKeysQueryDetailGpu *d = static_cast<LineKeysQueryDetailGpu *>(imp_);

  if ((d->query_flags & kQfGpuEvaluate))
  {
    initialiseGpu(*d);

    if (d->gpu_ok)
    {
      bool ok = lineKeysQueryGpu(*d, true);
      if (ok)
      {
        // d->queue.flush();
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


void LineKeysQueryGpu::onReset(bool /*hard_reset*/)
{
  LineKeysQueryDetailGpu *d = static_cast<LineKeysQueryDetailGpu *>(imp_);
  d->result_indices.clear();
  d->result_counts.clear();
}


bool LineKeysQueryGpu::onWaitAsync(unsigned timeout_ms)
{
  LineKeysQueryDetailGpu *d = static_cast<LineKeysQueryDetailGpu *>(imp_);
  const auto sleep_interval = std::chrono::milliseconds(0);
  const auto start_time = std::chrono::system_clock::now();
  auto timeout = std::chrono::milliseconds(timeout_ms);
  while (d->inflight)
  {
    std::this_thread::sleep_for(sleep_interval);
    if (timeout_ms != ~0u)
    {
      if (std::chrono::system_clock::now() - start_time >= timeout)
      {
        break;
      }
    }
  }

  return !d->inflight;
}


LineKeysQueryDetailGpu *LineKeysQueryGpu::imp()
{
  return static_cast<LineKeysQueryDetailGpu *>(imp_);
}


const LineKeysQueryDetailGpu *LineKeysQueryGpu::imp() const
{
  return static_cast<const LineKeysQueryDetailGpu *>(imp_);
}
