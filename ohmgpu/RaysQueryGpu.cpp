// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RaysQueryGpu.h"

#include "private/RaysQueryDetailGpu.h"

#include "GpuKey.h"
#include "GpuLayerCache.h"

#include <ohm/private/OccupancyMapDetail.h>

namespace ohm
{
RaysQueryGpu::RaysQueryGpu(RaysQueryDetailGpu *detail)
  : RaysQuery(detail)
{}


RaysQueryGpu::RaysQueryGpu()
  : RaysQuery(new RaysQueryDetailGpu)
{}


RaysQueryGpu::~RaysQueryGpu()
{}


bool RaysQueryGpu::onExecute()
{
  RaysQueryDetailGpu *d = imp();

  if (!d->map)
  {
    return false;
  }

  if (!onExecuteAsync())
  {
    return false;
  }

  // Sync.
  sync();

  return true;
}


bool RaysQueryGpu::onExecuteAsync()
{
  RaysQueryDetailGpu *d = imp();

  if (!d->map)
  {
    return false;
  }

  if (!(d->query_flags & kQfGpuEvaluate))
  {
    return RaysQuery::onExecute();
  }

  auto *gpu_cache = d->map->gpuCache();
  if (!gpu_cache)
  {
    // Missing GPU cache. Can't run the query.
    return false;
  }

  // This code replicates much of the GpuMap::intergrateRays() code but with a different goal.
  OccupancyMap &map = *d->map;

  if (d->rays_in.empty())
  {
    return true;
  }

  // Ensure we are using the correct GPU program.
  cacheGpuProgram();

  // Get the GPU cache.
  GpuLayerCache &layer_cache = *gpu_cache->layerCache(kGcIdOccupancy);
  d->batch_marker = layer_cache.beginBatch();

  // Region walking function tracking which regions are
  // affected by a ray.
  const auto region_func = [this](const glm::i16vec3 &region_key, const glm::dvec3 & /*origin*/,
                                  const glm::dvec3 & /*sample*/) {
    if (d->regions.find(region_key) == d->regions.end())
    {
      d->regions.insert(region_key);
    }
  };

  // Declare pinned buffers for use in upload_ray delegate.
  gputil::PinnedBuffer keys_pinned;
  gputil::PinnedBuffer rays_pinned;

  // Build region set and upload rays.
  d->regions.clear();

  std::array<gputil::float3, 2> ray_gpu;
  unsigned upload_count = 0u;

  const bool use_filter = bool(filter);

  // Take the input rays and move it into d->upload_rays. While we do so we apply the ray filter (if any). This can
  // change the origin and/or sample points.
  const double resolution = d->map->resolution();
  d->upload_rays.clear();
  RayItem ray;
  for (unsigned i = 0; i < d->rays.size(); i += 2)
  {
    ray.origin = rays[i + 0];
    ray.sample = rays[i + 1];
    ray.filter_flags = 0;

    if (use_filter)
    {
      if (!filter(&ray.origin, &ray.sample, &ray.filter_flags))
      {
        // Bad ray.
        continue;
      }
    }

    // This always adds either the full, unsegmented ray, or the last part for a segmented ray.
    ray.origin_key = map.voxelKey(ray.origin);
    ray.sample_key = map.voxelKey(ray.sample);

    d->upload_rays.emplace_back(ray);
  }

  // Reserve GPU memory for the rays.
  d->key_buffer.resize(sizeof(GpuKey) * 2 * d->upload_rays.size());
  d->ray_buffer.resize(sizeof(gputil::float3) * 2 * d->upload_rays.size());

  // Declare pinned buffers for use in upload_ray delegate.
  keys_pinned = gputil::PinnedBuffer(d->key_buffer, gputil::kPinWrite);
  rays_pinned = gputil::PinnedBuffer(d->ray_buffer, gputil::kPinWrite);

  // Upload to GPU.
  GpuKey line_start_key_gpu{};
  GpuKey line_end_key_gpu{};
  for (const RayItem &ray : d->upload_rays)
  {
    // Upload if not preloaded.
    line_start_key_gpu.region[0] = ray.origin_key.regionKey()[0];
    line_start_key_gpu.region[1] = ray.origin_key.regionKey()[1];
    line_start_key_gpu.region[2] = ray.origin_key.regionKey()[2];
    line_start_key_gpu.voxel[0] = ray.origin_key.localKey()[0];
    line_start_key_gpu.voxel[1] = ray.origin_key.localKey()[1];
    line_start_key_gpu.voxel[2] = ray.origin_key.localKey()[2];
    line_start_key_gpu.voxel[3] = 0;

    line_end_key_gpu.region[0] = ray.sample_key.regionKey()[0];
    line_end_key_gpu.region[1] = ray.sample_key.regionKey()[1];
    line_end_key_gpu.region[2] = ray.sample_key.regionKey()[2];
    line_end_key_gpu.voxel[0] = ray.sample_key.localKey()[0];
    line_end_key_gpu.voxel[1] = ray.sample_key.localKey()[1];
    line_end_key_gpu.voxel[2] = ray.sample_key.localKey()[2];
    line_end_key_gpu.voxel[3] = (ray.filter_flags & kRffClippedEnd) ? 1 : 0;

    keys_pinned.write(&line_start_key_gpu, sizeof(line_start_key_gpu), (upload_count + 0) * sizeof(GpuKey));
    keys_pinned.write(&line_end_key_gpu, sizeof(line_end_key_gpu), (upload_count + 1) * sizeof(GpuKey));

    // Localise the ray to single precision.
    // We change the ray coordinates to be relative to the end voxel centre. This assist later in voxel mean
    // calculations which are all relative to that voxel centre. Normally in CPU we have to make this adjustment
    // every time. We can avoid the adjustment via this logic.
    const glm::dvec3 end_voxel_centre = map.voxelCentreGlobal(ray.sample_key);
    ray_gpu[0].x = float(ray.origin.x - end_voxel_centre.x);
    ray_gpu[0].y = float(ray.origin.y - end_voxel_centre.y);
    ray_gpu[0].z = float(ray.origin.z - end_voxel_centre.z);
    ray_gpu[1].x = float(ray.sample.x - end_voxel_centre.x);
    ray_gpu[1].y = float(ray.sample.y - end_voxel_centre.y);
    ray_gpu[1].z = float(ray.sample.z - end_voxel_centre.z);
    rays_pinned.write(ray_gpu.data(), sizeof(ray_gpu), upload_count * sizeof(gputil::float3));
    upload_count += 2;

    // std::cout << i / 2 << ' ' << d->map->voxelKey(rays[i + 0]) << " -> " << d->map->voxelKey(rays[i + 1]) <<
    // "
    // "
    //          << ray_start << ':' << ray_end << "  <=>  " << rays[i + 0] << " -> " << rays[i + 1] << std::endl;
    // std::cout << "dirs: " << (ray_end - ray_start) << " vs " << (ray_end_d - ray_start_d) << std::endl;
    gpumap::walkRegions(*d->map, ray.origin, ray.sample, region_func);
  }

  // Asynchronous unpin. Kernels will wait on the associated event.
  keys_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &d->key_upload_event);
  rays_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &d->ray_upload_event);

  d->ray_count = unsigned(upload_count / 2);

  if (upload_count == 0)
  {
    return true;
  }

  enqueueRegions();

  // Setup the download.
  d->ranges.resize(d->ray_count);
  d->unobserved_volumes_out.resize(d->ray_count);
  d->terminal_states_out.resize(d->ray_count);

  d->ranges_buffer.read(d->ranges.data(), sizeof(*d->ranges.data()) * d->ranges.size(), 0, &gpu_cache->gpuQueue(),
                        &d->ray_upload_event, &d->ranges_event);
  d->unobserved_volumes_buffer.read(d->unobserved_volumes_out.data(),
                                    sizeof(*d->unobserved_volumes_out.data()) * d->unobserved_volumes_out.size(), 0,
                                    &gpu_cache->gpuQueue(), &d->ray_upload_event, &d->unobserved_volumes_event);
  d->terminal_states_buffer.read(d->terminal_states_out.data(),
                                 sizeof(*d->terminal_states_out.data()) * d->terminal_states_out.size(), 0,
                                 &gpu_cache->gpuQueue(), &d->ray_upload_event, &d->terminal_states_event);

  return true;
}


void RaysQueryGpu::onReset(bool hard_reset)
{
  RaysQuery::onReset(hard_reset);
  RaysQueryDetailGpu *d = imp();
  // Need to wait on the GPU program anyway.
  sync();
  d->query_event.wait();
  d->upload_rays.clear();
}


void RaysQueryGpu::cacheGpuProgram()
{}


void RaysQueryGpu::enqueueRegions()
{
  RaysQueryDetailGpu *d = imp();

  // For each region we need to enqueue the voxel data for that region. Within the GpuCache, each GpuLayerCache
  // manages the voxel data for a voxel layer and uploads into a single buffer for that layer returning an offset into
  // that buffer. For each (relevant) layer, we need to record the memory offset and upload corresponding event. These
  // need to be later fed to the update kernel. Size the region buffers.
  d->region_key_buffer.elementsResize<gputil::int3>(d->regions.size());

  d->occupancy_upload_info.offsets_buffer.elementsResize<uint64_t>(d->regions.size());
  d->occupancy_upload_info.offsets_buffer_pinned =
    gputil::PinnedBuffer(d->occupancy_upload_info.offsets_buffer, gputil::kPinWrite);

  // Execute on each region.
  gputil::PinnedBuffer regions_buffer(imp_->region_key_buffers[buffer_index], gputil::kPinWrite);

  GpuCache &gpu_cache = *this->gpuCache();
  for (const auto &region_key : d->regions)
  {
    const int try_limit = 2;
    for (int tries = 0; tries < try_limit; ++tries)
    {
      if (enqueueRegion(region_key, buffer_index))
      {
        // std::cout << "region: [" << regionKey.x << ' ' <<
        // regionKey.y << ' ' << regionKey.z << ']' <<
        // std::endl;
        gputil::int3 gpu_region_key = { region_key.x, region_key.y, region_key.z };
        regions_buffer.write(&gpu_region_key, sizeof(gpu_region_key), d->region_count * sizeof(gpu_region_key));
        ++d->region_count;
        break;
      }

      if (tries + 1 < try_limit)
      {
        // Enqueue region failed. Flush pending operations and before trying again.
        const int previous_buf_idx = buffer_index;

        regions_buffer.unpin(&gpu_cache.gpuQueue(), nullptr, &d->region_key_upload_event);
        d->occupancy_upload_info.offsets_buffer_pinned.unpin(&gpu_cache.gpuQueue(), nullptr,
                                                             &d->occupancy_upload_info.offset_upload_event);
        finaliseBatch(region_update_flags);

        // Repin these buffers, but the index has changed.
        const unsigned regions_processed = d->region_count;
        buffer_index = d->next_buffers_index;
        waitOnPreviousOperation(buffer_index);

        // Copy the rays buffer from the batch we just
        // finalised.
        gputil::copyBuffer(d->ray_buffer, d->ray_buffer, &gpu_cache.gpuQueue(), nullptr, &d->ray_upload_event);
        d->ray_count = d->ray_count;

        // This statement should always be true, but it would be bad to underflow.
        if (regions_processed < d->regions.size())
        {
          // Size the region buffers.
          d->region_key_buffer.gputil::Buffer::elementsResize<gputil::int3>(d->regions.size() - regions_processed);

          d->occupancy_upload_info.offsets_buffer.elementsResize<uint64_t>(d->regions.size() - regions_processed);
          d->occupancy_upload_info.offsets_buffer_pinned =
            gputil::PinnedBuffer(d->occupancy_upload_info.offsets_buffer, gputil::kPinWrite);

          regions_buffer = gputil::PinnedBuffer(d->region_key_buffer, gputil::kPinRead);
        }
      }
      else
      {
        // TODO(KS): throw with more information.
        std::cout << "Failed to enqueue region data" << std::endl;
      }
    }
  }

  regions_buffer.unpin(&gpu_cache.gpuQueue(), nullptr, &d->region_key_upload_event);
  d->occupancy_upload_info.offsets_buffer_pinned.unpin(&gpu_cache.gpuQueue(), nullptr,
                                                       &d->occupancy_upload_info.offset_upload_event);

  finaliseBatch();
}


void RaysQueryGpu::finaliseBatch()
{
  RaysQueryDetailGpu *d = imp();
  const int buf_idx = d->next_buffers_index;
  const OccupancyMapDetail *map = d->map->detail();

  // Complete region data upload.
  GpuCache &gpu_cache = *d->map->gpuCache();
  GpuLayerCache &occupancy_layer_cache = *gpu_cache.layerCache(kGcIdOccupancy);

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = d->region_count;
  const unsigned ray_count = d->ray_count;
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(d->query_kernel.optimalWorkGroupSize(), ray_count));
  gputil::EventList wait({ d->key_upload_event, d->ray_upload_event, d->region_key_upload_event,
                           d->occupancy_upload_info.offset_upload_event, d->occupancy_upload_info.voxel_upload_event });

  // Add voxel mean offset upload events.
  d->query_kernel(global_size, local_size, wait, d->region_update_event, &gpu_cache.gpuQueue(),
                  // Kernel args begin:
                  gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                  gputil::BufferArg<uint64_t>(d->occupancy_upload_info.offsets_buffer),
                  gputil::BufferArg<gputil::int3>(d->region_key_buffer), region_count,
                  gputil::BufferArg<GpuKey>(d->key_buffer), gputil::BufferArg<gputil::float3>(d->ray_buffer), ray_count,
                  region_dim_gpu, float(map->resolution), map->miss_value, map->hit_value,
                  map->occupancy_threshold_value, map->min_voxel_value, map->max_voxel_value, region_update_flags);

  // gpu_cache.gpuQueue().flush();

  // Update most recent chunk GPU event.
  occupancy_layer_cache.updateEvents(d->batch_marker, d->region_update_event);

  // std::cout << d->region_count << "
  // regions\n" << std::flush;

  d->region_count = 0;
  // Start a new batch for the GPU layers.
  d->batch_marker = occupancy_layer_cache.beginBatch();
}

void RaysQueryGpu::sync()
{
  RaysQueryDetailGpu *d = imp();
  std::array<gputil::Event *, 4> wait_events = { &d->query_event, &d->ranges_event, &d->unobserved_volumes_event,
                                                 &d->terminal_states_event };
  gputil::Event::wait(wait_events.data(), wait_events.size());

  // Clear the wait events.
  for (gputil::Event *event : wait_events)
  {
    *event->release();
  }
}


RaysQueryDetailGpu *RaysQueryGpu::imp()
{
  return static_cast<RaysQueryGpuDetail *>(imp_);
}


const RaysQueryDetailGpu *RaysQueryGpu::imp() const
{
  return static_cast<const RaysQueryGpuDetail *>(imp_);
}
}  // namespace ohm
