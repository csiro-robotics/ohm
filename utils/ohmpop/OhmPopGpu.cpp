// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmPopGpu.h"

#include <ohmgpu/GpuCache.h>
#include <ohmgpu/GpuMap.h>
#include <ohmgpu/GpuNdtMap.h>
#include <ohmgpu/OhmGpu.h>
#include <ohmutil/OhmUtil.h>

#include <ohmapp/DataSource.h>

#ifdef TES_ENABLE
#include <ohm/RayMapperTrace.h>
#endif  // TES_ENABLE

#include <gputil/gpuDevice.h>

// Must be after any argument type streaming operators.
#include <ohmutil/Options.h>

OhmPopGpu::GpuOptions::GpuOptions()
{
  // Build GPU options set.
  device_option_types.resize(ohm::gpuArgsInfo(nullptr, nullptr, 0));
  device_options.resize(device_option_types.size() * 2);
  ohm::gpuArgsInfo(device_options.data(), device_option_types.data(), unsigned(device_option_types.size()));
}


size_t OhmPopGpu::GpuOptions::gpuCacheSizeBytes() const
{
  return size_t(cache_size_gb * double(ohm::GpuCache::kGiB));
}


void OhmPopGpu::GpuOptions::configure(cxxopts::Options &parser)
{
  auto adder = parser.add_options("Gpu");
  // clang-format off
  adder
    ("gpu-cache-size", "Configured the GPU cache size used to cache regions for GPU update. Floating point value specified in GiB. A zero value uses half the available GPU RAM, 1GiB or 3/4 of RAM in order of preference.", optVal(cache_size_gb))
    ("gpu-ray-segment-length", "Configure the maximum allowed ray length for a single GPU thread to process. Longer rays are broken into multiple segments.", optVal(ray_segment_length));
    ;

  // clang-format on

  if (!device_options.empty())
  {
    for (size_t i = 0; i < device_options.size(); i += 2)
    {
      adder(device_options[i + 0], device_options[i + 1],
            device_option_types[i / 2] == 0 ? ::cxxopts::value<bool>() : ::cxxopts::value<std::string>());
    }
  }
}


void OhmPopGpu::GpuOptions::print(std::ostream &out)
{
  out << "Gpu cache size: " << ohm::util::Bytes(gpuCacheSizeBytes()) << '\n';
  out << "Gpu max ray segment: " << ray_segment_length << '\n';
}


OhmPopGpu::Options::Options()
{
  gpu_ = std::make_unique<GpuOptions>();
}

void OhmPopGpu::Options::configure(cxxopts::Options &parser)
{
  Super::Options::configure(parser);
  gpu_->configure(parser);
}


void OhmPopGpu::Options::print(std::ostream &out)
{
  Super::Options::print(out);
  gpu_->print(out);
}


OhmPopGpu::OhmPopGpu(std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::make_unique<Options>(), data_source)
{}


OhmPopGpu::OhmPopGpu(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::move(options), data_source)
{}


int OhmPopGpu::parseCommandLineOptions(int argc, const char *const *argv)
{
  int return_code = Super::parseCommandLineOptions(argc, argv);
  if (return_code)
  {
    return return_code;
  }

  return_code = ohm::configureGpuFromArgs(argc, argv);

  // Handle GPU cache auto sizing.
  auto opt_gpu = options().gpu();
  if (opt_gpu.cache_size_gb <= 0)
  {
    // Calculate the GPU cache size as half GPU RAM size or 1GiB - whichever is larger. 1 GiB is too large use
    // 3/4 of the GPU RAM.
    const uint64_t total_device_memory = ohm::gpuDevice().deviceMemory();
    // Seed with 3/4 of device memory.
    uint64_t target_gpu_cache_size = (total_device_memory * 3) / 4;
    // If this is more than 1GiB, use either 1GiB or half device memory.
    if (target_gpu_cache_size > ohm::GpuCache::kGiB)
    {
      target_gpu_cache_size = std::max<uint64_t>(total_device_memory / 2, ohm::GpuCache::kGiB);
    }
    opt_gpu.cache_size_gb = double(target_gpu_cache_size) / double(ohm::GpuCache::kGiB);
    // Cap the max auto allocation to 4GiB. Some embedded systems with unified RAM can try for very large cache size
    // which exceeds the maximum allowed allocation size. While GpuLayerCache caps this, we limit the auto selected size
    // to help avoid giving a false impression of having a very large amount of GPU memory allocated.
    opt_gpu.cache_size_gb = std::min(opt_gpu.cache_size_gb, 4.0);
    std::cout << "Auto select GPU cache upper bound: " << ohm::util::Bytes(opt_gpu.gpuCacheSizeBytes()) << std::endl;
  }

  return return_code;
}


ohm::GpuMap *OhmPopGpu::gpuMap()
{
  if (mapper_)
  {
    return static_cast<ohm::GpuMap *>(true_mapper_.get());
  }
  return nullptr;
}


const ohm::GpuMap *OhmPopGpu::gpuMap() const
{
  if (mapper_)
  {
    return static_cast<const ohm::GpuMap *>(true_mapper_.get());
  }
  return nullptr;
}


int OhmPopGpu::prepareForRun()
{
  int return_code = Super::prepareForRun();
  if (return_code)
  {
    return return_code;
  }

  // Will use GpuNdtMap instead.
  ndt_map_.release();

  const size_t gpu_cache_size = options().gpu().gpuCacheSizeBytes();
  unsigned reserve_batch_size = dataSource()->expectedBatchSize();
  if (!reserve_batch_size)
  {
    reserve_batch_size = defaultBatchSize();
  }
  true_mapper_ = std::unique_ptr<ohm::RayMapper>(
    (options().ndt().mode == ohm::NdtMode::kNone) ?
      new ohm::GpuMap(map_.get(), true, reserve_batch_size, gpu_cache_size) :
      new ohm::GpuNdtMap(map_.get(), true, reserve_batch_size, gpu_cache_size, options().ndt().mode));
  ohm::GpuMap *gpu_map = gpuMap();
  gpu_map->setRaySegmentLength(options().gpu().ray_segment_length);

  if (bool(options().ndt().mode))
  {
    ohm::NdtMap *ndt_map = &static_cast<ohm::GpuNdtMap *>(gpu_map)->ndtMap();
    ndt_map->setAdaptationRate(options().ndt().adaptation_rate);
    ndt_map->setSensorNoise(options().ndt().sensor_noise);
    ndt_map->setReinitialiseCovarianceThreshold(ohm::probabilityToValue(options().ndt().covariance_reset_probability));
    ndt_map->setReinitialiseCovariancePointCount(options().ndt().covariance_reset_sample_count);
  }

  // Ensure options reflect map flags.
  options().map().voxel_mean = map_->voxelMeanEnabled();
  options().map().traversal = map_->traversalEnabled();

  mapper_ = true_mapper_.get();
#ifdef TES_ENABLE
  if (!options().output().trace.empty() && !options().output().trace_final)
  {
    trace_mapper_ = std::make_unique<ohm::RayMapperTrace>(map_.get(), true_mapper_.get());
    mapper_ = trace_mapper_.get();
  }
#endif  // TES_ENABLE

  return return_code;
}


void OhmPopGpu::finaliseMap()
{
  if (auto *gpu_map = gpuMap())
  {
    ohm::logger::info("syncing GPU voxels\n");
    gpu_map->syncVoxels();
  }
  Super::finaliseMap();
}
