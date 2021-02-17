//
// author Kazys Stepanas
//
#include "OhmPopConfig.h"

#include <glm/glm.hpp>

#include <slamio/SlamCloudLoader.h>

#include <ohm/MapSerialise.h>
#include <ohm/Mapper.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/RayMapperNdt.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/RayMapperTrace.h>
#include <ohm/Trace.h>
#include <ohm/VoxelData.h>

#ifndef OHMPOP_CPU
#include <ohmgpu/ClearanceProcess.h>
#include <ohmgpu/GpuMap.h>
#include <ohmgpu/GpuNdtMap.h>
#include <ohmgpu/OhmGpu.h>
#endif  // OHMPOP_CPU

#include <ohmtools/OhmCloud.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/SafeIO.h>
#include <ohmutil/ScopedTimeDisplay.h>

#include <ohmutil/Options.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <locale>
#include <sstream>
#include <thread>

namespace
{
using Clock = std::chrono::high_resolution_clock;

int g_quit = 0;

void onSignal(int arg)
{
  if (arg == SIGINT || arg == SIGTERM)
  {
    ++g_quit;
  }
}

struct Options
{
  struct Ndt
  {
    float prob_hit = 0.0;                       // re-initialised from a default map
    float prob_miss = 0.0;                      // re-initialised from a default map
    float adaptation_rate = 0.0f;               // re-initialised from a default map
    float sensor_noise = 0.0f;                  // re-initialised from a default map
    float covariance_reset_probability = 0.0f;  // re-initialised from a default map
    // NDT map probabilities should be much narrower. The NDT process is more precise.
    unsigned covariance_reset_sample_count = 0;  // re-initialised from a default map
    bool enabled = false;
    bool ndt_tm = false;
  };

  std::string cloud_file;
  std::string trajectory_file;
  std::string output_base_name;
  std::string prior_map;
#ifdef TES_ENABLE
  std::string trace;
  bool trace_final;
#endif  // TES_ENABLE
  glm::dvec3 sensor_offset = glm::dvec3(0.0);
  glm::u8vec3 region_voxel_dim = glm::u8vec3(0);  // re-initialised from a default map
  uint64_t point_limit = 0;
  int64_t preload_count = 0;
  double start_time = 0;
  double time_limit = 0;
  double resolution = 0.1;
  double clip_near_range = 0.0;
  float prob_hit = 0.0f;                   // re-initialised from a default map
  float prob_miss = 0.0f;                  // re-initialised from a default map
  float prob_thresh = 0.5f;                // re-initialised from a default map
  glm::vec2 prob_range = glm::vec2(0, 0);  // re-initialised from a default map
  glm::vec3 cloud_colour = glm::vec3(0);
  unsigned batch_size = 4096;  // NOLINT(readability-magic-numbers)
  /// String value for the "--mode" argument. This sets the value of @c ray_mode_flags - see that member.
  std::string mode = "normal";
  /// @c ohm::RayFlag selection based on the "--mode" argument which is mapped into the @c mode member.
  ///
  /// Supported modes:
  /// - "normal" (default) => @c ohm::kRfDefault
  /// - "sample" (default) => @c ohm::kRfExcludeRay
  /// - "erode" (default) => @c ohm::kRfExcludeSample
  unsigned ray_mode_flags = ohm::kRfDefault;
  bool serialise = true;
  bool save_info = false;
  bool voxel_mean = false;
  bool uncompressed = false;
#ifndef OHMPOP_CPU
  double mapping_interval = 0.2;  // NOLINT(readability-magic-numbers)
  double progressive_mapping_slice = 0.0;
  float clearance = 0.0f;
  bool post_population_mapping = true;
  bool clearance_unknown_as_occupied = false;
#endif  // OHMPOP_CPU
  bool quiet = false;

  Ndt ndt;

  Options();

  void print(std::ostream **out, const ohm::OccupancyMap &map) const;
};


Options::Options()
{
  // Initialise defaults from map configurations.
  ohm::OccupancyMap defaults_map;

  region_voxel_dim = defaults_map.regionVoxelDimensions();
  prob_hit = defaults_map.hitProbability();
  prob_miss = defaults_map.missProbability();
  prob_thresh = defaults_map.occupancyThresholdProbability();
  prob_range[0] = defaults_map.minVoxelValue();
  prob_range[1] = defaults_map.maxVoxelValue();

  const ohm::NdtMap defaults_ndt(&defaults_map, false, true);
  // Default probabilities may differ for NDT.
  ndt.prob_hit = defaults_map.hitProbability();
  ndt.prob_miss = defaults_map.missProbability();
  ndt.adaptation_rate = defaults_ndt.adaptationRate();
  ndt.sensor_noise = defaults_ndt.sensorNoise();
  ndt.covariance_reset_probability = ohm::valueToProbability(defaults_ndt.reinitialiseCovarianceThreshold());
  ndt.covariance_reset_sample_count = defaults_ndt.reinitialiseCovariancePointCount();
  ndt.adaptation_rate = defaults_ndt.adaptationRate();
}


void Options::print(std::ostream **out, const ohm::OccupancyMap &map) const
{
  while (*out)
  {
    **out << "Cloud: " << cloud_file;
    if (!trajectory_file.empty())
    {
      **out << " + " << trajectory_file << '\n';
    }
    else
    {
      **out << " (no trajectory)\n";
    }
    if (preload_count)
    {
      **out << "Preload: ";
      if (preload_count < 0)
      {
        **out << "all";
      }
      else
      {
        **out << preload_count;
      }
      **out << '\n';
    }

    if (point_limit)
    {
      **out << "Maximum point: " << point_limit << '\n';
    }

    if (start_time > 0)
    {
      **out << "Process from timestamp: " << start_time << '\n';
    }

    if (time_limit > 0)
    {
      **out << "Process to timestamp: " << time_limit << '\n';
    }

    // std::string mem_size_string;
    // util::makeMemoryDisplayString(mem_size_string, ohm::OccupancyMap::voxelMemoryPerRegion(region_voxel_dim));
    **out << "Map resolution: " << resolution << '\n';
    **out << "Mapping mode: " << mode << '\n';
    **out << "Voxel mean position: " << (map.voxelMeanEnabled() ? "on" : "off") << '\n';
    **out << "Compressed: " << ((map.flags() & ohm::MapFlag::kCompressed) == ohm::MapFlag::kCompressed ? "on" : "off")
          << '\n';
    glm::i16vec3 region_dim = region_voxel_dim;
    region_dim.x = (region_dim.x) ? region_dim.x : OHM_DEFAULT_CHUNK_DIM_X;
    region_dim.y = (region_dim.y) ? region_dim.y : OHM_DEFAULT_CHUNK_DIM_Y;
    region_dim.z = (region_dim.z) ? region_dim.z : OHM_DEFAULT_CHUNK_DIM_Z;
    **out << "Map region dimensions: " << region_dim << '\n';
    // **out << "Map region memory: " << mem_size_string << '\n';
    **out << "Hit probability: " << prob_hit << " (" << map.hitValue() << ")\n";
    **out << "Miss probability: " << prob_miss << " (" << map.missValue() << ")\n";
    **out << "Probability range: [" << map.minVoxelProbability() << ' ' << map.maxVoxelProbability() << "]\n";
    **out << "Value range      : [" << map.minVoxelValue() << ' ' << map.maxVoxelValue() << "]\n";
    if (ndt.enabled)
    {
      **out << "NDT map enabled:" << '\n';
      **out << "NDT adaptation rate: " << ndt.adaptation_rate << '\n';
      **out << "NDT sensor noise: " << ndt.sensor_noise << '\n';
      **out << "NDT covariance reset probability: " << ndt.covariance_reset_probability << '\n';
      **out << "NDT covariance reset sample cout: " << ndt.covariance_reset_sample_count << '\n';
      **out << "NDT-TM: " << (ndt.ndt_tm ? "true" : "false") << '\n';
    }
#ifndef OHMPOP_CPU
    **out << "Ray batch size: " << batch_size << '\n';
    **out << "Clearance mapping: ";
    if (clearance > 0)
    {
      **out << clearance << "m range\n";
      **out << "Unknown as occupied: " << (clearance_unknown_as_occupied ? "on" : "off") << '\n';
    }
    else
    {
      **out << "disabled\n";
    }

    **out << "Mapping mode: ";
    if (progressive_mapping_slice > 0)
    {
      **out << "progressive time slice " << progressive_mapping_slice << "s\n";
      **out << "Mapping interval: " << mapping_interval << "s\n";
      **out << "Post population mapping: " << (post_population_mapping ? "on" : "off") << '\n';
    }
    else
    {
      **out << "post" << '\n';
    }
#endif  // OHMPOP_CPU

#ifdef TES_ENABLE
    if (!trace.empty())
    {
      **out << "3es trace file: " << trace << (trace_final ? "(final only)\n" : "\n");
    }
#endif  // TES_ENABLE

    **out << std::flush;

    ++out;
  }
}

class SerialiseMapProgress : public ohm::SerialiseProgress
{
public:
  explicit SerialiseMapProgress(ProgressMonitor &monitor)
    : monitor_(monitor)
  {}

  bool quit() const override { return ::g_quit > 1; }

  void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
  void incrementProgress(unsigned inc) override { monitor_.incrementProgressBy(inc); }

private:
  ProgressMonitor &monitor_;
};


enum SaveFlags : unsigned
{
  kSaveMap = (1u << 0u),
  kSaveCloud = (1u << 1u),
  // SaveClearanceCloud = (u1 << 2u),
};

void saveMap(const Options &opt, const ohm::OccupancyMap &map, const std::string &base_name, ProgressMonitor *prog,
             unsigned save_flags = kSaveMap)
{
  std::unique_ptr<SerialiseMapProgress> save_progress(prog ? new SerialiseMapProgress(*prog) : nullptr);

  if (g_quit >= 2)
  {
    return;
  }

  if (save_flags & kSaveMap)
  {
    std::string output_file = base_name + ".ohm";
    std::cout << "Saving map to " << output_file.c_str() << std::endl;

    if (prog)
    {
      prog->unpause();
    }

    int err = ohm::save(output_file.c_str(), map, save_progress.get());

    if (prog)
    {
      prog->endProgress();
      if (!opt.quiet)
      {
        std::cout << std::endl;
      }
    }

    if (err)
    {
      std::cerr << "Failed to save map: " << err << std::endl;
    }
  }

  if (save_flags & kSaveCloud)
  {
    // Save a cloud representation.
    std::cout << "Converting to point cloud." << std::endl;

    ohmtools::ProgressCallback save_progress_callback;
    ohmtools::ColourByHeight colour_by_height(map);
    ohmtools::SaveCloudOptions save_opt;
    save_opt.colour_select = [&colour_by_height](const ohm::Voxel<const float> &occupancy) {
      return colour_by_height.select(occupancy);
    };

    if (prog)
    {
      prog->beginProgress(ProgressMonitor::Info(map.regionCount()));
      save_progress_callback = [prog](size_t progress, size_t /*target*/) { prog->updateProgress(progress); };
    }

    std::string output_file = base_name + ".ply";
    // Ensure we don't overwrite the input data file.
    if (output_file == opt.cloud_file)
    {
      output_file = base_name + "-points.ply";
    }
    std::cout << "Saving point cloud to " << output_file.c_str() << std::endl;
    uint64_t point_count = saveCloud(output_file.c_str(), map, save_opt, save_progress_callback);

    if (prog)
    {
      prog->endProgress();
      prog->pause();
    }
    if (!opt.quiet)
    {
      std::cout << "\nExported " << point_count << " point(s)" << std::endl;
    }
  }
}

std::string getFileExtension(const std::string &file)
{
  const size_t last_dot = file.find_last_of('.');
  if (last_dot != std::string::npos)
  {
    return file.substr(last_dot + 1);
  }

  return "";
}
}  // namespace


int populateMap(const Options &opt)
{
  ohm::ScopedTimeDisplay time_display("Execution time");
  if (opt.quiet)
  {
    time_display.disable();
  }

  std::cout << "Loading points from " << opt.cloud_file << " with trajectory " << opt.trajectory_file << std::endl;

  SlamCloudLoader loader;
  if (!loader.open(opt.cloud_file.c_str(), opt.trajectory_file.c_str()))
  {
    fprintf(stderr, "Error loading cloud %s with trajectory %s \n", opt.cloud_file.c_str(),
            opt.trajectory_file.c_str());
    return -2;
  }

  ohm::MapFlag map_flags = ohm::MapFlag::kDefault;
  map_flags |= (opt.voxel_mean) ? ohm::MapFlag::kVoxelMean : ohm::MapFlag::kNone;
  map_flags &= (opt.uncompressed) ? ~ohm::MapFlag::kCompressed : ~ohm::MapFlag::kNone;
  ohm::OccupancyMap map(opt.resolution, opt.region_voxel_dim, map_flags);
#ifdef OHMPOP_CPU
  std::unique_ptr<ohm::NdtMap> ndt_map;
  if (opt.ndt.enabled)
  {
    ndt_map = std::make_unique<ohm::NdtMap>(&map, opt.ndt.ndt_tm, true);
  }
#else   // OHMPOP_CPU
  std::unique_ptr<ohm::GpuMap> gpu_map((!opt.ndt.enabled) ?
                                         new ohm::GpuMap(&map, true, opt.batch_size) :
                                         new ohm::GpuNdtMap(&map, true, opt.batch_size));
  ohm::NdtMap *ndt_map = &static_cast<ohm::GpuNdtMap *>(gpu_map.get())->ndtMap();
#endif  // OHMPOP_CPU

  if (opt.voxel_mean)
  {
    map.addVoxelMeanLayer();
  }

  if (opt.ndt.enabled)
  {
    ndt_map->setAdaptationRate(opt.ndt.adaptation_rate);
    ndt_map->setSensorNoise(opt.ndt.sensor_noise);
    ndt_map->setReinitialiseCovarianceThreshold(ohm::probabilityToValue(opt.ndt.covariance_reset_probability));
    ndt_map->setReinitialiseCovariancePointCount(opt.ndt.covariance_reset_sample_count);
  }

  std::atomic<uint64_t> elapsed_ms(0);
  ProgressMonitor prog(10);

  prog.setDisplayFunction([&elapsed_ms, &opt](const ProgressMonitor::Progress &prog) {
    if (!opt.quiet)
    {
      const uint64_t elapsed_ms_local = elapsed_ms;
      const uint64_t sec = elapsed_ms_local / 1000u;
      const auto ms = unsigned(elapsed_ms_local - sec * 1000);

      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (!prog.info.info.empty())
      {
        out << prog.info.info << " : ";
      }

      out << sec << '.' << std::setfill('0') << std::setw(3) << ms << "s : ";

      const auto fill_width = std::numeric_limits<decltype(prog.progress)>::digits10;
      out << std::setfill(' ') << std::setw(fill_width) << prog.progress;
      if (prog.info.total)
      {
        out << " / " << std::setfill(' ') << std::setw(fill_width) << prog.info.total;
      }
      out << "    ";
      std::cout << out.str() << std::flush;
    }
  });

  if (!opt.prior_map.empty())
  {
    std::cout << "Loading prior map " << opt.prior_map << std::endl;
    SerialiseMapProgress load_progress(prog);
    int load_err = ohm::load(opt.prior_map.c_str(), map, &load_progress);
    if (load_err)
    {
      std::cerr << "Error(" << load_err << ") loading prior map " << opt.prior_map << " : "
                << ohm::errorCodeString(load_err) << std::endl;
      return -3;
    }
  }

#ifdef TES_ENABLE
  std::unique_ptr<ohm::RayMapperTrace> trace_mapper;
#endif  // TES_ENABLE

  ohm::RayMapper *ray_mapper = nullptr;
#ifdef OHMPOP_CPU
  std::unique_ptr<ohm::RayMapperNdt> ndt_ray_mapper;
  ohm::RayMapperOccupancy ray_mapper2(&map);
  if (opt.ndt.enabled)
  {
    std::cout << "Building NDT map" << std::endl;
    ndt_ray_mapper = std::make_unique<ohm::RayMapperNdt>(ndt_map.get(), opt.ndt.ndt_tm);
    ray_mapper = ndt_ray_mapper.get();
  }
  else
  {
    ray_mapper = &ray_mapper2;
  }
#else   // OHMPOP_CPU
  ray_mapper = gpu_map.get();
#endif  // OHMPOP_CPU

#ifdef TES_ENABLE
  if (!opt.trace.empty() && !opt.trace_final)
  {
    trace_mapper = std::make_unique<ohm::RayMapperTrace>(&map, ray_mapper);
    ray_mapper = trace_mapper.get();
  }
#endif  // TES_ENABLE

  ohm::Mapper mapper(&map);
  std::vector<double> sample_timestamps;
  std::vector<glm::dvec3> origin_sample_pairs;
  glm::dvec3 origin;
  glm::dvec3 sample;
  glm::dvec3 last_batch_origin(0);
  float intensity;
  std::vector<float> intensities;
  // glm::vec3 voxel, ext(opt.resolution);
  double timestamp;
  uint64_t point_count = 0;
  // Update map visualisation every N samples.
  const size_t ray_batch_size = opt.batch_size;
  double timebase = -1;
  double first_timestamp = -1;
  double last_timestamp = -1;
  double accumulated_motion = 0;
  double delta_motion = 0;
  bool warned_no_motion = false;
#ifndef OHMPOP_CPU
  double next_mapper_update = opt.mapping_interval;
#endif  // OHMPOP_CPU
  Clock::time_point start_time;
  Clock::time_point end_time;

#ifndef OHMPOP_CPU
  if (!gpu_map->gpuOk())
  {
    std::cerr << "Failed to initialise GpuMap programs." << std::endl;
    return -3;
  }
#endif  // OHMPOP_CPU

  if (opt.clip_near_range > 0)
  {
    std::cout << "Filtering samples closer than: " << opt.clip_near_range << std::endl;
    // Install a self-strike removing clipping box.
    map.setRayFilter([&opt](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) -> bool {
      // Range filter.
      if (!ohm::goodRayFilter(start, end, filter_flags, 1e3))
      {
        return false;
      }

      const glm::dvec3 ray = *end - *start;
      if (glm::dot(ray, ray) < opt.clip_near_range * opt.clip_near_range)
      {
        // Too close.
        *filter_flags |= ohm::kRffClippedEnd;
      }

      return true;
    });
  }

  map.setHitProbability(opt.prob_hit);
  map.setOccupancyThresholdProbability(opt.prob_thresh);
  map.setMissProbability(opt.prob_miss);
  if (opt.prob_range[0] || opt.prob_range[1])
  {
    map.setMinVoxelValue(opt.prob_range[0]);
    map.setMaxVoxelValue(opt.prob_range[1]);
  }
  // map.setSaturateAtMinValue(opt.saturateMin);
  // map.setSaturateAtMaxValue(opt.saturateMax);

  // Prevent ready saturation to free.
  // map.setClampingThresMin(0.01);
  // printf("min: %g\n", map.getClampingThresMinLog());

#ifndef OHMPOP_CPU
  if (opt.clearance > 0)
  {
    unsigned clearance_flags = ohm::kQfGpuEvaluate;
    if (opt.clearance_unknown_as_occupied)
    {
      clearance_flags |= ohm::kQfUnknownAsOccupied;
    }
    mapper.addProcess(new ohm::ClearanceProcess(opt.clearance, clearance_flags));
  }
#endif  // OHMPOP_CPU

  std::array<std::ostream *, 3> streams = { &std::cout, nullptr, nullptr };
  std::ofstream info_stream;
  if (opt.save_info)
  {
    streams[1] = &info_stream;
    std::string output_file = opt.output_base_name + ".txt";
    std::ofstream out(output_file.c_str());
    info_stream.open(output_file.c_str());
  }

  opt.print(streams.data(), map);

  if (opt.preload_count)
  {
    int64_t preload_count = opt.preload_count;
    if (preload_count < 0 && opt.point_limit)
    {
      preload_count = opt.point_limit;
    }

    std::cout << "Preloading points";

    start_time = Clock::now();
    if (preload_count < 0)
    {
      std::cout << std::endl;
      loader.preload();
    }
    else
    {
      std::cout << " " << preload_count << std::endl;
      loader.preload(preload_count);
    }
    end_time = Clock::now();
    const double preload_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;
    std::cout << "Preload completed over " << preload_time << " seconds." << std::endl;
  }

  start_time = Clock::now();
  std::cout << "Populating map" << std::endl;

  prog.beginProgress(ProgressMonitor::Info((point_count && timebase == 0) ?
                                             std::min<uint64_t>(point_count, loader.numberOfPoints()) :
                                             loader.numberOfPoints()));
  prog.startThread();

  //------------------------------------
  // Population loop.
  //------------------------------------
  // mapper.start();
  origin = glm::vec3(0, 0, 0);
  while ((point_count < opt.point_limit || opt.point_limit == 0) &&
         (last_timestamp - timebase < opt.time_limit || opt.time_limit == 0) &&
         loader.nextPoint(sample, intensity, &origin, &timestamp))
  {
    if (timebase < 0)
    {
      timebase = timestamp;
    }

    if (timestamp - timebase < opt.start_time)
    {
      continue;
    }

    ++point_count;
    sample_timestamps.push_back(timestamp);
    origin_sample_pairs.push_back(origin);
    origin_sample_pairs.push_back(sample);
    intensities.push_back(intensity);

    if (last_timestamp < 0)
    {
      last_timestamp = timestamp;
      last_batch_origin = origin_sample_pairs[0];
    }

    if (first_timestamp < 0)
    {
      first_timestamp = timestamp;
    }

    if (point_count % ray_batch_size == 0 || g_quit)
    {
      ray_mapper->integrateRays(origin_sample_pairs.data(), unsigned(origin_sample_pairs.size()), intensities.data(),
                                opt.ray_mode_flags);
      delta_motion = glm::length(origin_sample_pairs[0] - last_batch_origin);
      accumulated_motion += delta_motion;
      last_batch_origin = origin_sample_pairs[0];

      if (point_count != ray_batch_size && !warned_no_motion && delta_motion == 0)
      {
        // Precisely zero motion seems awfully suspicious.
        std::cerr << "\nWarning: Precisely zero motion in batch\n" << std::flush;
        warned_no_motion = true;
      }

      const auto minmax = std::minmax_element(intensities.cbegin(), intensities.cend());
      std::cout << " batch min/max intensity: " << *(minmax.first) << "," << *(minmax.second) << "   ";

      sample_timestamps.clear();
      origin_sample_pairs.clear();
      intensities.clear();

      prog.incrementProgressBy(ray_batch_size);
      last_timestamp = timestamp;
      // Store into elapsedMs atomic.
      elapsed_ms = uint64_t((last_timestamp - timebase) * 1e3);

#ifndef OHMPOP_CPU
      const double elapsed_time = timestamp - last_timestamp;
      if (opt.progressive_mapping_slice > 0)
      {
        if (opt.mapping_interval >= 0)
        {
          next_mapper_update -= elapsed_time;
        }
        if (next_mapper_update <= 0)
        {
          next_mapper_update += opt.mapping_interval;
          // const auto mapper_start = Clock::now();
          mapper.update(opt.progressive_mapping_slice);
          // const auto mapper_end = Clock::now();
          // std::ostringstream msg;
          // msg << "\nMapper: " << (mapper_end - mapper_start) << '\n';
          // std::cout << msg.str();
        }
      }
#endif  // OHMPOP_CPU

      if (opt.point_limit && point_count >= opt.point_limit ||
          opt.time_limit > 0 && last_timestamp - timebase >= opt.time_limit || g_quit)
      {
        break;
      }
    }
  }

  // Make sure we have no more rays.
  if (!origin_sample_pairs.empty())
  {
    ray_mapper->integrateRays(origin_sample_pairs.data(), unsigned(origin_sample_pairs.size()), intensities.data(),
                              opt.ray_mode_flags);
    delta_motion = glm::length(origin_sample_pairs[0] - last_batch_origin);
    accumulated_motion += delta_motion;
    sample_timestamps.clear();
    origin_sample_pairs.clear();
    intensities.clear();
  }
  end_time = Clock::now();

  prog.endProgress();
  prog.pause();

  if (!opt.quiet)
  {
    std::cout << std::endl;
  }

  const double motion_epsilon = 1e-6;
  if (accumulated_motion < motion_epsilon)
  {
    std::cerr << "Warning: very low accumulated motion: " << accumulated_motion << std::endl;
  }

#ifndef OHMPOP_CPU
  const auto mapper_start = Clock::now();
  if (opt.post_population_mapping && !g_quit)
  {
    std::cout << "Finalising" << std::endl;
    mapper.update(0.0);
  }
  // mapper.join(!g_quit && opt.postPopulationMapping);
  end_time = Clock::now();
#endif  // OHMPOP_CPU

  // Sync the map.
  if (!opt.quiet)
  {
    std::cout << "syncing map" << std::endl;
  }
#ifndef OHMPOP_CPU
  gpu_map->syncVoxels();
#endif  // OHMPOP_CPU

  for (auto *out : streams)
  {
    if (!out)
    {
      continue;
    }
    const double time_range = last_timestamp - first_timestamp;
    const double processing_time_sec =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;

    *out << "Point count: " << point_count << '\n';
    *out << "Data time: " << time_range << '\n';
#ifndef OHMPOP_CPU
    *out << "Population completed in " << mapper_start - start_time << std::endl;
    *out << "Post mapper completed in " << end_time - mapper_start << std::endl;
#endif  // OHMPOP_CPU
    *out << "Total processing time: " << end_time - start_time << '\n';
    *out << "Efficiency: " << ((processing_time_sec > 0 && time_range > 0) ? time_range / processing_time_sec : 0.0)
         << '\n';
    *out << "Points/sec: " << unsigned((processing_time_sec > 0) ? point_count / processing_time_sec : 0.0) << '\n';
    const double mibibytes = 1024 * 1024;
    *out << "Memory (approx): " << map.calculateApproximateMemory() / (mibibytes) << " MiB\n";
    *out << std::flush;
  }

  if (opt.serialise)
  {
    saveMap(opt, map, opt.output_base_name, &prog, kSaveMap | kSaveCloud);
  }

  prog.joinThread();

  if (opt.ndt.enabled)
  {
#ifdef OHMPOP_CPU
    ndt_map->debugDraw();
#else   // OHMPOP_CPU
    static_cast<ohm::GpuNdtMap *>(gpu_map.get())->debugDraw();
#endif  //  OHMPOP_CPU
  }

  return 0;
}


int parseOptions(Options *opt, int argc, char *argv[])  // NOLINT(modernize-avoid-c-arrays)
{
  cxxopts::Options opt_parse(argv[0],
                             "Generate an occupancy map from a LAS/LAZ based point cloud and accompanying "
                             "trajectory file using GPU. The trajectory marks the scanner trajectory with timestamps "
                             "loosely corresponding to cloud point timestamps. Trajectory points are "
                             "interpolated for each cloud point based on corresponding times in the "
                             "trajectory.");
  opt_parse.positional_help("<cloud.laz> <_traj.txt> [output-base]");

  try
  {
#ifndef OHMPOP_CPU
    // Build GPU options set.
    std::vector<int> gpu_options_types(ohm::gpuArgsInfo(nullptr, nullptr, 0));
    std::vector<const char *> gpu_options(gpu_options_types.size() * 2);
    ohm::gpuArgsInfo(gpu_options.data(), gpu_options_types.data(), unsigned(gpu_options_types.size()));
#endif  // OHMPOP_CPU

    // clang-format off
    opt_parse.add_options()
      ("batch-size", "The number of points to process in each batch. Controls debug display.", optVal(opt->batch_size))
      ("help", "Show help.")
      ("cloud", "The input cloud (las/laz) to load.", cxxopts::value(opt->cloud_file))
      ("output","Output base name", optVal(opt->output_base_name))
      ("point-limit", "Limit the number of points loaded.", optVal(opt->point_limit))
      ("preload", "Preload this number of points before starting processing. -1 for all. May be used for separating processing and loading time.",
        optVal(opt->preload_count)->default_value("0")->implicit_value("-1"))
      ("q,quiet", "Run in quiet mode. Suppresses progress messages.", optVal(opt->quiet))
      ("sensor", "Offset from the trajectory to the sensor position. Helps correct trajectory to the sensor centre for better rays.", optVal(opt->sensor_offset))
      ("start-time", "Only process points time stamped later than the specified time.", optVal(opt->start_time))
      ("serialise", "Serialise the results? This option is intended for skipping saving during performance analysis.", optVal(opt->serialise))
      ("save-info", "Save timing information to text based on the output file name.", optVal(opt->save_info))
      ("time-limit", "Limit the elapsed time in the LIDAR data to process (seconds). Measured relative to the first data sample.", optVal(opt->time_limit))
      ("trajectory", "The trajectory (text) file to load.", cxxopts::value(opt->trajectory_file))
      ("prior", "Prior map file to load and continue to populate.", cxxopts::value(opt->prior_map))
      ("cloud-colour", "Colour for points in the saved cloud (if saving).", optVal(opt->cloud_colour))
#ifdef TES_ENABLE
      ("trace", "Enable debug tracing to the given file name to generate a 3es file. High performance impact.", cxxopts::value(opt->trace))
      ("trace-final", "Only output final map in trace.", cxxopts::value(opt->trace_final))
#endif // TES_ENABLE
      ;

    opt_parse.add_options("Map")
      ("clamp", "Set probability clamping to the given min/max. Given as a value, not probability.", optVal(opt->prob_range))
      ("clip-near", "Range within which samples are considered too close and are ignored. May be used to filter operator strikes.", optVal(opt->clip_near_range))
      ("dim", "Set the voxel dimensions of each region in the map. Range for each is [0, 255).", optVal(opt->region_voxel_dim))
      ("hit", "The occupancy probability due to a hit. Must be >= 0.5.", optVal(opt->prob_hit))
      ("miss", "The occupancy probability due to a miss. Must be < 0.5.", optVal(opt->prob_miss))
      ("resolution", "The voxel resolution of the generated map.", optVal(opt->resolution))
      ("uncompressed", "Maintain uncompressed map. By default, may regions may be compressed when no longer needed.", optVal(opt->uncompressed))
      ("voxel-mean", "Enable voxel mean coordinates?", optVal(opt->voxel_mean))
      ("threshold", "Sets the occupancy threshold assigned when exporting the map to a cloud.", optVal(opt->prob_thresh)->implicit_value(optStr(opt->prob_thresh)))
      ("ndt", "Use normal distibution transform occupancy map generation.", optVal(opt->ndt.enabled))
      ("ndt-cov-point-threshold", "Minimum number of samples requires in order to allow the covariance to reset at --ndt-cov-prob-threshold..", optVal(opt->ndt.covariance_reset_sample_count))
      ("ndt-cov-prob-threshold", "Low probability threshold at which the covariance can be reset as samples accumulate once more. See also --ndt-cov-point-threshold.", optVal(opt->ndt.covariance_reset_probability))
      ("ndt-adaptation-rate", "NDT adpatation rate [0, 1]. Controls how fast rays remove NDT voxels. Has a strong effect than miss_value when using NDT.",
        optVal(opt->ndt.adaptation_rate))
      ("ndt-sensor-noise", "Range sensor noise used for Ndt mapping. Must be > 0.", optVal(opt->ndt.sensor_noise))
      ("ndt-tm", "Use normal distibution transform traversability map generation.", optVal(opt->ndt.ndt_tm))
      ("mode", "Controls the mapping mode [ normal, sample, erode ]. The 'normal' mode is the default, with the full ray "
               "being integrated into the map. 'sample' mode only adds samples to increase occcupancy, while 'erode' "
               "only erodes free space by skipping the sample voxels.", optVal(opt->mode))
      ;

    // clang-format on

#ifndef OHMPOP_CPU
    // clang-format off
    opt_parse.add_options("Mapping")
      ("clearance", "Calculate clearance values for the map using this as the maximum search range. Zero to disable.", optVal(opt->clearance))
      ("clearance-uao", "During clearance value calculations, consider 'Unknown(voxels)-As-Occupied'.", optVal(opt->clearance_unknown_as_occupied))
      ("progressive", "Time slice allowed for progressive mapping processes. Zero to disable and update after population.", optVal(opt->progressive_mapping_slice))
      ("progressive-interval", "Interval for progressive mapping. Time is based on input data time.", cxxopts::value(opt->mapping_interval)->default_value(optStr(opt->mapping_interval)))
      ("post-mapping", "Allow mapping thread to complete after population?", optVal(opt->post_population_mapping))
      ;

    // clang-format on

    if (!gpu_options.empty())
    {
      auto adder = opt_parse.add_options("GPU");
      for (size_t i = 0; i < gpu_options_types.size(); ++i)
      {
        adder(gpu_options[(i << 1u) + 0], gpu_options[(i << 1u) + 1],
              gpu_options_types[i] == 0 ? ::cxxopts::value<bool>() : ::cxxopts::value<std::string>());
      }
    }
#endif  // OHMPOP_CPU


    opt_parse.parse_positional({ "cloud", "trajectory", "output" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Map", "Mapping", "GPU" }) << std::endl;
      return 1;
    }

    if (opt->cloud_file.empty())
    {
      std::cerr << "Missing input cloud" << std::endl;
      return -1;
    }

    // Derive ray_mode_flags from mode
    if (opt->mode == "normal")
    {
      opt->ray_mode_flags = ohm::kRfDefault;
    }
    else if (opt->mode == "samples")
    {
      opt->ray_mode_flags = ohm::kRfExcludeRay;
    }
    else if (opt->mode == "erode")
    {
      opt->ray_mode_flags = ohm::kRfExcludeSample;
    }
    else
    {
      std::cerr << "Unknown mode argument: " << opt->mode << std::endl;
      return -1;
    }

    // Set default ndt probability if using.
    if (opt->ndt.enabled)
    {
      bool prob_hit_given = false;
      bool prob_miss_given = false;
      for (const auto &item : parsed.arguments())
      {
        if (item.key() == "hit")
        {
          prob_hit_given = true;
        }
        if (item.key() == "miss")
        {
          prob_miss_given = true;
        }
      }

      if (!prob_hit_given)
      {
        // Use ndt default hit prob
        opt->prob_hit = opt->ndt.prob_hit;
      }

      if (!prob_miss_given)
      {
        // Use ndt default hit prob
        opt->prob_miss = opt->ndt.prob_miss;
      }
    }

#ifdef TES_ENABLE
    if (!opt->trace.empty())
    {
      if (getFileExtension(opt->trace) != "3es")
      {
        opt->trace += ".3es";
      }
    }
#endif  // TES_ENABLE
  }
  catch (const cxxopts::OptionException &e)
  {
    std::cerr << "Argument error\n" << e.what() << std::endl;
    return -1;
  }

  return 0;
}

int main(int argc, char *argv[])
{
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = parseOptions(&opt, argc, argv);

  if (res)
  {
    return res;
  }

  // Initialise TES
#ifdef TES_ENABLE
  std::unique_ptr<ohm::Trace> trace;
  if (!opt.trace.empty())
  {
    trace = std::make_unique<ohm::Trace>(opt.trace.c_str());
  }
#endif  //  TES_ENABLE

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  // Generate output name based on input if not specified.
  if (opt.output_base_name.empty())
  {
    const auto extension_start = opt.cloud_file.find_last_of('.');
    if (extension_start != std::string::npos)
    {
      opt.output_base_name = opt.cloud_file.substr(0, extension_start);
    }
    else
    {
      opt.output_base_name = opt.cloud_file;
    }
  }

#ifndef OHMPOP_CPU
  res = ohm::configureGpuFromArgs(argc, argv);
#endif  // OHMPOP_CPU
  if (res)
  {
    return res;
  }

  res = populateMap(opt);

  return res;
}
