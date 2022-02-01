// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmAppCpu.h"

#include <ohm/DebugDraw.h>
#include <ohm/MapSerialise.h>
#include <ohm/Mapper.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/RayMapperNdt.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/RayMapperTrace.h>
#include <ohm/RayMapperTsdf.h>
#include <ohm/Trace.h>
#include <ohm/VoxelBlockCompressionQueue.h>
#include <ohm/VoxelData.h>

#include <ohmapp/DataSource.h>

#ifdef TES_ENABLE
#include <ohm/RayMapperTrace.h>
#endif  // TES_ENABLE

#include <ohmtools/OhmCloud.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/SafeIO.h>
#include <ohmutil/ScopedTimeDisplay.h>

std::istream &operator>>(std::istream &in, ohm::NdtMode &mode);
std::ostream &operator<<(std::ostream &out, const ohm::NdtMode mode);

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

std::istream &operator>>(std::istream &in, ohm::NdtMode &mode)
{
  // Note: we don't use ndtModeFromString() because we use abbreviations
  std::string mode_str;
  in >> mode_str;
  if (mode_str == "off")
  {
    mode = ohm::NdtMode::kNone;
  }
  else if (mode_str == "om")
  {
    mode = ohm::NdtMode::kOccupancy;
  }
  else if (mode_str == "tm")
  {
    mode = ohm::NdtMode::kTraversability;
  }
  else
  {
    throw cxxopts::invalid_option_format_error(mode_str);
  }
  return in;
}

std::ostream &operator<<(std::ostream &out, const ohm::NdtMode mode)
{
  // Note: we don't use ndtModeToString() because we use abbreviations
  switch (mode)
  {
  case ohm::NdtMode::kNone:
    out << "off";
    break;
  case ohm::NdtMode::kOccupancy:
    out << "om";
    break;
  case ohm::NdtMode::kTraversability:
    out << "tm";
    break;
  default:
    out << "<unknown>";
  }
  return out;
}

namespace ohmapp
{
OhmAppCpu::MapOptions::MapOptions()
{
  // Initialise defaults from map configurations.
  ohm::OccupancyMap defaults_map;

  region_voxel_dim = defaults_map.regionVoxelDimensions();
  prob_hit = defaults_map.hitProbability();
  prob_miss = defaults_map.missProbability();
  prob_thresh = defaults_map.occupancyThresholdProbability();
  prob_range[0] = defaults_map.minVoxelValue();
  prob_range[1] = defaults_map.maxVoxelValue();
}


void OhmAppCpu::MapOptions::configure(cxxopts::OptionAdder &adder)
{
  Super::MapOptions::configure(adder);
  // clang-format off
  adder
    ("ray-length-max", "Maximum ray length. Longer rays are clipped to this range. Use zero to disable.", optVal(ray_length_max))
    ("clamp", "Set probability clamping to the given min/max. Given as a value, not probability.", optVal(prob_range))
    ("dim", "Set the voxel dimensions of each region in the map. Range for each is [0, 255).", optVal(region_voxel_dim))
    ("hit", "The occupancy probability due to a hit. Must be >= 0.5.", optVal(prob_hit))
    ("miss", "The occupancy probability due to a miss. Must be < 0.5.", optVal(prob_miss))
    ("voxel-mean", "Enable voxel mean coordinates?", optVal(voxel_mean))
    ("traversal", "Enable traversal layer?", optVal(traversal))
    ("threshold", "Sets the occupancy threshold assigned when exporting the map to a cloud.", optVal(prob_thresh)->implicit_value(optStr(prob_thresh)))
    ("tsdf", "Build a tsdf map instead of an occupancy map. Incompatible with other voxel or occupancy options.", optVal(tsdf_enabled))
    ("tsdf-max-weight", "Maximum TSDF voxel weight.", optVal(tsdf.max_weight))
    ("tsdf-default-truncation", "Default truncation distance.", optVal(tsdf.default_truncation_distance))
    ("tsdf-dropoff", "Dropoff for the dropoff adjustment. Zero or negative to disable.", optVal(tsdf.dropoff_epsilon))
    ("tsdf-sparsity", "Compensation for sparse data sets. Zero or negative to disable.", optVal(tsdf.sparsity_compensation_factor))
    ("mode", "Controls the mapping mode [ normal, sample, erode ]. The 'normal' mode is the default, with the full ray "
              "being integrated into the map. 'sample' mode only adds samples to increase occupancy, while 'erode' "
              "only erodes free space by skipping the sample voxels.", optVal(mode))
    ;
  // clang-format on
}


void OhmAppCpu::MapOptions::print(std::ostream &out)
{
  Super::MapOptions::print(out);
  glm::i16vec3 region_dim = region_voxel_dim;
  region_dim.x = (region_dim.x) ? region_dim.x : OHM_DEFAULT_CHUNK_DIM_X;
  region_dim.y = (region_dim.y) ? region_dim.y : OHM_DEFAULT_CHUNK_DIM_Y;
  region_dim.z = (region_dim.z) ? region_dim.z : OHM_DEFAULT_CHUNK_DIM_Z;
  out << "Map region dimensions: " << region_dim << '\n';
  if (!tsdf_enabled)
  {
    out << "Mapping mode: " << mode << '\n';
    out << "Voxel mean position: " << (voxel_mean ? "on" : "off") << '\n';
    // out << "Map region memory: " << util::Bytes(ohm::OccupancyMap::voxelMemoryPerRegion(region_voxel_dim)) << '\n';
    out << "Hit probability: " << prob_hit << " (" << ohm::probabilityToValue(prob_hit) << ")\n";
    out << "Miss probability: " << prob_miss << " (" << ohm::probabilityToValue(prob_miss) << ")\n";
    out << "Probability threshold: " << prob_thresh << '\n';
    // out << "Probability range: [" << map.minVoxelProbability() << ' ' << map.maxVoxelProbability() << "]\n";
    // out << "Value range      : [" << map.minVoxelValue() << ' ' << map.maxVoxelValue() << "]\n";
  }
  else
  {
    out << "TSDF mode: fast\n";
    out << "TSDF max weight: " << tsdf.max_weight << '\n';
    out << "TSDF default truncation distance: " << tsdf.default_truncation_distance << '\n';
    out << "TSDF dropoff epsilon: ";
    if (tsdf.dropoff_epsilon > 0)
    {
      out << tsdf.dropoff_epsilon << '\n';
    }
    else
    {
      out << "off\n";
    }
    out << "TSDF sparsity compensation factor: ";
    if (tsdf.sparsity_compensation_factor > 0)
    {
      out << tsdf.sparsity_compensation_factor << '\n';
    }
    else
    {
      out << "off\n";
    }
  }

  out << "Ray length max: " << ray_length_max << '\n';
}


OhmAppCpu::NdtOptions::NdtOptions()
{
  ohm::OccupancyMap defaults_map;
  const ohm::NdtMap defaults_ndt(&defaults_map, true);
  // Default probabilities may differ for NDT.
  prob_hit = defaults_map.hitProbability();
  prob_miss = defaults_map.missProbability();
  adaptation_rate = defaults_ndt.adaptationRate();
  sensor_noise = defaults_ndt.sensorNoise();
  covariance_reset_probability = ohm::valueToProbability(defaults_ndt.reinitialiseCovarianceThreshold());
  covariance_reset_sample_count = defaults_ndt.reinitialiseCovariancePointCount();
  adaptation_rate = defaults_ndt.adaptationRate();
}


OhmAppCpu::NdtOptions::~NdtOptions() = default;


void OhmAppCpu::NdtOptions::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Ndt");
  configure(adder);
}


void OhmAppCpu::NdtOptions::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("ndt", "Normal distribution transform (NDT) occupancy map generation mode {off,om,tm}. Mode om is the NDT occupancy mode, where tm adds traversability mapping data.", optVal(mode)->implicit_value(optStr(ohm::NdtMode::kOccupancy)))
    ("ndt-cov-point-threshold", "Minimum number of samples requires in order to allow the covariance to reset at --ndt-cov-prob-threshold..", optVal(covariance_reset_sample_count))
    ("ndt-cov-prob-threshold", "Low probability threshold at which the covariance can be reset as samples accumulate once more. See also --ndt-cov-point-threshold.", optVal(covariance_reset_probability))
    ("ndt-adaptation-rate", "NDT adaptation rate [0, 1]. Controls how fast rays remove NDT voxels. Has a strong effect than miss_value when using NDT.", optVal(adaptation_rate))
    ("ndt-sensor-noise", "Range sensor noise used for Ndt mapping. Must be > 0.", optVal(sensor_noise))
    ;
  // clang-format on
}


void OhmAppCpu::NdtOptions::print(std::ostream &out)
{
  if (bool(mode))
  {
    out << "NDT mode: " << mode << '\n';
    out << "NDT adaptation rate: " << adaptation_rate << '\n';
    out << "NDT sensor noise: " << sensor_noise << '\n';
    out << "NDT covariance reset probability: " << covariance_reset_probability << '\n';
    out << "NDT covariance reset sample cout: " << covariance_reset_sample_count << '\n';
  }
}


OhmAppCpu::CompressionOptions::CompressionOptions()
{
  ohm::VoxelBlockCompressionQueue cq(true);  // Create in test mode.
  high_tide = ohm::util::Bytes(cq.highTide());
  low_tide = ohm::util::Bytes(cq.lowTide());
}


OhmAppCpu::CompressionOptions::~CompressionOptions() = default;


void OhmAppCpu::CompressionOptions::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Compression");
  configure(adder);
}

void OhmAppCpu::CompressionOptions::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("high-tide", "Set the high memory tide which the background compression thread will try keep below.", optVal(high_tide))
    ("low-tide", "Set the low memory tide to which the background compression thread will try reduce to once high-tide is exceeded.", optVal(low_tide))
    ("uncompressed", "Maintain uncompressed map. By default, may regions may be compressed when no longer needed.", optVal(uncompressed))
  ;
  // clang-format on
}


void OhmAppCpu::CompressionOptions::print(std::ostream &out)
{
  out << "Compression: " << (uncompressed ? "off" : "on") << '\n';
  if (!uncompressed)
  {
    out << "  High tide: " << high_tide << '\n';
    out << "  Low tide: " << low_tide << '\n';
  }
}


OhmAppCpu::Options::Options()
{
  map_ = std::make_unique<OhmAppCpu::MapOptions>();
  ndt_ = std::make_unique<OhmAppCpu::NdtOptions>();
  compression_ = std::make_unique<OhmAppCpu::CompressionOptions>();
  default_help_sections.emplace_back("Ndt");
  default_help_sections.emplace_back("Compression");
}


void OhmAppCpu::Options::configure(cxxopts::Options &parser)
{
  Super::Options::configure(parser);
  ndt_->configure(parser);
  compression_->configure(parser);
}


void OhmAppCpu::Options::print(std::ostream &out)
{
  Super::Options::print(out);
  ndt_->print(out);
  compression_->print(out);
}


OhmAppCpu::OhmAppCpu(std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::make_unique<Options>(), data_source)
{
  data_source->setSamplesOnly(false);
  data_source->requestBatchSettings(defaultBatchSize(), 0);
}


OhmAppCpu::OhmAppCpu(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::move(options), data_source)
{
  data_source->setSamplesOnly(false);
  data_source->requestBatchSettings(defaultBatchSize(), 0);
}


#if SLAMIO_HAVE_PDAL
#define CLOUD_TYPE "PDAL supported point cloud"
#else  // SLAMIO_HAVE_PDAL
#define CLOUD_TYPE "PLY point cloud"
#endif  // SLAMIO_HAVE_PDAL

std::string OhmAppCpu::description() const
{
  return "Generate an occupancy map from a ray cloud or a point cloud with accompanying "
         "trajectory file. The trajectory marks the scanner trajectory with timestamps "
         "loosely corresponding to cloud point timestamps. Trajectory points are "
         "interpolated for each cloud point based on corresponding times in the "
         "trajectory. A ray cloud uses the normals channel to provide a vector from "
         "point sample back to sensor location (see "
         "https://github.com/csiro-robotics/raycloudtools).\n"
         "\n"
         "The sample file is a " CLOUD_TYPE " file, while the trajectory is either a text "
         "trajectory containing [time x y z <additional>] items per line or is itself a "
         "point cloud file.";
}


int OhmAppCpu::validateOptions(const cxxopts::ParseResult &parsed)
{
  int return_code = Super::validateOptions(parsed);
  if (return_code)
  {
    return return_code;
  }

  // Derive ray_mode_flags from mode
  if (options().map().mode == "normal")
  {
    options().map().ray_mode_flags = ohm::kRfDefault;
  }
  else if (options().map().mode == "samples")
  {
    options().map().ray_mode_flags = ohm::kRfExcludeRay;
  }
  else if (options().map().mode == "erode")
  {
    options().map().ray_mode_flags = ohm::kRfExcludeSample;
  }
  else
  {
    std::cerr << "Unknown mode argument: " << options().map().mode << std::endl;
    return -1;
  }

  // Set default ndt probability if using.
  if (int(options().ndt().mode))
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
      options().map().prob_hit = options().ndt().prob_hit;
    }

    if (!prob_miss_given)
    {
      // Use ndt default hit prob
      options().map().prob_miss = options().ndt().prob_miss;
    }
  }
  return 0;
}


int OhmAppCpu::prepareForRun()
{
  ohm::MapFlag map_flags = ohm::MapFlag::kDefault;
  map_flags |= (options().map().voxel_mean) ? ohm::MapFlag::kVoxelMean : ohm::MapFlag::kNone;
  map_flags &= (options().compression().uncompressed) ? ~ohm::MapFlag::kCompressed : ~ohm::MapFlag::kNone;
  map_ = std::make_unique<ohm::OccupancyMap>(options().map().resolution, options().map().region_voxel_dim, map_flags);

  // Make sure we build layers before initialising any GPU map. Otherwise we can cache the wrong GPU programs.
  if (options().map().voxel_mean)
  {
    map_->addVoxelMeanLayer();
  }
  if (options().map().traversal)
  {
    map_->addTraversalLayer();
  }

  std::unique_ptr<ohm::NdtMap> ndt_map;
  if (bool(options().ndt().mode))
  {
    ndt_map_ = std::make_unique<ohm::NdtMap>(map_.get(), true, options().ndt().mode);
    ndt_map_->setAdaptationRate(options().ndt().adaptation_rate);
    ndt_map_->setSensorNoise(options().ndt().sensor_noise);
    ndt_map_->setReinitialiseCovarianceThreshold(ohm::probabilityToValue(options().ndt().covariance_reset_probability));
    ndt_map_->setReinitialiseCovariancePointCount(options().ndt().covariance_reset_sample_count);

    true_mapper_ = std::make_unique<ohm::RayMapperNdt>(ndt_map_.get());
  }
  else if (options().map().tsdf_enabled)
  {
    // Create a new layout with just the TSDF layer. We won't need occupancy.
    ohm::MapLayout layout;
    addTsdf(layout);
    map_->updateLayout(layout);
    auto tsdf_mapper = std::make_unique<ohm::RayMapperTsdf>(map_.get());
    tsdf_mapper->setTsdfOptions(options().map().tsdf);
    true_mapper_ = std::move(tsdf_mapper);
  }
  else
  {
    true_mapper_ = std::make_unique<ohm::RayMapperOccupancy>(map_.get());
  }

  map_->setHitProbability(options().map().prob_hit);
  map_->setOccupancyThresholdProbability(options().map().prob_thresh);
  map_->setMissProbability(options().map().prob_miss);
  if (options().map().prob_range[0] || options().map().prob_range[1])
  {
    map_->setMinVoxelValue(options().map().prob_range[0]);
    map_->setMaxVoxelValue(options().map().prob_range[1]);
  }
  // map_->setSaturateAtMinValue(options().map().saturateMin);
  // map_->setSaturateAtMaxValue(options().map().saturateMax);

  // Prevent ready saturation to free.
  // map_->setClampingThresMin(0.01);

  // Ensure options reflect map flags.
  options().map().voxel_mean = map_->voxelMeanEnabled();
  options().map().traversal = map_->traversalEnabled();

  if (options().map().ray_length_max > 0)
  {
    const auto ray_length_max = options().map().ray_length_max;
    map_->setRayFilter([ray_length_max](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) {
      return ohm::clipRayFilter(start, end, filter_flags, ray_length_max);
    });
  }
  else
  {
    // No ray length filter installed, but make sure we skip bad rays.
    map_->setRayFilter([](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) {
      return ohm::goodRayFilter(start, end, filter_flags, 0);
    });
  }

  mapper_ = true_mapper_.get();
#ifdef TES_ENABLE
  if (!options().output().trace.empty() && !options().output().trace_final)
  {
    trace_mapper_ = std::make_unique<ohm::RayMapperTrace>(map_.get(), true_mapper_.get());
    mapper_ = trace_mapper_.get();
  }
#endif  // TES_ENABLE

  return 0;
}


bool OhmAppCpu::processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                             const std::vector<double> &timestamps, const std::vector<float> &intensities,
                             const std::vector<glm::vec4> &colours)
{
  (void)batch_origin;
  (void)colours;
  mapper_->integrateRays(sensor_and_samples.data(), unsigned(sensor_and_samples.size()), intensities.data(),
                         timestamps.data(), options().map().ray_mode_flags);
  progress_.incrementProgressBy(timestamps.size());
  return !quitPopulation();
}


void OhmAppCpu::finaliseMap()
{
#ifdef TES_ENABLE
  if (map_)
  {
    ohm::debugDraw(*map_);
  }
#endif  // TES_ENABLE
}


int OhmAppCpu::saveMap(const std::string &path_without_extension)
{
  std::string output_file = path_without_extension + ".ohm";
  std::ostringstream out;
  out.imbue(std::locale(""));
  ohm::logger::info("Saving map to ", output_file, '\n');

  std::unique_ptr<SerialiseMapProgress> save_progress;
  save_progress = std::make_unique<SerialiseMapProgress>(progress_, quitLevelPtr());
  progress_.unpause();

  int err = ohm::save(output_file.c_str(), *map_, save_progress.get());

  progress_.endProgress();
  if (!quiet())
  {
    std::cout << std::endl;
  }

  if (err)
  {
    ohm::logger::error("Failed to save map: ", err, '\n');
  }

  return err;
}


int OhmAppCpu::saveCloud(const std::string &path_ply)
{
  // Save a cloud representation.
  ohm::logger::info("Converting to point cloud.\n");

  ohmtools::ProgressCallback save_progress_callback;
  uint64_t point_count = 0;

  ohm::logger::info("Saving point cloud to ", path_ply, '\n');
  progress_.beginProgress(ProgressMonitor::Info(map_->regionCount()));
  save_progress_callback = [this](size_t progress, size_t /*target*/) { progress_.updateProgress(progress); };

  if (!options().map().tsdf_enabled)
  {
    ohmtools::ColourByHeight colour_by_height(*map_);
    ohmtools::SaveCloudOptions save_opt;

    if (glm::all(glm::greaterThanEqual(options().output().cloud_colour, glm::vec3(0.0f))))
    {
      const ohm::Colour uniform_colour = ohm::Colour::fromRgbf(
        options().output().cloud_colour.r, options().output().cloud_colour.g, options().output().cloud_colour.b);
      save_opt.colour_select = [uniform_colour](const ohm::Voxel<const float> &) { return uniform_colour; };
    }
    else
    {
      save_opt.colour_select = [&colour_by_height](const ohm::Voxel<const float> &occupancy) {
        return colour_by_height.select(occupancy);
      };
    }

    point_count = ohmtools::saveCloud(path_ply.c_str(), *map_, save_opt, save_progress_callback);
  }
  else
  {
    glm::dvec3 min_ext{};
    glm::dvec3 max_ext{};
    map_->calculateExtents(&min_ext, &max_ext);
    point_count = ohmtools::saveTsdfCloud(
      path_ply.c_str(), *map_, min_ext, max_ext,
      std::min(options().map().tsdf.default_truncation_distance, 1.0f * float(options().map().resolution)),
      save_progress_callback);
  }

  progress_.endProgress();
  progress_.pause();

  if (!quiet())
  {
    ohm::logger::info("\nExported ", point_count, " point(s)\n");
  }

  return 0;
}


void OhmAppCpu::tearDown()
{
  // ndt_map->debugDraw();
  mapper_ = nullptr;
#ifdef TES_ENABLE
  trace_mapper_.release();
#endif  // TES_ENABLE
  true_mapper_.release();
  ndt_map_.release();
  map_.release();
}
}  // namespace ohmapp
