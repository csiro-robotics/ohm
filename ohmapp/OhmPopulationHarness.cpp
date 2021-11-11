// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmPopulationHarness.h"

#include <slamio/SlamCloudLoader.h>

#include <ohmutil/OhmUtil.h>

#include <glm/vec4.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

namespace ohm
{
using Clock = std::chrono::high_resolution_clock;


OhmPopulationHarness::InputOptions::~InputOptions() = default;


void OhmPopulationHarness::InputOptions::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Input");
  configure(adder);
}


void OhmPopulationHarness::InputOptions::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("batch-delta", "Maximum delta in the sensor movement before forcing a batch up. Zero/negative to disable.", optVal(sensor_batch_delta))
    ("batch-size", "The number of points to process in each batch. Controls debug display. In GPU mode, this controls the GPU grid size.", optVal(batch_size))
    ("cloud", "The input cloud (las/laz) to load.", cxxopts::value(cloud_file))
    ("point-limit", "Limit the number of points loaded.", optVal(point_limit))
    ("points-only", "Assume the point cloud is providing points only. Otherwise a cloud file with no trajectory is considered a ray cloud.", optVal(point_cloud_only))
    ("preload", "Preload this number of points before starting processing. -1 for all. May be used for separating processing and loading time.",
      optVal(preload_count)->default_value("0")->implicit_value("-1"))
    ("sensor", "Offset from the trajectory to the sensor position. Helps correct trajectory to the sensor centre for better rays.", optVal(sensor_offset))
    ("start-time", "Only process points time stamped later than the specified time.", optVal(start_time))
    ("time-limit", "Limit the elapsed time in the LIDAR data to process (seconds). Measured relative to the first data sample.", optVal(time_limit))
    ("trajectory", "The trajectory (text) file to load.", cxxopts::value(trajectory_file))
    ;
  // clang-format on
}


void OhmPopulationHarness::InputOptions::print(std::ostream &out)
{
  out << "Cloud: " << cloud_file;
  if (!trajectory_file.empty() && !point_cloud_only)
  {
    out << " + " << trajectory_file << '\n';
  }
  else
  {
    if (point_cloud_only)
    {
      out << " (no trajectory)\n";
    }
    else
    {
      out << " (ray cloud)\n";
    }
  }

  if (preload_count)
  {
    out << "Preload: ";
    if (preload_count < 0)
    {
      out << "all";
    }
    else
    {
      out << preload_count;
    }
    out << '\n';
  }

  if (point_limit)
  {
    out << "Maximum point: " << point_limit << '\n';
  }

  if (start_time > 0)
  {
    out << "Process from timestamp: " << start_time << '\n';
  }

  if (time_limit > 0)
  {
    out << "Process to timestamp: " << time_limit << '\n';
  }

  if (sensor_batch_delta >= 0)
  {
    out << "Sensor batch delta: " << sensor_batch_delta << '\n';
  }
  if (batch_size)
  {
    out << "Points batch size: " << batch_size << '\n';
  }
}


OhmPopulationHarness::OutputOptions::OutputOptions() = default;
OhmPopulationHarness::OutputOptions::~OutputOptions() = default;


void OhmPopulationHarness::OutputOptions::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Output");
  configure(adder);
}


void OhmPopulationHarness::OutputOptions::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("cloud-colour", "Colour for points in the saved cloud (if saving).", optVal(cloud_colour))
    ("output", "Output base name. Saved file paths are derived from this string adding appropriate file extensions.", optVal(base_name))
    ("q,quiet", "Run in quiet mode. Suppresses progress messages.", optVal(quiet))
    ("save-cloud", "Save a point cloud after population?", optVal(save_cloud))
    ("save-info", "Write information on how the map was generated to text file?", optVal(save_info))
    ("save-map", "Save the map object after population?", optVal(save_map))
    ("trace", "Enable debug tracing to the given file name to generate a 3es file. High performance impact.", optVal(trace)->implicit_value("trace"))
    ("trace-final", "Only output final map in trace.", optVal(trace_final))
  ;
  // clang-format on
}


void OhmPopulationHarness::OutputOptions::print(std::ostream &out)
{
  out << "Output base path: " << base_name << '\n';
  std::string save_items;
  if (save_map)
  {
    save_items += " map";
  }
  if (save_cloud)
  {
    save_items += " cloud";
    if (cloud_colour != glm::vec3(0.0f))
    {
      out << "Save cloud colour: " << cloud_colour << '\n';
    }
  }
  if (!save_items.empty())
  {
    out << "Will save: " << save_items << '\n';
  }
  if (!trace.empty())
  {
#ifdef TES_ENABLE
    out << "3es trace file: " << trace << (trace_final ? "(final only)\n" : "\n");
#else   //
    out << "3es trace ignore (not compiled)\n";
#endif  // TES_ENABLE
  }
}


OhmPopulationHarness::MapOptions::~MapOptions() = default;


void OhmPopulationHarness::MapOptions::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Map");
  configure(adder);
}


void OhmPopulationHarness::MapOptions::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("resolution", "The voxel resolution of the generated map.", optVal(resolution))
  ;
  // clang-format on
}


void OhmPopulationHarness::MapOptions::print(std::ostream &out)
{
  out << "Map resolution: " << resolution << '\n';
}


OhmPopulationHarness::Options::Options()
{
  input_ = std::make_unique<OhmPopulationHarness::InputOptions>();
  output_ = std::make_unique<OhmPopulationHarness::OutputOptions>();
  map_ = std::make_unique<OhmPopulationHarness::MapOptions>();
}


OhmPopulationHarness::Options::~Options() = default;


void OhmPopulationHarness::Options::configure(cxxopts::Options &parser)
{
  parser.add_options()("h,help", "Display this help.");
  input_->configure(parser);
  output_->configure(parser);
  map_->configure(parser);

  parser.parse_positional(positional_args);
}


void OhmPopulationHarness::Options::print(std::ostream &out)
{
  out.imbue(std::locale(""));
  out << std::boolalpha;
  input_->print(out);
  output_->print(out);
  map_->print(out);
}


OhmPopulationHarness::OhmPopulationHarness(std::unique_ptr<Options> &&options)
  : options_(std::move(options))
{}


OhmPopulationHarness::~OhmPopulationHarness() = default;


std::string OhmPopulationHarness::getFileExtension(const std::string &file)
{
  const size_t last_dot = file.find_last_of('.');
  if (last_dot != std::string::npos)
  {
    return file.substr(last_dot + 1);
  }

  return "";
}


void OhmPopulationHarness::info(const std::string &msg)
{
  std::clog << msg << std::flush;
}


void OhmPopulationHarness::warn(const std::string &msg)
{
  std::cout << msg << std::flush;
}


void OhmPopulationHarness::error(const std::string &msg)
{
  std::cerr << msg << std::flush;
}


int OhmPopulationHarness::parseCommandLineOptions(int argc, const char *const *argv)
{
  cxxopts::Options parser(argv[0]);
  configureOptions(parser);

  int result = 0;
  try
  {
    cxxopts::ParseResult parsed = parser.parse(argc, argv);
    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << parser.help(options_->default_help_sections) << std::endl;
      quit_level_ = maxQuitLevel();
      return 0;
    }

    result = validateOptions(parsed);
  }
  catch (const cxxopts::OptionException &e)
  {
    std::cerr << "Argument error\n" << e.what() << std::endl;
    result = -1;
  }

  return result;
}


int OhmPopulationHarness::validateOptions(const cxxopts::ParseResult &parsed)
{
  (void)parsed;
  if (options_->input().cloud_file.empty())
  {
    error("Missing input cloud\n");
    return -1;
  }

  // Generate output name based on input if not specified.
  if (options_->output().base_name.empty())
  {
    const auto extension_start = options_->input().cloud_file.find_last_of('.');
    if (extension_start != std::string::npos)
    {
      options_->output().base_name = options_->input().cloud_file.substr(0, extension_start);
    }
    else
    {
      options_->output().base_name = options_->input().cloud_file;
    }
  }

#ifdef TES_ENABLE
  if (!options().output().trace.empty())
  {
    if (getFileExtension(options().output().trace) != "3es")
    {
      options().output().trace += ".3es";
    }
  }
#endif  // TES_ENABLE

  return 0;
}


int OhmPopulationHarness::run()
{
  int result_code = 0;

  if (quit_level_)
  {
    // Nothing to do.
    return result_code;
  }

#ifdef TES_ENABLE
  if (!options().output().trace.empty())
  {
    trace_ = std::make_unique<ohm::Trace>(options().output().trace.c_str());
  }
#endif  //  TES_ENABLE

  progress_.setDisplayFunction([this](const ProgressMonitor::Progress &progress) { displayProgress(progress); });

  slam_loader_ = createSlamLoader();
  slam_loader_->setSensorOffset(options_->input().sensor_offset);

  std::ofstream info_file_out;
  std::array<std::ostream *, 2> info_streams = { nullptr };

  if (!quiet())
  {
    info_streams[0] = &std::cout;
  }

  if (options_->output().save_info)
  {
    const std::string output_txt = options_->output().base_name + ".txt";
    info_file_out.open(output_txt.c_str());
    info_streams[1] = &info_file_out;
  }

  result_code = prepareForRun();
  if (result_code)
  {
    tearDown();
    return result_code;
  }

  for (auto *out : info_streams)
  {
    if (out)
    {
      options_->print(*out);
    }
  }

  // Data samples array, optionall interleaved with sensor position.
  std::vector<glm::dvec3> sensor_and_samples;
  std::vector<glm::vec4> colours;
  std::vector<float> intensities;
  std::vector<double> timestamps;
  glm::dvec3 batch_origin(0);
  glm::dvec3 last_batch_origin(0);
  uint64_t processed_count = 0;
  // Update map visualisation every N samples.
  const size_t ray_batch_size = options_->input().batch_size;
  double timebase = -1;
  double first_timestamp = -1;
  double last_batch_timestamp = -1;
  double last_timestamp = 0;
  double accumulated_motion = 0;
  double delta_motion = 0;
  bool warned_no_motion = false;
  Clock::time_point start_time;
  Clock::time_point end_time;

  if (options_->input().preload_count)
  {
    int64_t preload_count = options_->input().preload_count;
    if (preload_count < 0 && options_->input().point_limit)
    {
      preload_count = options_->input().point_limit;
    }

    std::cout << "Preloading points";

    start_time = Clock::now();
    if (preload_count < 0)
    {
      std::cout << std::endl;
      slam_loader_->preload();
    }
    else
    {
      std::cout << " " << preload_count << std::endl;
      slam_loader_->preload(preload_count);
    }
    end_time = Clock::now();
    const double preload_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;
    std::cout << "Preload completed over " << preload_time << " seconds." << std::endl;
  }

  start_time = Clock::now();
  std::cout << "Populating map" << std::endl;

  const auto point_limit = options_->input().point_limit;
  progress_.beginProgress(ProgressMonitor::Info(
    (point_limit) ? std::min<uint64_t>(point_limit, slam_loader_->numberOfPoints()) : slam_loader_->numberOfPoints()));
  progress_.startThread();

  //------------------------------------
  // Population loop.
  //------------------------------------
  slamio::SamplePoint sample{};
  bool point_pending = false;
  bool first_sample = true;
  // Cache control variables
  const auto time_limit = options_->input().time_limit;
  const auto input_start_time = options_->input().start_time;
  const auto sensor_batch_delta = options_->input().sensor_batch_delta;
  while ((processed_count < point_limit || point_limit == 0) &&
         (last_batch_timestamp - timebase < time_limit || time_limit == 0) &&
         (point_pending || slam_loader_->nextSample(sample)) && !quit_level_)
  {
    // Initialise time base for display.
    timebase = (timebase >= 0) ? timebase : sample.timestamp;

    if (sample.timestamp - timebase < input_start_time)
    {
      // Too soon.
      continue;
    }

    if (first_sample)
    {
      last_batch_origin = sample.origin;
      first_sample = false;
    }

    if (timestamps.empty())
    {
      batch_origin = sample.origin;
    }

    // Cache first processed timestamp.
    first_timestamp = (first_timestamp >= 0) ? first_timestamp : sample.timestamp;

    const double sensor_delta_sq = glm::dot(sample.origin - batch_origin, sample.origin - batch_origin);
    const bool sensor_delta_exceeded =
      sensor_batch_delta > 0 && sensor_delta_sq > sensor_batch_delta * sensor_batch_delta;

    // Add sample to the batch.
    if (!sensor_delta_exceeded)
    {
      if (collate_sensor_and_samples_)
      {
        sensor_and_samples.emplace_back(sample.origin);
      }
      sensor_and_samples.emplace_back(sample.sample);
      colours.emplace_back(sample.colour);
      intensities.emplace_back(sample.intensity);
      timestamps.emplace_back(sample.timestamp);
      point_pending = false;
    }
    else
    {
      // Flag a point as being pending so it's added on the next loop.
      point_pending = true;
    }

    if (sensor_delta_exceeded || timestamps.size() >= ray_batch_size)
    {
      const auto current_batch_size = timestamps.size();
      processBatch(batch_origin, sensor_and_samples, timestamps, intensities, colours);
      progress_.incrementProgressBy(current_batch_size);
      last_timestamp = timestamps.back();
      dataset_elapsed_ms_ = uint64_t((last_timestamp - timebase) * 1e3);

      delta_motion = glm::length(batch_origin - last_batch_origin);
      accumulated_motion += delta_motion;
      last_batch_origin = batch_origin;

      if (processed_count && !warned_no_motion && delta_motion == 0 && timestamps.size() > 1)
      {
        // Precisely zero motion seems awfully suspicious.
        warn("\nWarning: Precisely zero motion in batch\n");
        warned_no_motion = true;
      }
      processed_count += timestamps.size();

      sensor_and_samples.clear();
      timestamps.clear();
      intensities.clear();
      colours.clear();
    }
  }

  if (!timestamps.empty())
  {
    const auto current_batch_size = timestamps.size();
    processBatch(last_batch_origin, sensor_and_samples, timestamps, intensities, colours);
    progress_.incrementProgressBy(current_batch_size);
    dataset_elapsed_ms_ = uint64_t((last_batch_timestamp - timebase) * 1e3);
    processed_count += timestamps.size();
  }

  finaliseMap();

  progress_.endProgress();
  progress_.pause();

  end_time = Clock::now();

  const double time_range = last_timestamp - first_timestamp;
  const double processing_time_sec =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;
  std::cout << std::endl;

  if (quit_level_ < 2)
  {
    for (auto *out : info_streams)
    {
      if (out)
      {
        *out << "Sample count: " << processed_count << '\n';
        *out << "Data time: " << time_range << '\n';
        *out << "Total processing time: " << end_time - start_time << '\n';
        *out << "Efficiency: " << ((processing_time_sec > 0 && time_range > 0) ? time_range / processing_time_sec : 0.0)
             << '\n';
        *out << "Points/sec: " << unsigned((processing_time_sec > 0) ? processed_count / processing_time_sec : 0.0)
             << '\n';
        // *out << "Memory (approx): " << map.calculateApproximateMemory() / (1024.0 * 1024.0) << " MiB\n";
        *out << std::flush;
      }
    }
  }

  const double motion_epsilon = 1e-6;
  if (accumulated_motion < motion_epsilon)
  {
    std::ostringstream out;
    out.imbue(std::locale(""));
    out << "Warning: very low accumulated motion: " << accumulated_motion << std::endl;
    warn(out.str());
  }

  result_code = serialise();

  progress_.joinThread();

  tearDown();

  return 0;
}


void OhmPopulationHarness::configureOptions(cxxopts::Options &parser)
{
  options_->configure(parser);
}


std::unique_ptr<slamio::SlamCloudLoader> OhmPopulationHarness::createSlamLoader()
{
  std::unique_ptr<slamio::SlamCloudLoader> loader = std::make_unique<slamio::SlamCloudLoader>();
  loader->setErrorLog([this](const char *msg) { this->error(msg); });
  if (!options_->input().trajectory_file.empty())
  {
    if (!loader->openWithTrajectory(options_->input().cloud_file.c_str(), options_->input().trajectory_file.c_str()))
    {
      std::ostringstream err;
      err.imbue(std::locale(""));
      err << "Error loading cloud " << options_->input().cloud_file << " with trajectory "
          << options_->input().trajectory_file << "\n";
      error(err.str());
      return nullptr;
    }
  }
  else if (!options_->input().point_cloud_only)
  {
    if (!loader->openRayCloud(options_->input().cloud_file.c_str()))
    {
      std::ostringstream err;
      err.imbue(std::locale(""));
      err << "Error loading ray " << options_->input().cloud_file << "\n";
      error(err.str());
      return nullptr;
    }
  }
  else if (options_->input().point_cloud_only)
  {
    if (!loader->openPointCloud(options_->input().cloud_file.c_str()))
    {
      std::ostringstream err;
      err.imbue(std::locale(""));
      err << "Error loading point cloud " << options_->input().cloud_file << "\n";
      error(err.str());
      return nullptr;
    }
  }

  return loader;
}


int OhmPopulationHarness::serialise()
{
  int result_code = 0;
  if (options_->output().save_map && quit_level_ < 2)
  {
    result_code = saveMap(options_->output().base_name);
    if (result_code)
    {
      return result_code;
    }
  }

  if (options_->output().save_cloud && quit_level_ < 2)
  {
    std::string output_ply = options_->output().base_name + ".ply";
    if (output_ply == options_->input().cloud_file || output_ply == options_->input().trajectory_file)
    {
      output_ply = options_->output().base_name + "_cloud.ply";
    }
    result_code = saveCloud(output_ply);
    if (result_code)
    {
      return result_code;
    }
  }

  return result_code;
}


void OhmPopulationHarness::displayProgress(const ProgressMonitor::Progress &progress)
{
  if (!quiet())
  {
    const uint64_t elapsed_ms_local = dataset_elapsed_ms_;
    const uint64_t sec = elapsed_ms_local / 1000u;
    const auto ms = unsigned(elapsed_ms_local - sec * 1000);

    std::ostringstream out;
    out.imbue(std::locale(""));
    out << '\r';

    if (!progress.info.info.empty())
    {
      out << progress.info.info << " : ";
    }

    out << sec << '.' << std::setfill('0') << std::setw(3) << ms << "s : ";

    const auto fill_width = std::numeric_limits<decltype(progress.progress)>::digits10;
    out << std::setfill(' ') << std::setw(fill_width) << progress.progress;
    if (progress.info.total)
    {
      out << " / " << std::setfill(' ') << std::setw(fill_width) << progress.info.total;
    }
    out << "    ";
    std::cout << out.str() << std::flush;
  }
}
}  // namespace ohm
