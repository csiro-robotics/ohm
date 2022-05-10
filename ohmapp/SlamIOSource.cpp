// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "SlamIOSource.h"

#include <ohm/Logger.h>

#include <slamio/SlamCloudLoader.h>

#include <glm/glm.hpp>

#include <chrono>
#include <fstream>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

using namespace ohm;
namespace ohmapp
{
void SlamIOSource::Options::configure(cxxopts::OptionAdder &adder)
{
  Super::Options::configure(adder);
  // clang-format off
  adder
    ("batch-delta", "Maximum delta in the sensor movement before forcing a batch up. Zero/negative to disable.", optVal(sensor_batch_delta))
    ("batch-size", "The number of points to process in each batch. Controls debug display. In GPU mode, this controls the GPU grid size.", optVal(batch_size))
    ("cloud", "The input cloud (las/laz) to load.", cxxopts::value(cloud_file))
    ("points-only", "Assume the point cloud is providing points only. Otherwise a cloud file with no trajectory is considered a ray cloud.", optVal(point_cloud_only))
    ("preload", "Preload this number of points before starting processing. -1 for all. May be used for separating processing and loading time.", optVal(preload_count)->default_value("0")->implicit_value("-1"))
    ("sensor", "Offset from the trajectory to the sensor position. Helps correct trajectory to the sensor centre for better rays.", optVal(sensor_offset))
    ("trajectory", "The trajectory (text) file to load.", cxxopts::value(trajectory_file))
    ;
  // clang-format on
}


void SlamIOSource::Options::print(std::ostream &out)
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

  if (sensor_batch_delta >= 0)
  {
    out << "Sensor batch delta: " << sensor_batch_delta << '\n';
  }
  if (batch_size)
  {
    out << "Points batch size: " << batch_size << '\n';
  }

  Super::Options::print(out);
}

SlamIOSource::SlamIOSource()
  : Super(std::make_unique<Options>())
{}


SlamIOSource::~SlamIOSource() = default;


std::string SlamIOSource::sourceName() const
{
  const auto extension_start = options().cloud_file.find_last_of('.');
  if (extension_start != std::string::npos)
  {
    return options().cloud_file.substr(0, extension_start);
  }
  return options().cloud_file;
}


uint64_t SlamIOSource::processedPointCount() const
{
  return processed_point_count_;
}


double SlamIOSource::processedTimeRange() const
{
  return processed_time_range_;
}


unsigned SlamIOSource::expectedBatchSize() const
{
  return options().batch_size;
}


void SlamIOSource::requestBatchSettings(unsigned batch_size, double max_sensor_motion)
{
  options().batch_size = batch_size;
  options().sensor_batch_delta = max_sensor_motion;
}


int SlamIOSource::validateOptions()
{
  if (options().cloud_file.empty())
  {
    ohm::logger::error("Missing input cloud\n");
    return -1;
  }

  return 0;
}


int SlamIOSource::prepareForRun(uint64_t &point_count, const std::string &reference_name)
{
  loader_ = std::make_unique<slamio::SlamCloudLoader>();
  loader_->enableReturnNumberInference(options().return_number_mode == ReturnNumberMode::Auto);

  loader_->setErrorLog([this](const char *msg) { ohm::logger::error(msg); });
  if (!options().trajectory_file.empty())
  {
    if (!loader_->openWithTrajectory(options().cloud_file.c_str(), options().trajectory_file.c_str()))
    {
      ohm::logger::error("Error loading cloud ", options().cloud_file, " with trajectory ", options().trajectory_file,
                         '\n');
      return 1;
    }
  }
  else if (!options().point_cloud_only)
  {
    if (!loader_->openRayCloud(options().cloud_file.c_str()))
    {
      ohm::logger::error("Error loading ray ", options().cloud_file, '\n');
      return 1;
    }
  }
  else if (options().point_cloud_only)
  {
    if (!loader_->openPointCloud(options().cloud_file.c_str()))
    {
      ohm::logger::error("Error loading point cloud ", options().cloud_file, '\n');
      return 1;
    }
  }

  loader_->setSensorOffset(options().sensor_offset);

  Clock::time_point end_time;

  if (options().preload_count)
  {
    int64_t preload_count = options().preload_count;
    if (preload_count < 0 && options().point_limit)
    {
      preload_count = options().point_limit;
    }

    ohm::logger::info("Preloading points");

    const auto start_time = Clock::now();
    if (preload_count < 0)
    {
      ohm::logger::info('\n');
      loader_->preload();
    }
    else
    {
      ohm::logger::info(" ", preload_count, '\n');
      loader_->preload(preload_count);
    }
    const auto end_time = Clock::now();
    const double preload_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;
    ohm::logger::info("Preload completed over ", preload_time, " seconds.", '\n');
  }

  const auto point_limit = options().point_limit;
  point_count = (point_limit) ? std::min<uint64_t>(point_limit, loader_->numberOfPoints()) : loader_->numberOfPoints();

  if (!reference_name.empty() && options().stats_mode == StatsMode::Csv)
  {
    const auto csv_stats_file = reference_name + "_stats.csv";
    stats_csv_ = std::make_unique<std::ofstream>(csv_stats_file.c_str());
    Stats::writeCsvHeader(*stats_csv_);
  }

  return 0;
}


int SlamIOSource::run(BatchFunction batch_function, unsigned *quit_level_ptr)
{
  const auto flush_on_exit = [this]() {
    if (stats_csv_)
    {
      stats_csv_->flush();
      stats_csv_.reset();
    };
  };

  time_point_start_ = Clock::now();
  if (!loader_)
  {
    flush_on_exit();
    return 1;
  }

  // Data samples array, optionall interleaved with sensor position.
  std::vector<glm::dvec3> sensor_and_samples;
  std::vector<glm::vec4> colours;
  std::vector<float> intensities;
  std::vector<double> timestamps;
  std::vector<uint8_t> return_numbers;
  glm::dvec3 batch_origin(0);
  glm::dvec3 last_batch_origin(0);
  // Update map visualisation every N samples.
  const size_t ray_batch_size = options().batch_size;
  double timebase = -1;
  double last_batch_timestamp = -1;
  double accumulated_motion = 0;
  double delta_motion = 0;
  bool warned_no_motion = false;

  //------------------------------------
  // Population loop.
  //------------------------------------
  Stats batch_stats{};
  slamio::SamplePoint sample{};
  bool point_pending = false;
  bool have_processed = false;
  bool finish = false;
  // Cache control variables
  const auto point_limit = options().point_limit;
  const auto time_limit = options().time_limit;
  const auto input_start_time = options().start_time;
  const auto sensor_batch_delta = options().sensor_batch_delta;

  // Get to the first sample tiem.
  // Read the first sample and set the time base.
  if (!loader_->nextSample(sample))
  {
    // No work to do.
    ohm::logger::info("No points to process\n");
    flush_on_exit();
    return 0;
  }

  timebase = sample.timestamp;

  while (sample.timestamp - timebase < input_start_time)
  {
    if (!loader_->nextSample(sample))
    {
      ohm::logger::info("No sample points before selected start time ", input_start_time, ". Nothign to do.\n");
      flush_on_exit();
      return 0;
    }
  }

  batch_origin = sample.origin;

  point_pending = true;
  global_stats_.data_time_start = batch_stats.data_time_start = timebase;
  global_stats_.process_time_start = batch_stats.process_time_start = 0;

  uint64_t process_points_local = 0;
  const uint8_t max_return_number =
    (options().return_number_mode != ReturnNumberMode::Off) ? std::numeric_limits<uint8_t>::max() : 0u;
  const bool use_return_number = loader_->hasReturnNumber() || loader_->returnNumberInference();
  processed_point_count_ = 0;
  processed_time_range_ = 0;
  while ((process_points_local < point_limit || point_limit == 0) &&
         (last_batch_timestamp - timebase < time_limit || time_limit == 0) && point_pending && !finish)
  {
    const double sensor_delta_sq = glm::dot(sample.origin - batch_origin, sample.origin - batch_origin);
    const bool sensor_delta_exceeded =
      sensor_batch_delta > 0 && sensor_delta_sq > sensor_batch_delta * sensor_batch_delta;

    // Add sample to the batch.
    if (!sensor_delta_exceeded)
    {
      if (!samplesOnly())
      {
        sensor_and_samples.emplace_back(sample.origin);
      }
      sensor_and_samples.emplace_back(sample.sample);
      colours.emplace_back(sample.colour);
      intensities.emplace_back(sample.intensity);
      timestamps.emplace_back(sample.timestamp);
      if (use_return_number)
      {
        return_numbers.emplace_back(std::min(sample.return_number, max_return_number));
      }

      const double ray_length = glm::length(sample.sample - sample.origin);
      batch_stats.ray_length_minimum = std::min(ray_length, batch_stats.ray_length_minimum);
      batch_stats.ray_length_maximum = std::max(ray_length, batch_stats.ray_length_maximum);
      batch_stats.ray_length_total += ray_length;

      point_pending = false;
    }
    else
    {
      // Flag a point as being pending so it's added on the next loop.
      point_pending = true;
    }

    if (!timestamps.empty() && (sensor_delta_exceeded || timestamps.size() >= ray_batch_size ||
                                point_limit && process_points_local + timestamps.size() >= point_limit))
    {
      finish = !processBatch(batch_function, batch_origin, sensor_and_samples, timestamps, intensities, colours,
                             return_numbers, batch_stats);

      delta_motion = glm::length(batch_origin - last_batch_origin);
      accumulated_motion += delta_motion;
      last_batch_origin = batch_origin;

      if (have_processed && !warned_no_motion && delta_motion == 0 && !timestamps.empty())
      {
        // Precisely zero motion seems awfully suspicious.
        ohm::logger::warn("\nWarning: Precisely zero motion in batch\n");
        warned_no_motion = true;
      }
      have_processed = true;
      process_points_local += timestamps.size();
      processed_point_count_ = process_points_local;
      processed_time_range_ = timestamps.back() - timebase;

      sensor_and_samples.clear();
      timestamps.clear();
      intensities.clear();
      colours.clear();
      return_numbers.clear();
    }

    if (!point_pending)
    {
      // Fetch next sample.
      point_pending = loader_->nextSample(sample);
    }
    batch_origin = (timestamps.empty()) ? sample.origin : batch_origin;

    finish = finish || (quit_level_ptr && *quit_level_ptr != 0u);
  }

  if (point_pending && (!point_limit || process_points_local < point_limit))
  {
    // Final point processing.
    if (!samplesOnly())
    {
      sensor_and_samples.emplace_back(sample.origin);
    }
    sensor_and_samples.emplace_back(sample.sample);
    colours.emplace_back(sample.colour);
    intensities.emplace_back(sample.intensity);
    timestamps.emplace_back(sample.timestamp);

    const double ray_length = glm::length(sample.sample - sample.origin);
    batch_stats.ray_length_minimum = std::min(ray_length, batch_stats.ray_length_minimum);
    batch_stats.ray_length_maximum = std::max(ray_length, batch_stats.ray_length_maximum);
    batch_stats.ray_length_total += ray_length;

    point_pending = false;
  }

  // Process the final batch.
  if (!timestamps.empty() && !finish)
  {
    processBatch(batch_function, last_batch_origin, sensor_and_samples, timestamps, intensities, colours,
                 return_numbers, batch_stats);
    process_points_local += timestamps.size();
    processed_point_count_ = process_points_local;
    processed_time_range_ = timestamps.back() - timebase;
  }

  const double motion_epsilon = 1e-6;
  if (accumulated_motion < motion_epsilon)
  {
    ohm::logger::warn("Warning: very low accumulated motion: ", accumulated_motion, '\n');
  }

  loader_->close();
  loader_.release();

  flush_on_exit();
  return 0;
}


DataSource::Stats SlamIOSource::globalStats() const
{
  return global_stats_;
}


DataSource::Stats SlamIOSource::windowedStats() const
{
  std::unique_lock<std::mutex> guard(stats_lock_);
  return windowed_stats_;
}


void SlamIOSource::updateWindowedStats()
{
  Stats stats{};
  calculateWindowedStats(stats, windowed_stats_buffer_.begin(), windowed_stats_buffer_.end());
  std::unique_lock<std::mutex> guard(stats_lock_);
  windowed_stats_ = stats;
  guard.unlock();
  if (stats_csv_)
  {
    *stats_csv_ << stats << '\n';
  }
}

void SlamIOSource::addBatchStats(const Stats &stats)
{
  windowed_stats_buffer_next_ = DataSource::addBatchStats(stats, global_stats_, windowed_stats_buffer_,
                                                          windowed_stats_buffer_size_, windowed_stats_buffer_next_);

  // Update the windowed stats whenever the buffer is full. We essentially write whenever the ring buffer *next* write
  // index wraps around to 0.
  if (windowed_stats_buffer_next_ == 0)
  {
    updateWindowedStats();
  }
}


// Yes, try inline this private function.
inline bool SlamIOSource::processBatch(const BatchFunction &batch_function, const glm::dvec3 &batch_origin,
                                       const std::vector<glm::dvec3> &sensor_and_samples,
                                       const std::vector<double> &timestamps, const std::vector<float> &intensities,
                                       const std::vector<glm::vec4> &colours, const std::vector<uint8_t> &return_number,
                                       Stats &stats)
{
  const auto time_now = Clock::now();
  stats.process_time_end = std::chrono::duration<double>(time_now - time_point_start_).count();
  stats.data_time_end = (!timestamps.empty()) ? timestamps.back() : stats.data_time_start;
  stats.ray_count = unsigned(timestamps.size());
  const bool keep_processing =
    batch_function(batch_origin, sensor_and_samples, timestamps, intensities, colours, return_number);
  if (options().stats_mode != StatsMode::Off)
  {
    addBatchStats(stats);
  }
  stats.reset(stats.process_time_end, stats.data_time_end);

  return keep_processing;
}
}  // namespace ohmapp
