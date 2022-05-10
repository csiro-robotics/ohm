// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMAPP_DATASOURCE_H
#define OHMAPP_DATASOURCE_H

#include "OhmAppConfig.h"

#include <ohm/Trace.h>

#include <ohmutil/ProgressMonitor.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <iosfwd>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace cxxopts
{
class OptionAdder;
class Options;
class ParseResult;
}  // namespace cxxopts

namespace slamio
{
class SlamCloudLoader;
}

namespace ohmapp
{
/// A data source for ohm map data used with @c MapHarness . The data source is responsible for running a data loop
/// in @c run() feeding data to the @c BatchFunction as required.
///
/// The @c MapHarness uses this object as follows:
///
/// - Call @c configure()
/// - Call @c validateOptions()
/// - Optionally call @c printConfiguration()
/// - Call @c prepareForRun()
/// - Call @c run() to process the data.
class DataSource
{
public:
  /// Describes how stats should be collected.
  enum class StatsMode
  {
    /// No stats
    Off,
    /// Display to console only.
    Console,
    /// Log to CSV file.
    Csv
  };

  /// Options on how return numbers are treated in point clouds.
  enum class ReturnNumberMode
  {
    /// No special handling of return number.
    Off,
    /// Use explicit fields if pressent, but allow infering return number based on timestamps. Sequential points sharing
    /// the same timestamp are marked as a second return.
    Auto,
    /// Supported if an explicit @c return_number field is present in the cloud.
    Explicit
  };

  /// Base options common to all data sources.
  struct Options
  {
    /// Process no more that this number of points. Zero for unbounded.
    uint64_t point_limit = 0;
    /// Skip data before this timestamp (seconds). May be relative or absolute depending on the implementation.
    double start_time = 0;
    /// Process data within a time interval set by this value (seconds).
    double time_limit = 0;
    /// How to should statistics be collected?
    StatsMode stats_mode = StatsMode::Off;
    /// How return number is treated in point clouds.
    ReturnNumberMode return_number_mode = ReturnNumberMode::Off;

    /// Virtual destructor.
    virtual ~Options();

    /// Configure command line parsing.
    /// @param parser The parsing object.
    void configure(cxxopts::Options &parser);
    /// Configure command line parsing.
    /// @param adder The object to add options to.
    virtual void configure(cxxopts::OptionAdder &adder);
    /// Print configured options.
    /// @param out Stream to print to.
    virtual void print(std::ostream &out);
  };

  /// Statistics about rays. Scope of the statistics depends on the context.
  struct Stats
  {
    /// Processing start time.
    double process_time_start = 0;
    /// Processing end time.
    double process_time_end = 0;
    /// Data start time.
    double data_time_start = 0;
    /// Data end time.
    double data_time_end = 0;
    /// Minimum sample ray length.
    double ray_length_minimum = std::numeric_limits<double>::max();
    /// Maximum sample ray length.
    double ray_length_maximum = 0;
    /// Total of the samples ray length - for calculating the average.
    double ray_length_total = 0;
    /// Number of sample rays.
    uint64_t ray_count = 0;

    /// Calculate the processing time range from start to end. Will (approximately) match @c dataTime() in real time
    /// processing.
    /// @return The process time range.
    inline double processTime() const { return process_time_end - process_time_start; }

    /// Calculate the data time range from start to end.
    /// @return The data time range.
    inline double dataTime() const { return data_time_end - data_time_start; }

    /// Calculate and return the average ray length. Returns zero when the average cannot be calcualted (no ray count).
    /// @return The average ray length or zero.
    inline double rayLengthAverage() const { return (ray_count) ? ray_length_total / double(ray_count) : 0.0; }

    /// Calculate the number of rays per second in processing time.
    /// @return The rays per second, or zero if no time range is available.
    inline double processRaysPerSecond() const { return (processTime() > 0) ? double(ray_count) / processTime() : 0.0; }

    /// Calculate the number of rays per second in data time.
    /// @return The rays per second, or zero if no time range is available.
    inline double dataRaysPerSecond() const { return (dataTime() > 0) ? double(ray_count) / dataTime() : 0.0; }

    inline void reset() { reset(0.0, 0.0); }

    /// Reset the stats setting the data and processing start times.
    /// @param time_start_reset Value to reset @c time_start to.
    inline void reset(double process_time_start_reset, double data_time_start_reset)
    {
      process_time_start = process_time_start_reset;
      data_time_start = data_time_start_reset;
      data_time_end = process_time_end = 0;
      ray_length_minimum = std::numeric_limits<double>::max();
      ray_length_maximum = ray_length_total = 0;
      ray_count = 0;
    }

    /// Write CSV format headers to @p out matching output of the streaming operator for @c Stats .
    static std::ostream &writeCsvHeader(std::ostream &out);
  };

  /// Create a harness with the given options specialisation.
  ///
  /// Takes ownership of the pointer. @p options should generally be a specialisation of @c Options .
  ///
  /// @param options Configuration options.
  DataSource(std::unique_ptr<Options> &&options);

  /// Virtual destructor.
  virtual ~DataSource();

  /// Get the configured options (mutable).
  inline Options &options() { return *options_; }
  /// Get the configured options.
  inline const Options &options() const { return *options_; }

  /// Get the extension for @p file excluding the leading '.' character.
  /// @param file File name to check.
  static std::string getFileExtension(const std::string &file);

  /// Should only samples be given to the first array argument of @c BatchFunction ( @c true) or sensor/sample pairs
  /// ( @c false ).
  inline bool samplesOnly() const { return samples_only_; }
  /// Set @c samplesOnly() .
  /// @param samples_only True to only pass samples to @c BatchFunction , false to interleave sensor/sample pairs.
  virtual void setSamplesOnly(bool samples_only);

  /// Return the name of the data source. For example, this may be the input file name without extension.
  /// @return A name for the data source.
  virtual std::string sourceName() const = 0;

  /// Query the number of points which have been processed.
  ///
  /// Threadsafe.
  ///
  /// @return The number of samples which have been submitted to the @c BatchFunction .
  virtual uint64_t processedPointCount() const = 0;
  /// Query the time interval covered by the sample points which have been processed.
  ///
  /// Threadsafe.
  ///
  /// @return The time range of samples which have been submitted to the @c BatchFunction .
  virtual double processedTimeRange() const = 0;

  /// Get the expected batch size for calls to @c BatchFunction . Zero if unknown.
  /// @return The expected sample batch size or zero if unknown.
  virtual unsigned expectedBatchSize() const = 0;

  /// Request that batches come in either the given @p batch_size or so as not to exceed the given  @p max_sensor_motion
  ///
  /// Implementations may ignore this setting.
  ///
  /// @param batch_size Target size for a batch.
  /// @param max_sensor_motion Maximum sensor motion for a batch. Zero to disable.
  virtual void requestBatchSettings(unsigned batch_size, double max_sensor_motion) = 0;

  /// Configure command line options for this data source.
  /// @param parser Command line parser.
  virtual void configure(cxxopts::Options &parser);

  /// Validate configured options.
  /// @param parsed Command line option parsing result.
  virtual int validateOptions();

  /// Print the command line configuration.
  /// @param out Stream to print to.
  virtual void printConfiguration(std::ostream &out);

  /// Prepare to execute, finalising the configured options.
  /// @param[out] predicted_point_count Set to the predicted number of points to process. Set to zero if unknown.
  /// @param reference_name Optional reference name which identifies the run. Generally used for additional file output
  /// such as stats logging.
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int prepareForRun(uint64_t &predicted_point_count, const std::string &reference_name = std::string()) = 0;

  /// Function object invoked for each data sample batch.
  /// @param batch_origin The sensor position for the first point in the batch.
  /// @param sensor_and_samples Sensor/sample point pairs when @c samplesOnly() is @c false or just sample points when
  /// @c samplesOnly() is @c true . Sensor and sample points are in the same frame as the @c batch_origin .
  /// @param timestamps Time stamps for each sample point.
  /// @param intensities Intensity values for each sample point. Will be zero when the input data has no intensity.
  /// @param colour Colour values for each sample point. Will be zero when the input data has no colour.
  /// @param return_number Zero based, sample return number where available. Zero is the first/primary return, 1 is the
  /// second return, etc. Empty when this information is not available.
  /// @return True to continue processessing, @c false to stop.
  using BatchFunction =
    std::function<bool(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                       const std::vector<double> &timestamps, const std::vector<float> &intensities,
                       const std::vector<glm::vec4> &colours, const std::vector<uint8_t> &return_number)>;

  /// Run data sample loading calling the @p batch_function for each data batch loaded.
  ///
  /// Implementations block until all data samples have been processed by @p batch_function or the @p batch_function
  /// returns false.
  ///
  /// @note @c samplesOnly() affects data passed to @c batch_function . When @c true, the @p batch_function
  /// @p sensor_and_samples array contains only data samples. When @c false it interleavses sensor and sample pairs
  /// where the sensor location may be unique for each sample point.
  ///
  /// @param batch_function Function call for each data sample batch.
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int run(BatchFunction batch_function, unsigned *quit_level_ptr) = 0;

  /// Retrieve the global stats on all data processed so far.
  ///
  /// This function is generally not thread safe. Use @c windowedStats() for thread safe, progressive statistics.
  ///
  /// @return Stats on the data processed.
  virtual Stats globalStats() const = 0;

  /// Retrieve windowed stats covering a recent time window determined by the implementation..
  ///
  /// This must be thread safe for asynchronous access.
  ///
  /// @return Stats on the recent sample window.
  virtual Stats windowedStats() const = 0;

  /// A helper for collating stats.
  ///
  /// Adds @p stats to the global stats and windowed stats buffer.
  ///
  /// @param stats Stats to add.
  /// @param global_stats Global stats to modify.
  /// @param windowed_stats_buffer Ring buffer of windowed stats items.
  /// @param windowed_stats_buffer_size Number of items to store in @p windowed_stats_buffer before cycling.
  /// @param windowed_stats_buffer_next Next insertion/overwrite index for @p windowed_stats_buffer once full.
  /// @return The next value of @c windowed_stats_buffer_next .
  static unsigned addBatchStats(const Stats &stats, Stats &global_stats, std::vector<Stats> &windowed_stats_buffer,
                                unsigned windowed_stats_buffer_size, unsigned windowed_stats_buffer_next);

  /// A helper function for calculating windowed stats.
  /// @param stats Object to calculate stats into.
  /// @param begin First collected stats iterator.
  /// @param end Collects stats end iterator.
  /// @tparam Iter An interatable type referencing a @c Stats range.
  template <typename Iter>
  inline static void calculateWindowedStats(Stats &stats, const Iter begin, const Iter end)
  {
    stats.reset();

    if (begin != end)
    {
      stats.data_time_start = begin->data_time_start;
      stats.data_time_end = begin->data_time_end;
      stats.process_time_start = begin->process_time_start;
      stats.process_time_end = begin->process_time_end;

      for (auto iter = begin; iter != end; ++iter)
      {
        stats.data_time_start = std::min(iter->data_time_start, stats.data_time_start);
        stats.data_time_end = std::max(iter->data_time_end, stats.data_time_end);
        stats.process_time_start = std::min(iter->process_time_start, stats.process_time_start);
        stats.process_time_end = std::max(iter->process_time_end, stats.process_time_end);
        stats.ray_length_minimum = std::min(iter->ray_length_minimum, stats.ray_length_minimum);
        stats.ray_length_maximum = std::max(iter->ray_length_maximum, stats.ray_length_maximum);
        stats.ray_length_total += iter->ray_length_total;
        stats.ray_count += iter->ray_count;
      }
    }
  }

private:
  /// User configurable options.
  std::unique_ptr<Options> options_;
  /// True to provide only data in the first array argument passed to @c processBatch() , false to include
  /// sensor/sample pairs.
  bool samples_only_ = true;
};
}  // namespace ohmapp

/// Streaming operator for @c ohmapp::DataSource::Stats logging comma separated values.
///
/// @param out The output stream.
/// @param stats The stats to print.
/// @return The output stream.
std::ostream &operator<<(std::ostream &out, const ohmapp::DataSource::Stats &stats);

/// Output streaming for @c ohmapp::DataSource::StatsMode .
/// @param out Output stream.
/// @param mode Mode value.
/// @return The output stream.
std::ostream &operator<<(std::ostream &out, ohmapp::DataSource::StatsMode mode);
/// Output streaming for @c ohmapp::DataSource::StatsMode .
/// @param in Intput stream.
/// @param mode Mode reference to read into.
/// @return The input stream.
std::istream &operator>>(std::istream &in, ohmapp::DataSource::StatsMode &mode);

/// Output streaming for @c ohmapp::DataSource::ReturnNumberMode .
/// @param out Output stream.
/// @param mode Mode value.
/// @return The output stream.
std::ostream &operator<<(std::ostream &out, ohmapp::DataSource::ReturnNumberMode mode);
/// Output streaming for @c ohmapp::DataSource::ReturnNumberMode .
/// @param in Intput stream.
/// @param mode Mode reference to read into.
/// @return The input stream.
std::istream &operator>>(std::istream &in, ohmapp::DataSource::ReturnNumberMode &mode);

#endif  // OHMAPP_DATASOURCE_H
