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
  /// Base options common to all data sources.
  struct Options
  {
    /// Process no more that this number of points. Zero for unbounded.
    uint64_t point_limit = 0;
    /// Skip data before this timestamp (seconds). May be relative or absolute depending on the implementation.
    double start_time = 0;
    /// Process data within a time interval set by this value (seconds).
    double time_limit = 0;

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
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int prepareForRun(uint64_t &predicted_point_count) = 0;

  /// Function object invoked for each data sample batch.
  /// @param batch_origin The sensor position for the first point in the batch.
  /// @param sensor_and_samples Sensor/sample point pairs when @c samplesOnly() is @c false or just sample points when
  /// @c samplesOnly() is @c true . Sensor and sample points are in the same frame as the @c batch_origin .
  /// @param timestamps Time stamps for each sample point.
  /// @param intensities Intensity values for each sample point. Will be zero when the input data has no intensity.
  /// @param colour Colour values for each sample point. Will be zero when the input data has no colour.
  /// @return True to continue processessing, @c false to stop.
  using BatchFunction =
    std::function<bool(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                       const std::vector<double> &timestamps, const std::vector<float> &intensities,
                       const std::vector<glm::vec4> &colours)>;

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

private:
  /// User configurable options.
  std::unique_ptr<Options> options_;
  /// True to provide only data in the first array argument passed to @c processBatch() , false to include sensor/sample
  /// pairs.
  bool samples_only_ = true;
};
}  // namespace ohmapp

#endif  // OHMAPP_DATASOURCE_H
