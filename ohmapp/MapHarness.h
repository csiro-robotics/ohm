// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMAPP_MAPHARNESS_H
#define OHMAPP_MAPHARNESS_H

#include "OhmAppConfig.h"

#include <ohm/Logger.h>
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

namespace ohmapp
{
class DataSource;

/// A base class which can be extended to define an application which populates an occupancy map.
///
/// Usage:
/// - instantiate the harness specialisation
/// - Call @c parseCommandLineOptions() or configure @c options() directly.
/// - Call @c run()
/// - Release/destroy
class MapHarness
{
public:
  constexpr static unsigned maxQuitLevel() { return 2u; }

  struct OutputOptions
  {
    std::string base_name;
    glm::vec3 cloud_colour{};
    /// 3rd Eye Scene trace file
    std::string trace;
    bool trace_final = false;
    bool save_map = true;
    bool save_cloud = true;
    bool save_info = false;
    /// Suppress console output.
    bool quiet = false;

    OutputOptions();
    virtual ~OutputOptions();

    void configure(cxxopts::Options &parser);
    virtual void configure(cxxopts::OptionAdder &adder);
    virtual void print(std::ostream &out);
  };

  struct MapOptions
  {
    double resolution = 0.1;

    virtual ~MapOptions();

    void configure(cxxopts::Options &parser);
    virtual void configure(cxxopts::OptionAdder &adder);
    virtual void print(std::ostream &out);
  };

  struct Options
  {
    std::unique_ptr<OutputOptions> output_;
    std::unique_ptr<MapOptions> map_;

    /// Positional argument names set when @c configure() is called.
    std::vector<std::string> positional_args = { "cloud", "trajectory", "output" };
    /// List of help sections to show when @c --help is used.
    std::vector<std::string> default_help_sections = { "", "Input", "Output", "Map" };

    Options();
    virtual ~Options();

    inline OutputOptions &output() { return *output_; }
    inline const OutputOptions &outpu() const { return *output_; }

    inline MapOptions &map() { return *map_; }
    inline const MapOptions &map() const { return *map_; }

    virtual void configure(cxxopts::Options &parser);
    virtual void print(std::ostream &out);
  };

  /// Create a harness with the given options specialisation.
  ///
  /// Takes ownership of the pointer. @p options should generally be a specialisation of @c Options .
  MapHarness(std::unique_ptr<Options> &&options, std::shared_ptr<DataSource> data_source);
  /// Virtual destructor.
  virtual ~MapHarness();

  inline bool quiet() const { return options_->output().quiet; }

  /// Description string for command line help.
  virtual std::string description() const = 0;

  /// Setup the harness for execution, parsing the command line arguments using and preparing for execution.
  ///
  /// Calls
  /// - @c configureOptions()
  /// - @c validateOptions()
  ///
  /// @param argc Number of elements in @p argv .
  /// @param argv Command line arguments.
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int parseCommandLineOptions(int argc, const char *const *argv);

  /// Run the map generation.
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  int run();

  /// Request a quit, incrementing the quit level.
  inline void requestQuit() { ++quit_level_; }
  /// Query the current quit level.
  inline unsigned quitLevel() const { return quit_level_; }

  /// Get the @c quitLevel() address.
  inline unsigned *quitLevelPtr() { return &quit_level_; }

  /// Get the @c quitLevel() address.
  inline const unsigned *quitLevelPtr() const { return &quit_level_; }

  /// Query if the @c quitLevel() is high enough to skip map population.
  inline bool quitPopulation() const { return quit_level_ != 0u; }

  /// Query if the @c quitLevel() is high enough to skip serialisation.
  inline bool quitSerialisation() const { return quit_level_ > 1; }

  /// Get the @c Options - read only.
  const Options &options() const { return *options_; }

  /// Get the @c Options - read/write.
  Options &options() { return *options_; }

  /// Access the @c DataSource source object.
  /// @return The data source object.
  std::shared_ptr<ohmapp::DataSource> dataSource() const { return data_source_; }

protected:
  /// Configure the @p parser to parse command line options into @p options() .
  virtual void configureOptions(cxxopts::Options &parser);

  /// Validate and modify options after parsing. Called from @c parseCommandLineOptions() .
  ///
  /// The default implementation validates input/ouput arguments.
  ///
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int validateOptions(const cxxopts::ParseResult &parsed);

  /// Perform custom setup for execution such as map creation - called from @c run() after @c createSlamLoader() .
  virtual int prepareForRun() = 0;

  /// Process a batch of points as.
  ///
  /// The implementation is to integrate the samples into the map. Note that @p sensor_and_samples will either be an
  /// array of sensor origin/sample pairs or samples only depending on @c DataSource::samplesOnly()
  /// (`dataSource()->samplesOnly()`). Sensor and sample points are both in the same frame as @c batch_origin and not
  /// in the sensor frame.
  ///
  /// @note It is the implementation's responsibility to call @c progress_.incrementProgressBy(timestamps.size()) .
  ///
  /// @param batch_origin The sensor position for the first point in the batch.
  /// @param sensor_and_samples Sensor/sample point pairs or just sample points (see above).Sensor and sample points are
  /// in the same frame as the @c batch_origin .
  /// @param timestamps Time stamps for each sample point.
  /// @param intensities Intensity values for each sample point. Will be zero when the input data has no intensity.
  /// @param colour Colour values for each sample point. Will be zero when the input data has no colour.
  /// @return True to continue processing - generally should be `return !quitPopulation();`
  virtual bool processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                            const std::vector<double> &timestamps, const std::vector<float> &intensities,
                            const std::vector<glm::vec4> &colours) = 0;

  /// Called after app data have been added to the map to finalise.
  inline virtual void finaliseMap() {}

  /// Serialise the map based on @c OutputOptions . Calls @c saveMap() and/or @c saveCloud() as indicated by the
  /// @c OutputOptions::save_flags .
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  int serialise();

  /// Save map data. Called from se
  virtual int saveMap(const std::string &path_without_extension) = 0;
  /// Save point cloud from the map.
  virtual int saveCloud(const std::string &path_ply) = 0;

  /// Perform tear down after processing has completed. Called from @c run() after saving.
  ///
  /// Also called on failure to process if @c prepareForRun() has been called (including when that function fails).
  virtual void tearDown() = 0;

  /// Callback for @c ProgressMonitor::setDisplayFunction() which displays to @c std::cout reusing the same line (\\r).
  virtual void displayProgress(const ProgressMonitor::Progress &progress, bool final);

  /// Options for the populator. May be a derivation of @c MapHarness::Options .
  std::unique_ptr<Options> options_;
  /// Progress reporting thread helper.
  ProgressMonitor progress_;

  /// 3es debug trace pointer. Will only be available when TES_ENABLE is preprocessor defined.
  std::unique_ptr<ohm::Trace> trace_;

private:
  /// Time elapsed in the input data set timestamps (milliseconds).
  std::atomic<uint64_t> dataset_elapsed_ms_;
  /// Object from which samples are loaded.
  std::shared_ptr<DataSource> data_source_;
  /// Value set when a quit is requested. A quit value of 1 will stop map population, while 2 will also skip
  /// serialisation.
  unsigned quit_level_ = 0;
};
}  // namespace ohmapp

#endif  // OHMAPP_MAPHARNESS_H
