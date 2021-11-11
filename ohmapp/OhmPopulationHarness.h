// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMTOOLS_OHMPOPULATIONHARNESS_H_
#define OHMTOOLS_OHMPOPULATIONHARNESS_H_

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

namespace ohm
{
/// A base class which can be extended to define an application which populates an occupancy map.
///
/// Usage:
/// - instantiate the harness specialisation
/// - Call @c parseCommandLineOptions() or configure @c options() directly.
/// - Call @c run()
/// - Release/destroy
class OhmPopulationHarness
{
public:
  constexpr static unsigned maxQuitLevel() { return 2u; }

  struct InputOptions
  {
    std::string cloud_file;
    std::string trajectory_file;
    glm::dvec3 sensor_offset = glm::dvec3(0.0);
    uint64_t point_limit = 0;
    int64_t preload_count = 0;
    double start_time = 0;
    double time_limit = 0;
    double sensor_batch_delta = 0.0;
    unsigned batch_size = 4096;
    /// Allow point cloud which is not a ray cloud if this is set. Integrates points only, no ray data.
    bool point_cloud_only = false;

    virtual ~InputOptions();

    void configure(cxxopts::Options &parser);
    virtual void configure(cxxopts::OptionAdder &adder);
    virtual void print(std::ostream &out);
  };

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
    std::unique_ptr<InputOptions> input_;
    std::unique_ptr<OutputOptions> output_;
    std::unique_ptr<MapOptions> map_;

    /// Positional argument names set when @c configure() is called.
    std::vector<std::string> positional_args = { "cloud", "trajectory", "output" };
    /// List of help sections to show when @c --help is used.
    std::vector<std::string> default_help_sections = { "", "Input", "Output", "Map" };

    Options();
    virtual ~Options();

    inline InputOptions &input() { return *input_; }
    inline const InputOptions &input() const { return *input_; }

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
  OhmPopulationHarness(std::unique_ptr<Options> &&options);
  /// Virtual destructor.
  virtual ~OhmPopulationHarness();

  /// Get the extension for @p file excluding the leading '.' character.
  static std::string getFileExtension(const std::string &file);

  inline bool quiet() const { return options_->output().quiet; }
  virtual void info(const std::string &msg);
  virtual void warn(const std::string &msg);
  virtual void error(const std::string &msg);

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

  inline bool collateSensorAndSamples() const { return collate_sensor_and_samples_; }

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

protected:
  void setCollateSensorAndSamples(bool collate) { collate_sensor_and_samples_ = collate; }

  /// Configure the @p parser to parse command line options into @p options() .
  virtual void configureOptions(cxxopts::Options &parser);

  /// Validate and modify options after parsing. Called from @c parseCommandLineOptions() .
  ///
  /// The default implementation validates input/ouput arguments.
  ///
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int validateOptions(const cxxopts::ParseResult &parsed);

  /// Create the @c SlamCloudLoader . The default implementation handles the required setup and options validation.
  ///
  /// Called from @c run() .
  virtual std::unique_ptr<slamio::SlamCloudLoader> createSlamLoader();

  /// Perform custom setup for execution such as map creation - called from @c run() after @c createSlamLoader() .
  virtual int prepareForRun() = 0;

  /// Process a batch of points as.
  ///
  /// The implementation is to integrate the samples into the map. Note that @p sensor_and_samples will either be an
  /// array of sensor origin/sample pairs when @c collateSensorAndSamples() is true, or it will be just the samples when
  /// @c collateSensorAndSamples() is false. In either case, @c batch_origin will be the sensor position for the first
  /// sample point. All sensor positions will include the @p InputOptions::sensor_offset .
  ///
  /// Called whenever one of the following conditions are met:
  /// - the @c InputOptions::batch_size is reached
  /// - @p InputOptions::sensor_batch_delta is positive and the sensor delta has been exceeded
  /// - there is no more input data and we have residual data in the buffers.
  ///
  /// @param batch_origin The sensor position for the first point in the batch.
  /// @param sensor_and_samples Sensor/sample point pairs or just sample points (see above).
  /// @param timestamps Time stamps for each sample point.
  /// @param intensities Intensity values for each sample point. Will be zero when the input data has no intensity.
  /// @param colour Colour values for each sample point. Will be zero when the input data has no colour.
  virtual void processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
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
  virtual void displayProgress(const ProgressMonitor::Progress &progress);

  /// Options for the populator. May be a derivation of @c OhmPopulationHarness::Options .
  std::unique_ptr<Options> options_;
  /// Progress reporting thread helper.
  ProgressMonitor progress_;

  /// 3es debug trace pointer. Will only be available when TES_ENABLE is preprocessor defined.
  std::unique_ptr<ohm::Trace> trace_;

private:
  /// Time elapsed in the input data set timestamps (milliseconds).
  std::atomic<uint64_t> dataset_elapsed_ms_;
  /// Slam cloud loader. Valid after calling @c createSlamLoader() as called from @c run() .
  std::unique_ptr<slamio::SlamCloudLoader> slam_loader_;
  /// True to collate sensor position and samples in the array passed to @c processBatch() .
  bool collate_sensor_and_samples_ = true;
  /// Value set when a quit is requested. A quit value of 1 will stop map population, while 2 will also skip
  /// serialisation.
  unsigned quit_level_ = 0;
};
}  // namespace ohm

#endif  // OHMTOOLS_OHMPOPULATIONHARNESS_H_
