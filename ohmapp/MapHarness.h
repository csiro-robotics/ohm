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
#include <glm/vec4.hpp>

#include <functional>
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
///
/// Derivations should implement the pure virtual funtions as detailed in those function comments. Derivations may also
/// extend various @c Options structures providing their own type. It is recommended that appropriate conversion
/// functions are put in place to access the underlying types. For example, if @c MapOptions is extended then
/// @c Options should first be extended to instantiate the specialisation. Additional @c Options should override
/// @c Options::map() to return the @c MapOptions specialisation and @c MapHarness should override @c options() to
/// return the @c Options specialisation.
class MapHarness
{
public:
  /// Maximum value for @c quitLevel()
  constexpr static unsigned maxQuitLevel() { return 2u; }

  /// Callback function signature invoked from @c parseCommandLineOptions() after parsing, but before
  /// @c validateOptions() .
  using OnConfigureCallback = std::function<void()>;

  /// Options controlling output.
  struct OutputOptions
  {
    /// Base output name. Typically used as a base file name/path where an extension may be added appropriate to the
    /// serialisation type. E.g., saving an ohm map will add @c '.ohm' while saving a ply would add @c '.ply'.
    std::string base_name;
    /// Explicit colour to use when exportin a pointcloud. Any negative colour channels indicate the explicit colour
    /// is not being used.
    glm::vec3 cloud_colour{ -1.0f };
    /// 3rd Eye Scene trace file. Enables 3es debugging if available (compile switch).
    std::string trace;
    /// Only use 3es to visualise the final map?
    bool trace_final = false;
    /// Save the map file?
    bool save_map = true;
    /// Save a point cloud form the map?
    bool save_cloud = true;
    /// Save statistics to file?
    bool save_info = false;
    /// Suppress console output.
    bool quiet = false;

    OutputOptions();
    virtual ~OutputOptions();

    /// Configure the command line options for the given @c parser . Calls @c `configure(const cxxopts::OptionAdder &)`
    /// @param parser The command line parser.
    void configure(cxxopts::Options &parser);
    /// Add command line options.
    /// Derivations should override this to add their own options as well as calling this base version.
    /// @param adder Object to add command line options to.
    virtual void configure(cxxopts::OptionAdder &adder);
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    virtual void print(std::ostream &out);
  };

  /// Options controlling the map configuration.
  struct MapOptions
  {
    /// Voxel size.
    double resolution = 0.1;

    virtual ~MapOptions();

    /// Configure the command line options for the given @c parser . Calls @c `configure(const cxxopts::OptionAdder &)`
    /// @param parser The command line parser.
    void configure(cxxopts::Options &parser);
    /// Add command line options. Derivations should override this to add their own options as well as calling this
    /// base version.
    /// @param adder Object to add command line options to.
    virtual void configure(cxxopts::OptionAdder &adder);
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    virtual void print(std::ostream &out);
  };

  /// Collated options.
  struct Options
  {
    /// The output options.
    std::unique_ptr<OutputOptions> output_;
    /// The map options.
    std::unique_ptr<MapOptions> map_;

    /// Positional argument names set when @c configure() is called.
    std::vector<std::string> positional_args = { "cloud", "trajectory", "output" };
    /// List of help sections to show when @c --help is used.
    std::vector<std::string> default_help_sections = { "", "Input", "Output", "Map" };

    Options();
    virtual ~Options();

    /// Access the output options by reference. Should be overriden by derivation to return their own output options
    /// specialisation.
    /// @return The @c OutputOptions .
    inline OutputOptions &output() { return *output_; }
    /// @overload
    inline const OutputOptions &output() const { return *output_; }

    /// Access the map options by reference. Should be overriden by derivation to return their own map options
    /// specialisation.
    /// @return The @c MapOptions .
    inline MapOptions &map() { return *map_; }
    /// @overload
    inline const MapOptions &map() const { return *map_; }

    /// Configure the command line options for the given @c parser . Calls @c `configure(const cxxopts::OptionAdder &)`
    /// @param parser The command line parser.
    virtual void configure(cxxopts::Options &parser);
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    virtual void print(std::ostream &out);
  };

  /// Create a harness with the given options specialisation.
  ///
  /// Takes ownership of the pointer. @p options should generally be a specialisation of @c Options .
  MapHarness(std::unique_ptr<Options> &&options, std::shared_ptr<DataSource> data_source);
  /// Virtual destructor.
  virtual ~MapHarness();

  /// Running in quiet mode? Suppresses logging and progress display.
  inline bool quiet() const { return options_->output().quiet; }

  /// Access the @c ProgressMonitor object.
  /// @return The progress monitor.
  inline ProgressMonitor &progress() { return progress_; }
  /// @overload
  inline const ProgressMonitor &progress() const { return progress_; }

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
  ///
  /// The @c run() function manages execution as follows:
  /// - Binds the @c ProgressMonitor - @c progress() - display function to @c displayProgress()
  /// - Calls @c prepareForRun()
  /// - Displays @c DataSource options and the internal @c options() calling the @c print() function. May be called
  ///   multiple times displaying to different streams
  /// - Calls @c DataSource::prepareForRun()
  /// - Starts the @c progress() thread.
  /// - Calls @c DataSource::run() . This calls contains main execution loop.
  /// - Calls @c finaliseMap(). Serialisation may occur from here.
  /// - Displays statistics
  /// - Calls @c tearDown()
  ///
  /// Note that @c tearDown() is called out of sequence if any failures occur during execution. That is, @c tearDown()
  /// is called on error and the @c run() sequence terminated.
  ///
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  int run();

  /// Request a quit, incrementing the quit level.
  ///
  /// Incrementing the quit level progressively terminates certain loops. Specifically:
  /// - level 1 quits the map generation loop (as controlled by the @c DataSource )
  /// - level 2 quits serialisation
  ///
  /// The quit level ay be incremented any number of types, but has no further sematic effect after reaching
  /// @c maxQuitLevel() .
  inline void requestQuit() { ++quit_level_; }
  /// Query the current quit level.
  /// @return The current quit level.
  inline unsigned quitLevel() const { return quit_level_; }

  /// Get the @c quitLevel() address - generally used for incrementing on @c SIGINT .
  /// @return The @c quitLevel() address.
  inline unsigned *quitLevelPtr() { return &quit_level_; }

  /// @overload
  inline const unsigned *quitLevelPtr() const { return &quit_level_; }

  /// Query if the @c quitLevel() is high enough to skip map population.
  /// @return True to quit map generation.
  inline bool quitPopulation() const { return quit_level_ != 0u; }

  /// Query if the @c quitLevel() is high enough to skip serialisation.
  /// @return True to quit map serialisation.
  inline bool quitSerialisation() const { return quit_level_ > 1; }

  /// Get the @c Options - read only.
  /// @return The configured options.
  const Options &options() const { return *options_; }

  /// Get the @c Options - read/write.
  /// @return The configured options.
  Options &options() { return *options_; }

  /// Access the @c DataSource source object.
  /// @return The data source object.
  std::shared_ptr<ohmapp::DataSource> dataSource() const { return data_source_; }

  /// Set the callback to invoke on starting.
  ///
  /// Called after @c configureOptions(), but before @c validateOptions() .
  /// @param callback The function to invoke. May be set to empty for no callback.
  void setOnConfigureOptionsCallback(OnConfigureCallback callback) { on_configure_options_callback_ = callback; }

  /// Get the callback invoked on starting.
  /// @return The post configuration callback.
  OnConfigureCallback onConfigureOptionsCallback() const { return on_configure_options_callback_; }

protected:
  /// Configure the @p parser to parse command line options into @p options() .
  /// @param parser The command line parser.
  virtual void configureOptions(cxxopts::Options &parser);

  /// Validate and modify options after parsing. Called from @c parseCommandLineOptions() .
  ///
  /// The default implementation validates input/ouput arguments.
  ///
  /// @param parsed The results from the parsing call.
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int validateOptions(const cxxopts::ParseResult &parsed);

  /// Perform custom setup for execution such as map creation - called from @c run() after @c createSlamLoader() .
  /// @return Zero on success or an exit code on error.
  virtual int prepareForRun() = 0;

  /// Process a batch of samples, adding them to the map.
  ///
  /// The implementation is to integrate the samples into the map. Note that @p sensor_and_samples will either be an
  /// array of sensor origin/sample pairs or samples only depending on @c DataSource::samplesOnly()
  /// ( @c dataSource()->samplesOnly() ). Sensor and sample points are both in the same frame as @c batch_origin and not
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

  /// Save map data. Called from @c serialise() .
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
  virtual int saveMap(const std::string &path_without_extension) = 0;
  /// Save point cloud from the map. Called from @c serialise() .
  /// @return Zero on success, a non-zero value on failure which can be used as the program exit code.
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
  /// Called after @c configureOptions() for external notification, but before @c validateOptions() .
  OnConfigureCallback on_configure_options_callback_;
};
}  // namespace ohmapp

#endif  // OHMAPP_MAPHARNESS_H
