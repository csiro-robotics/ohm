// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMAPP_OHMPOPCPU_H_
#define OHMAPP_OHMPOPCPU_H_

#include "OhmAppConfig.h"

#include <ohm/MapSerialise.h>
#include <ohm/NdtMap.h>
#include <ohm/NdtMode.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapper.h>
#include <ohm/VoxelTsdf.h>
#include <ohmutil/OhmUtil.h>

#include <ohmapp/MapHarness.h>

namespace ohmapp
{
/// Helper to display map serialisation progress.
class SerialiseMapProgress : public ohm::SerialiseProgress
{
public:
  /// Create with the given @p monitor using @p quit_level to track whether it should quit (when > 1).
  explicit SerialiseMapProgress(ProgressMonitor &monitor, const unsigned *quit_level = nullptr)
    : monitor_(monitor)
    , quit_level_(quit_level)
  {}

  /// Check if quit_level was set and is greater than 1.
  bool quit() const override { return quit_level_ && *quit_level_ > 1; }

  void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
  void incrementProgress(unsigned inc) override { monitor_.incrementProgressBy(inc); }

private:
  ProgressMonitor &monitor_;
  const unsigned *quit_level_;
};

/// Population harness to generate an @c ohm::OccupancyMap using ohm CPU algorithms.
class OhmAppCpu : public ohmapp::MapHarness
{
public:
  /// Base class alias.
  using Super = ohmapp::MapHarness;

  inline static constexpr unsigned defaultBatchSize() { return 4096u; }

  /// Specialised map options.
  struct MapOptions : public Super::MapOptions
  {
    /// Remove this much of the start of each ray.
    double clip_near_range = 0.0;
    /// Specifies the number of voxels along each dimension of a chunk.
    glm::u8vec3 region_voxel_dim = glm::u8vec3(0);  // will be re-initialised from a default map
    /// Probability for an occupancy hit.
    float prob_hit = 0.0f;  // will be re-initialised from a default map
    /// Probability for an occupancy miss.
    float prob_miss = 0.0f;  // will be re-initialised from a default map
    /// Occupancy probability threshold. Occupied when occupancy reaches this probability.
    float prob_thresh = 0.5f;                // will be re-initialised from a default map
    glm::vec2 prob_range = glm::vec2(0, 0);  // will be re-initialised from a default map
    /// String value for the "--mode" argument. This sets the value of @c ray_mode_flags - see that member.
    std::string mode = "normal";
    /// @c ohm::RayFlag selection based on the "--mode" argument which is mapped into the @c mode member.
    ///
    /// Supported modes:
    /// - "normal" (default) => @c ohm::kRfDefault
    /// - "sample" (default) => @c ohm::kRfExcludeRay
    /// - "erode" (default) => @c ohm::kRfExcludeSample
    unsigned ray_mode_flags = ohm::kRfDefault;
    /// Generate the map with voxel mean sub-voxel positioning?
    bool voxel_mean = false;
    /// Generate the map with voxel traversal for density queries?
    bool traversal = false;

    /// TSDF options.
    ohm::TsdfOptions tsdf{};
    bool tsdf_enabled = false;  ///< Enable TSDF mapping? Not compatible with occupancy.

    MapOptions();

    void configure(cxxopts::OptionAdder &adder) override;
    void print(std::ostream &out) override;
  };

  /// NDT map options.
  struct NdtOptions
  {
    // NDT map probabilities should be much narrower. The NDT process is more precise.
    /// Probability for an NDT map occupancy hit.
    float prob_hit = 0.0;  // will be re-initialised from a default map
    /// Probability for an NDT map occupancy miss.
    float prob_miss = 0.0;  // will be re-initialised from a default map
    /// NDT adaptation rate.
    float adaptation_rate = 0.0f;  // will be re-initialised from a default map
    /// Expected sensor noise for NDT model.
    float sensor_noise = 0.0f;  // will be re-initialised from a default map
    /// Reset covariance probability.
    float covariance_reset_probability = 0.0f;  // will be re-initialised from a default map
    /// Reset covariance sample count.
    unsigned covariance_reset_sample_count = 0;  // will be re-initialised from a default map
    /// NDT mode
    ohm::NdtMode mode = ohm::NdtMode::kNone;

    NdtOptions();
    virtual ~NdtOptions();

    /// Configure the command line options for the given @c parser . Calls @c `configure(const cxxopts::OptionAdder &)`
    /// @param parser The command line parser.
    void configure(cxxopts::Options &parser);
    /// Add command line options.
    /// Derivations should override this to add their own options as well as calling this base version.
    /// @param adder Object to add command line options to.
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    virtual void configure(cxxopts::OptionAdder &adder);
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    virtual void print(std::ostream &out);
  };

  /// Background compression options.
  struct CompressionOptions
  {
    /// High tide: invoke compression when this is exceeded.
    ohm::util::Bytes high_tide;
    /// Low tide: compress until this is reached.
    ohm::util::Bytes low_tide;
    /// True to disable compression.
    bool uncompressed = false;

    CompressionOptions();
    virtual ~CompressionOptions();

    /// Configure the command line options for the given @c parser . Calls @c `configure(const cxxopts::OptionAdder &)`
    /// @param parser The command line parser.
    void configure(cxxopts::Options &parser);
    /// Add command line options.
    /// Derivations should override this to add their own options as well as calling this base version.
    /// @param adder Object to add command line options to.
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    virtual void configure(cxxopts::OptionAdder &adder);
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    virtual void print(std::ostream &out);
  };

  /// Specialise collated options.
  struct Options : public Super::Options
  {
    /// NDT mapping options
    std::unique_ptr<NdtOptions> ndt_;
    /// Compression options
    std::unique_ptr<CompressionOptions> compression_;

    inline MapOptions &map() { return static_cast<MapOptions &>(*map_); }
    inline const MapOptions &map() const { return static_cast<const MapOptions &>(*map_); }

    /// Access the ndt mapping options by reference. Should be overriden by derivation to return their own ndt options
    /// specialisation.
    /// @return The @c NdtOptions .
    inline NdtOptions &ndt() { return *ndt_; }
    /// @overload
    inline const NdtOptions &ndt() const { return *ndt_; }

    /// Access the compressions options by reference. Should be overriden by derivation to return their own options
    /// specialisation.
    /// @return The @c CompressionOptions .
    inline CompressionOptions &compression() { return *compression_; }
    /// @overload
    inline const CompressionOptions &compression() const { return *compression_; }

    Options();

    void configure(cxxopts::Options &parser) override;
    void print(std::ostream &out) override;
  };

  /// Default constructor.
  OhmAppCpu(std::shared_ptr<ohmapp::DataSource> data_source);

  std::string description() const override;

  /// Access options by reference.
  /// @return The application @c Options .
  Options &options() { return static_cast<Options &>(Super::options()); }

  /// @overload
  const Options &options() const { return static_cast<const Options &>(Super::options()); }

  /// Access the map object.
  /// @return The @c OccupancyMap .
  inline ohm::OccupancyMap *map() { return map_.get(); }
  /// @overload
  inline const ohm::OccupancyMap *map() const { return map_.get(); }

protected:
  /// Constructor for derivied classes.
  /// @param options Options pointer.
  /// @param data_source Data source pointer.
  OhmAppCpu(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source);

  int validateOptions(const cxxopts::ParseResult &parsed) override;
  int prepareForRun() override;
  bool processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                    const std::vector<double> &timestamps, const std::vector<float> &intensities,
                    const std::vector<glm::vec4> &colours) override;
  void finaliseMap() override;
  int saveMap(const std::string &path_without_extension) override;
  int saveCloud(const std::string &path_ply) override;
  void tearDown() override;

  /// Object used to process points from @c processBatch() . Normally the same as @c true_mapper_
  ohm::RayMapper *mapper_ = nullptr;
  /// The occupancy map
  std::unique_ptr<ohm::OccupancyMap> map_;
  /// NDT map used when NDT is enabled. Null when NDT is disabled.
  std::unique_ptr<ohm::NdtMap> ndt_map_;
  /// The @c RayMapper used to add points to the map.
  std::unique_ptr<ohm::RayMapper> true_mapper_;
  /// Debug drawing mapper. Installed in @c mapper_ when tracing is enabled.
  std::unique_ptr<ohm::RayMapper> trace_mapper_;
};
}  // namespace ohmapp

#endif  // OHMAPP_OHMPOPCPU_H_
