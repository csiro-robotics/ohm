// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMPOPCPU_H_
#define OHMPOPCPU_H_

#include "OhmPopConfig.h"

#include <ohm/MapSerialise.h>
#include <ohm/NdtMap.h>
#include <ohm/NdtMode.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapper.h>
#include <ohmutil/OhmUtil.h>

#include <ohmapp/MapHarness.h>

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
class OhmPopCpu : public ohmapp::MapHarness
{
public:
  using Super = ohmapp::MapHarness;

  inline static constexpr unsigned defaultBatchSize() { return 4096u; }

  /// Specialised map options.
  struct MapOptions : public Super::MapOptions
  {
    double clip_near_range = 0.0;
    glm::u8vec3 region_voxel_dim = glm::u8vec3(0);  // will be re-initialised from a default map
    float prob_hit = 0.0f;                          // will be re-initialised from a default map
    float prob_miss = 0.0f;                         // will be re-initialised from a default map
    float prob_thresh = 0.5f;                       // will be re-initialised from a default map
    glm::vec2 prob_range = glm::vec2(0, 0);         // will be re-initialised from a default map
    /// String value for the "--mode" argument. This sets the value of @c ray_mode_flags - see that member.
    std::string mode = "normal";
    /// @c ohm::RayFlag selection based on the "--mode" argument which is mapped into the @c mode member.
    ///
    /// Supported modes:
    /// - "normal" (default) => @c ohm::kRfDefault
    /// - "sample" (default) => @c ohm::kRfExcludeRay
    /// - "erode" (default) => @c ohm::kRfExcludeSample
    unsigned ray_mode_flags = ohm::kRfDefault;
    bool voxel_mean = false;
    bool traversal = false;

    MapOptions();

    void configure(cxxopts::OptionAdder &adder) override;
    void print(std::ostream &out) override;
  };

  /// NDT map options.
  struct NdtOptions
  {
    // NDT map probabilities should be much narrower. The NDT process is more precise.
    float prob_hit = 0.0;                        // will be re-initialised from a default map
    float prob_miss = 0.0;                       // will be re-initialised from a default map
    float adaptation_rate = 0.0f;                // will be re-initialised from a default map
    float sensor_noise = 0.0f;                   // will be re-initialised from a default map
    float covariance_reset_probability = 0.0f;   // will be re-initialised from a default map
    unsigned covariance_reset_sample_count = 0;  // will be re-initialised from a default map
    ohm::NdtMode mode = ohm::NdtMode::kNone;

    NdtOptions();
    virtual ~NdtOptions();

    void configure(cxxopts::Options &parser);
    virtual void configure(cxxopts::OptionAdder &adder);
    virtual void print(std::ostream &out);
  };

  /// Background compression options.
  struct CompressionOptions
  {
    ohm::util::Bytes high_tide;
    ohm::util::Bytes low_tide;
    bool uncompressed = false;

    CompressionOptions();
    virtual ~CompressionOptions();

    void configure(cxxopts::Options &parser);
    virtual void configure(cxxopts::OptionAdder &adder);
    virtual void print(std::ostream &out);
  };

  /// Specialise collated options.
  struct Options : public Super::Options
  {
    std::unique_ptr<NdtOptions> ndt_;
    std::unique_ptr<CompressionOptions> compression_;

    inline MapOptions &map() { return static_cast<MapOptions &>(*map_); }
    inline const MapOptions &map() const { return static_cast<const MapOptions &>(*map_); }

    inline NdtOptions &ndt() { return *ndt_; }
    inline const NdtOptions &ndt() const { return *ndt_; }

    inline CompressionOptions &compression() { return *compression_; }
    inline const CompressionOptions &compression() const { return *compression_; }

    Options();

    void configure(cxxopts::Options &parser) override;
    void print(std::ostream &out) override;
  };

  /// Default constructor.
  OhmPopCpu(std::shared_ptr<ohmapp::DataSource> data_source);

  std::string description() const override;

  const Options &options() const { return static_cast<const Options &>(Super::options()); }

  Options &options() { return static_cast<Options &>(Super::options()); }

protected:
  OhmPopCpu(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source);

  int validateOptions(const cxxopts::ParseResult &parsed) override;
  int prepareForRun() override;
  bool processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                    const std::vector<double> &timestamps, const std::vector<float> &intensities,
                    const std::vector<glm::vec4> &colours) override;
  void finaliseMap() override;
  int saveMap(const std::string &path_without_extension) override;
  int saveCloud(const std::string &path_ply) override;
  void tearDown() override;

  ohm::RayMapper *mapper_ = nullptr;
  std::unique_ptr<ohm::OccupancyMap> map_;
  std::unique_ptr<ohm::NdtMap> ndt_map_;
  std::unique_ptr<ohm::RayMapper> true_mapper_;
  /// Debug drawing mapper.
  std::unique_ptr<ohm::RayMapper> trace_mapper_;
};

#endif  // OHMPOPCPU_H_
