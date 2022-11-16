// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmAppConfig.h"

#include "DataSource.h"

#include <glm/vec3.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

namespace slamio
{
class SlamCloudLoader;
}

namespace ohmapp
{
/// A @c DataSource which loads from point clouds or ray clouds using @c slamio::SlamCloudLoader .
class ohmapp_API SlamIOSource : public DataSource
{
public:
  /// Base class alias.
  using Super = DataSource;

  /// Options specialisation.
  struct ohmapp_API Options : public Super::Options
  {
    /// Point cloud data file. May alternatively specify a ray cloud file where the normals prepresent a ray back to
    /// the sensor location. Ray clouds do not require a trajectory file.
    std::string cloud_file;
    /// Trajectory file. May be a point cloud or a text trajectory file. See @c slamio library.
    std::string trajectory_file;
    /// Translation offset to apply to the trajectory positions to get to resolve the sensor position.
    glm::dvec3 sensor_offset = glm::dvec3(0.0);
    /// Preload this number of points before starting map generation. -1 to preload all points. Can be used for timing.
    int64_t preload_count = 0;
    /// Delta sensor motion required before triggering a batch.
    double sensor_batch_delta = 0.0;
    /// Trigger a batch whenever this number of points have been loaded.
    unsigned batch_size = 4096;
    /// True to process a point cloud without a trajectory. No sensor positions are known, and the sensor positions are
    /// given as the sample positions.
    bool point_cloud_only = false;

    void configure(cxxopts::OptionAdder &adder) override;
    void print(std::ostream &out) override;
  };

  /// Default constructor.
  SlamIOSource();
  /// Destructor.
  ~SlamIOSource();

  /// Access options by reference.
  /// @return The application @c Options .
  inline Options &options() { return static_cast<Options &>(Super::options()); }
  /// @overload
  inline const Options &options() const { return static_cast<const Options &>(Super::options()); }

  std::string sourceName() const override;

  uint64_t processedPointCount() const override;
  double processedTimeRange() const override;
  unsigned expectedBatchSize() const override;
  void requestBatchSettings(unsigned batch_size, double max_sensor_motion) override;

  int validateOptions() override;
  int prepareForRun(uint64_t &point_count, const std::string &reference_name) override;

  int run(BatchFunction batch_function, unsigned *quit_level_ptr) override;

  Stats globalStats() const override;

  Stats windowedStats() const override;

private:
  /// Update and cache the @c windowedStats() .
  void updateWindowedStats();

  /// Add the given batch stats.
  /// @param stats Batch stats to add.
  void addBatchStats(const Stats &stats);

  /// Process a data batch, collecting stats.
  bool processBatch(const BatchFunction &batch_function, const glm::dvec3 &batch_origin,
                    const std::vector<glm::dvec3> &sensor_and_samples, const std::vector<double> &timestamps,
                    const std::vector<float> &intensities, const std::vector<glm::vec4> &colours,
                    const std::vector<uint8_t> &return_number, Stats &stats);

  using Clock = std::chrono::high_resolution_clock;

  /// Slam cloud loader. Valid after calling @c createSlamLoader() as called from @c run() .
  std::unique_ptr<slamio::SlamCloudLoader> loader_;
  /// Number of points processed. Must be kept up to date during @c run() for display and statistics.
  std::atomic<uint64_t> processed_point_count_{};
  /// Time range processed. Must be kept up to date during @c run() for display and statistics.
  std::atomic<double> processed_time_range_{};
  /// Stats window ring buffer.
  std::vector<Stats> windowed_stats_buffer_;
  /// Target buffer size for the stats window ring buffer.
  unsigned windowed_stats_buffer_size_ = 20;
  /// Next insertion index into the @c windowed_stats_buffer_ ring buffer.
  unsigned windowed_stats_buffer_next_ = 0;
  /// Global data stats.
  Stats global_stats_;
  /// Windowed data stats.
  Stats windowed_stats_;
  /// Processing (real) start time.
  Clock::time_point time_point_start_;
  /// Access guard for @c windowed_stats_
  mutable std::mutex stats_lock_;
  /// CSV logging stream for stats.
  std::unique_ptr<std::ostream> stats_csv_;
};
}  // namespace ohmapp
