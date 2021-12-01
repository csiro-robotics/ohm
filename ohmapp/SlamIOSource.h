// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmAppConfig.h"

#include "DataSource.h"

#include <glm/vec3.hpp>

#include <atomic>
#include <memory>

namespace slamio
{
class SlamCloudLoader;
}

namespace ohmapp
{
/// A @c DataSource which loads from point clouds or ray clouds using @c slamio::SlamCloudLoader .
class SlamIOSource : public DataSource
{
public:
  /// Base class alias.
  using Super = DataSource;

  /// Options specialisation.
  struct Options : public Super::Options
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
  int prepareForRun(uint64_t &point_count) override;

  int run(BatchFunction batch_function, unsigned *quit_level_ptr) override;

private:
  /// Slam cloud loader. Valid after calling @c createSlamLoader() as called from @c run() .
  std::unique_ptr<slamio::SlamCloudLoader> loader_;
  /// Number of points processed. Must be kept up to date during @c run() for display and statistics.
  std::atomic<uint64_t> processed_point_count_{};
  /// Time range processed. Must be kept up to date during @c run() for display and statistics.
  std::atomic<double> processed_time_range_{};
};
}  // namespace ohmapp
