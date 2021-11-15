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
class SlamIOSource : public DataSource
{
public:
  using Super = DataSource;

  struct Options : public Super::Options
  {
    std::string cloud_file;
    std::string trajectory_file;
    glm::dvec3 sensor_offset = glm::dvec3(0.0);
    double sensor_batch_delta = 0.0;
    int64_t preload_count = 0;
    unsigned batch_size = 4096;
    bool point_cloud_only = false;

    void configure(cxxopts::OptionAdder &adder) override;
    void print(std::ostream &out) override;
  };

  /// Default constructor.
  SlamIOSource();
  /// Destructor.
  ~SlamIOSource();

  inline Options &options() { return static_cast<Options &>(Super::options()); }
  inline const Options &options() const { return static_cast<const Options &>(Super::options()); }

  std::string sourceName() const override;

  uint64_t processedPointCount() const override;
  double processedTimeRange() const override;
  unsigned expectedBatchSize() const override;
  void requestBatchSettings(unsigned batch_size, double max_sensor_motion) override;

  int validateOptions() override;
  int prepareForRun(uint64_t &point_count) override;

  int run(BatchFunction batch_function) override;

private:
  /// Slam cloud loader. Valid after calling @c createSlamLoader() as called from @c run() .
  std::unique_ptr<slamio::SlamCloudLoader> loader_;
  std::atomic<uint64_t> processed_point_count_{};
  std::atomic<double> processed_time_range_{};
};
}  // namespace ohmapp
