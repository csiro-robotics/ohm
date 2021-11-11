// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMPOPGPU_H_
#define OHMPOPGPU_H_

#include "OhmPopConfig.h"

#include "OhmPopCpu.h"

#include <ohmgpu/GpuMap.h>
#include <ohmgpu/GpuNdtMap.h>

class OhmPopGpu : public OhmPopCpu
{
public:
  using Super = OhmPopCpu;

  /// GPU options.
  struct GpuOptions
  {
    /// See @c ohm::gpuArgsInfo()
    std::vector<int> device_option_types;
    /// See @c ohm::gpuArgsInfo()
    std::vector<const char *> device_options;
    double cache_size_gb = 0;
    double ray_segment_length = 0;

    GpuOptions();

    size_t gpuCacheSizeBytes() const;

    void configure(cxxopts::Options &parser);
    void print(std::ostream &out);
  };

  /// Specialise collated options.
  struct Options : public Super::Options
  {
    std::unique_ptr<GpuOptions> gpu_;

    inline GpuOptions &gpu() { return *gpu_; }
    inline const GpuOptions &gpu() const { return *gpu_; }

    Options();

    void configure(cxxopts::Options &parser) override;
    void print(std::ostream &out) override;
  };

  /// Default constructor.
  OhmPopGpu();

  int parseCommandLineOptions(int argc, const char *const *argv) override;

  const Options &options() const { return static_cast<const Options &>(Super::options()); }

  Options &options() { return static_cast<Options &>(Super::options()); }

  ohm::GpuMap *gpuMap();
  const ohm::GpuMap *gpuMap() const;

protected:
  OhmPopGpu(std::unique_ptr<Options> &&options);

  int prepareForRun() override;
  void finaliseMap() override;
};

#endif  // OHMPOPGPU_H_
