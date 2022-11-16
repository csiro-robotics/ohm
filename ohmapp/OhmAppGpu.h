// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMAPP_OHMPOPGPU_H_
#define OHMAPP_OHMPOPGPU_H_

#include "OhmAppGpuConfig.h"

#include "OhmAppCpu.h"

#include <ohmgpu/GpuMap.h>
#include <ohmgpu/GpuNdtMap.h>

namespace ohmapp
{
/// Population harness to generate an @c ohm::OccupancyMap using ohm GPU algorithms.
class ohmapp_API OhmAppGpu : public OhmAppCpu
{
public:
  /// Base class alias.
  using Super = OhmAppCpu;

  /// GPU options.
  struct ohmapp_API GpuOptions
  {
    /// See @c ohm::gpuArgsInfo()
    std::vector<int> device_option_types;
    /// See @c ohm::gpuArgsInfo()
    std::vector<const char *> device_options;
    /// Size of the GPU cache in GiB - supports non integer values.
    double cache_size_gb = 0;
    /// Maximum ray length before breaking up into multiple rays before GPU processing. Zero to submit rays as is.
    double ray_segment_length = 0;
    /// Trace rays forwards. GPU defaults to reverse ray tracing for performance.
    bool forward_trace = false;

    GpuOptions();

    /// Requested GPU cache size in bytes.
    size_t gpuCacheSizeBytes() const;

    /// Configure the command line options for the given @c parser . Calls @c `configure(const cxxopts::OptionAdder &)`
    /// @param parser The command line parser.
    void configure(cxxopts::Options &parser);
    /// Print command line options to the given stream.
    /// Derivations should override this to print their own options as well as calling this base version.
    /// @param out Output stream to print configured options to.
    void print(std::ostream &out);
  };

  /// Specialise collated options.
  struct ohmapp_API Options : public Super::Options
  {
    /// GPU options.
    std::unique_ptr<GpuOptions> gpu_;

    /// Access the GPU options by reference. Should be overriden by derivation to return their own options
    /// specialisation.
    /// @return The @c GpuOptions .
    inline GpuOptions &gpu() { return *gpu_; }
    /// @overload
    inline const GpuOptions &gpu() const { return *gpu_; }

    Options();

    void configure(cxxopts::Options &parser) override;
    void print(std::ostream &out) override;
  };

  /// Default constructor.
  /// @param data_source Options which will manage the data loop and provide data to @c progressBatch()
  OhmAppGpu(std::shared_ptr<ohmapp::DataSource> data_source);

  int parseCommandLineOptions(int argc, const char *const *argv) override;

  /// Access options by reference.
  /// @return The application @c Options .
  Options &options() { return static_cast<Options &>(Super::options()); }
  /// @overload
  const Options &options() const { return static_cast<const Options &>(Super::options()); }

  /// Access the GPU map object.
  /// @return The @c GpuMap .
  ohm::GpuMap *gpuMap();
  /// @overload
  const ohm::GpuMap *gpuMap() const;

protected:
  /// Constructor for derivied classes.
  /// @param options Options pointer.
  /// @param data_source Data source pointer.
  OhmAppGpu(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source);

  int prepareForRun() override;
  void finaliseMap() override;
};
}  // namespace ohmapp

#endif  // OHMAPP_OHMPOPGPU_H_
