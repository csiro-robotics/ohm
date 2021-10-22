// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGpuConfig.h"

#include "GpuMapDetail.h"

#include <ohmgpu/GpuMap.h>
#include <ohmgpu/gpu/RaysQueryResult.h>

#include <ohm/private/RaysQueryDetail.h>


namespace ohm
{
struct RaysQueryMapWrapperDetail : public GpuMapDetail
{
  gputil::Event results_event;
  gputil::Buffer results_gpu;
  std::vector<RaysQueryResult> results_cpu;
  float volume_coefficient = 1.0f;
  bool needs_sync = false;

  inline RaysQueryMapWrapperDetail()
    : GpuMapDetail(nullptr, false)
  {}
};

/// This is a creative use of the @c GpuMap where we use the standard @c GpuMap logic to marshal our rays, but
/// override the GPU kernel invocation to perform the rays query instead.
///
/// Notes:
/// - Only supports one inflight query.
/// - Rays cannot be removed by filtering, only clipped.
/// - Behaviour is undefined if the ray query does not fit in a single GPU batch.
class ohmgpu_API RaysQueryMapWrapper final : public GpuMap
{
public:
  /// Create a @c RaysQueryMapWrapper . Note we initialise with a null map and set the map pointer later.
  RaysQueryMapWrapper();

  /// Destructor
  ~RaysQueryMapWrapper() final;

  void setMap(OccupancyMap *map);
  void setVolumeCoefficient(float coefficient);
  float volumeCoefficient() const;

  const std::vector<RaysQueryResult> &results() const;

  using RayMapper::integrateRays;

protected:
  size_t integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities, const double *timestamps,
                       unsigned ray_update_flags) override;

  void onSyncVoxels(int buffer_index) override;

  /// Helper to access the internal pimpl cast to the correct type.
  RaysQueryMapWrapperDetail *detail();
  /// Helper to access the internal pimpl cast to the correct type.
  const RaysQueryMapWrapperDetail *detail() const;

  /// Load and cache the required GPU program. The @p with_voxel_mean value is irrelevant.
  void cacheGpuProgram(bool with_voxel_mean, bool with_traversal, bool force) final;

  /// Override the GPU kernenel invocation to perform the rays query.
  void finaliseBatch(unsigned region_update_flags) final;
};

struct RaysQueryDetailGpu : public RaysQueryDetail
{
  std::unique_ptr<RaysQueryMapWrapper> gpu_interface;
};
}  // namespace ohm
