// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUNDTMAP_H
#define GPUNDTMAP_H

#include "OhmGpuConfig.h"

#include "GpuMap.h"

namespace ohm
{
  class NdtMap;
  struct GpuNdtMapDetail;

  /// A GPU implementation of the @c NdtMap algorithm.
  ///
  /// For details of what the NDT algorithm is, see @c NdtMap. The comments here focus on how this algorithm is applied
  /// in GPU.
  ///
  /// The NDT algorithm requires voxels to have @c VoxelMean and @c CovarianceVoxel data in addition to occupancy
  /// values. While the mean can still be reasonably updated with contension in GPU using atomic operations because of
  /// it's relatively independent values, the covariance has 6 floating point values. These 6 floats are highly
  /// dependent and must be calculated and updated together. This could be done in a lock free fashion using a double
  /// buffered approach, but this would double the per voxel data size.
  ///
  /// Instead we do not support contension when calculating and updating covariance values. Covariance is updated in
  /// a second pass in @c integrateRays() . The first pass only integrates the voxels the rays pass through, skipping
  /// the sample voxels. This pass has the NDT miss logic applied, where we adjust the probability based on the current
  /// covariance value of each cell.
  ///
  /// The second pass of @c integrateRays() processes only the sample voxels. We run one GPU thread per sample, however,
  /// there may be multiple threads for each voxel as multiple samples are likely to fall in the same voxel. So we have
  /// multiple threads calculating the same result for the same voxel. We only allow the first - lowest global id -
  /// thread to write the result.
  ///
  /// This is not the most efficient process with redundant calculations made by multiple threads, however, it ensures
  /// that:
  ///
  /// 1. We avoid GPU contention on the covaraince
  /// 2. We keep all the voxel data in GPU memory. Doing the work in CPU requires repeatedly synching GPU and CPU.
  /// 3. We do not need to do additional CPU work such as sorting the samples by sample voxel.
  ///
  /// The base @c GpuMap supports having two batches in flight. It supports a second batch being prepped while the
  /// first is allowed to continue. The NDT changes do not support this and there is a sync point before executing
  /// the NDT sample update.
  ///
  /// The NDT changes are applied by overriding @c finaliseBatch()
  class GpuNdtMap : public GpuMap
  {
  public:
    GpuNdtMap(OccupancyMap *map, bool borrowed_map = true, unsigned expected_element_count = 2048,
              size_t gpu_mem_size = 0u);

    /// Set the range sensor noise estimate. For example, the range noise for a lidar sensor.
    ///
    /// @param noise_range The sensor noise range. Must be greater than zero.
    void setSensorNoise(float noise_range);

    /// Read the range sensor noise estimate.
    float sensorNoise() const;

    /// Debug render the NDT map ellipsoids via 3rd Eye Scene.
    void debugDraw() const;

  protected:
    /// Helper to access the internal pimpl cast to the correct type.
    GpuNdtMapDetail *detail();
    /// Helper to access the internal pimpl cast to the correct type.
    const GpuNdtMapDetail *detail() const;

    /// @override
    /// Cache the NDT kernel as well.
    void cacheGpuProgram(bool with_voxel_mean, bool force) override;

    /// @override
    void finaliseBatch(unsigned region_update_flags) override;

    /// @override
    void releaseGpuProgram() override;
  };
}  // namespace ohm

#endif  // GPUNDTMAP_H
