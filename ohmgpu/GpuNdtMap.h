// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUNDTMAP_H
#define GPUNDTMAP_H

#include "OhmGpuConfig.h"

#include "GpuMap.h"

#include <ohm/NdtMode.h>

#include <array>

namespace gputil
{
struct Dim3;
class Event;
class EventList;
}  // namespace gputil

namespace ohm
{
class NdtMap;
class GpuLayerCache;
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
/// 1. We avoid GPU contention on the covariance
/// 2. We keep all the voxel data in GPU memory. Doing the work in CPU requires repeatedly synching GPU and CPU.
/// 3. We do not need to do additional CPU work such as sorting the samples by sample voxel.
///
/// The base @c GpuMap supports having two batches in flight. It supports a second batch being prepared while the
/// first is allowed to continue. The NDT changes do not support this and there is a sync point before executing
/// the NDT sample update.
///
/// The NDT changes are applied by overriding @c finaliseBatch()
class GpuNdtMap : public GpuMap
{
public:
  /// Create a @c GpuNdtMap around the given @p map representation.
  /// @param map The map to wrap.
  /// @param borrowed_map True to borrow the map, @c false for this object to take ownership.
  /// @param expected_element_count The expected point count for calls to @c integrateRays(). Used as a hint.
  /// @param gpu_mem_size Optionally specify the target GPU cache memory to allocate.
  /// @param ndt_mode Specified which NDT mode to use. Using @c kNone is invalid resulting in undefined behaviour.
  explicit GpuNdtMap(OccupancyMap *map, bool borrowed_map = true, unsigned expected_element_count = 2048u,
                     size_t gpu_mem_size = 0u, NdtMode ndt_mode = NdtMode::kOccupancy);

  /// @overload
  inline GpuNdtMap(OccupancyMap *map, bool borrowed_map, NdtMode mode)
    : GpuNdtMap(map, borrowed_map, 2048u, 0u, mode)
  {}

  /// @overload
  inline GpuNdtMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, NdtMode mode)
    : GpuNdtMap(map, borrowed_map, expected_element_count, 0u, mode)
  {}

  /// Destructor
  ~GpuNdtMap() override;

  /// Set the range sensor noise estimate. For example, the range noise for a lidar sensor.
  ///
  /// @param noise_range The sensor noise range. Must be greater than zero.
  void setSensorNoise(float noise_range);

  /// Read the range sensor noise estimate.
  float sensorNoise() const;

  /// Access the CPU based @c NdtMap wrapper. This should be used with great care as changes to the voxels via
  /// this interface may not be correctly reflected in GPU.
  /// @return The CPU based @c NdtMap
  NdtMap &ndtMap();

  /// Access the CPU based @c NdtMap wrapper.
  /// @return The CPU based @c NdtMap
  const NdtMap &ndtMap() const;

  /// Debug render the NDT map ellipsoids via 3rd Eye Scene.
  void debugDraw() const;

protected:
  /// Helper to access the internal pimpl cast to the correct type.
  GpuNdtMapDetail *detail();
  /// Helper to access the internal pimpl cast to the correct type.
  const GpuNdtMapDetail *detail() const;

  /// Cache the NDT kernel as well.
  /// @param with_voxel_mean True to cache the program which supports voxel mean positioning (@ref voxelmean). Redundant
  ///   for NDT as it is already required.
  /// @param with_traversal True to cache the program which supports the traversal layer.
  /// @param force Force release and program caching even if already correct. Must be used on initialisation.
  void cacheGpuProgram(bool with_voxel_mean, bool with_traversal, bool force) override;

  void finaliseBatch(unsigned region_update_flags) override;

  using TouchedCacheSet = std::array<GpuLayerCache *, 8>;
  void invokeNdtOm(unsigned region_update_flags, int buf_idx, gputil::EventList &wait, TouchedCacheSet &used_caches);
  void invokeNdtTm(unsigned region_update_flags, int buf_idx, gputil::EventList &wait, TouchedCacheSet &used_caches);

  void releaseGpuProgram() override;
};
}  // namespace ohm

#endif  // GPUNDTMAP_H
