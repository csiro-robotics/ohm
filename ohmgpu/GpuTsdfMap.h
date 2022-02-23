// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUTSDFMAP_H
#define GPUTSDFMAP_H

#include "OhmGpuConfig.h"

#include "GpuMap.h"

#include <ohm/VoxelTsdf.h>

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
struct GpuTsdfMapDetail;

/// A GPU implementation of a TSDF algorithm.
///
/// The TSDF algorithm requires voxels to have @c VoxelTsdf but does not require any occupancy data.
///
/// Note: @c GpuTsdfMap only respects the following @c RayFlag values:
/// - @c kRfForwardWalk
class ohmgpu_API GpuTsdfMap : public GpuMap
{
public:
  /// Create a @c GpuTsdfMap around the given @p map representation.
  /// @param map The map to wrap.
  /// @param borrowed_map True to borrow the map, @c false for this object to take ownership.
  /// @param expected_element_count The expected point count for calls to @c integrateRays(). Used as a hint.
  /// @param gpu_mem_size Optionally specify the target GPU cache memory to allocate.
  /// @param ndt_mode Specified which NDT mode to use. Using @c kNone is invalid resulting in undefined behaviour.
  explicit GpuTsdfMap(OccupancyMap *map, bool borrowed_map = true, unsigned expected_element_count = 2048u,
                      size_t gpu_mem_size = 0u);

  /// @overload
  inline GpuTsdfMap(OccupancyMap *map, bool borrowed_map)
    : GpuTsdfMap(map, borrowed_map, 2048u, 0u)
  {}

  /// @overload
  inline GpuTsdfMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count)
    : GpuTsdfMap(map, borrowed_map, expected_element_count, 0u)
  {}

  /// Destructor
  ~GpuTsdfMap() override;

  void setTsdfOptions(const TsdfOptions &options);
  const TsdfOptions &tsdfOptions() const;

  void setMaxWeight(float max_weight);
  float maxWeight() const;
  void setDefaultTruncationDistance(float default_truncation_distance);
  float defaultTruncationDistance() const;
  void setDropoffEpsilon(float dropoff_epsilon);
  float dropoffEpsilon() const;
  void setSparsityCompensationFactor(float sparsity_compensation_factor);
  float sparsityCompensationFactor() const;

protected:
  /// Helper to access the internal pimpl cast to the correct type.
  GpuTsdfMapDetail *detail();
  /// Helper to access the internal pimpl cast to the correct type.
  const GpuTsdfMapDetail *detail() const;

  /// Cache the NDT kernel as well.
  /// @param with_voxel_mean Ignored.
  /// @param with_traversal Ignored.
  /// @param force Force release and program caching even if already correct. Must be used on initialisation.
  void cacheGpuProgram(bool with_voxel_mean, bool with_traversal, bool force) override;

  void finaliseBatch(unsigned region_update_flags) override;
};
}  // namespace ohm

#endif  // GPUTSDFMAP_H
