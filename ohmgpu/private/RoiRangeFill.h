// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_ROIRANGEFILL_H
#define OHMGPU_ROIRANGEFILL_H

#include "OhmGpuConfig.h"

#include <glm/glm.hpp>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuKernel.h>

namespace ohm
{
  class OccupancyMap;
  struct OccupancyMapDetail;
  class GpuCache;
  class GpuLayerCache;
  class GpuProgramRef;

  /// GPU algorithm to calculate per voxel clearance values (range to nearest obstruction).
  class RoiRangeFill
  {
  public:
    RoiRangeFill(gputil::Device &gpu);  // NOLINT(google-runtime-references)
    ~RoiRangeFill();

    gputil::Buffer &gpuCornerVoxelKey() { return gpu_corner_voxel_key_; }
    const gputil::Buffer &gpuCornerVoxelKey() const { return gpu_corner_voxel_key_; }

    gputil::Buffer &gpuRegionKeys() { return gpu_region_keys_; }
    const gputil::Buffer &gpuRegionKeys() const { return gpu_region_keys_; }

    gputil::Buffer &gpuOccupancyRegionOffsets() { return gpu_occupancy_region_offsets_; }
    const gputil::Buffer &gpuOccupancyRegionOffsets() const { return gpu_occupancy_region_offsets_; }

    gputil::Buffer &gpuRegionClearanceBuffer() { return gpu_region_clearance_buffer_; }
    const gputil::Buffer &gpuRegionClearanceBuffer() const { return gpu_region_clearance_buffer_; }

    gputil::Buffer &gpuWork(int buffer) { return gpu_work_[buffer]; }
    const gputil::Buffer &gpuWork(int buffer) const { return gpu_work_[buffer]; }

    const gputil::Device &gpu() const { return gpu_; }

    const glm::vec3 &axisScaling() const { return axis_scaling_; }
    void setAxisScaling(const glm::vec3 &scaling) { axis_scaling_ = scaling; }

    float searchRadius() const { return search_radius_; }
    void setSearchRadius(float radius) { search_radius_ = radius; }

    unsigned queryFlags() const { return query_flags_; }
    void setQueryFlags(unsigned flags) { query_flags_ = flags; }

    unsigned regionCount() const { return region_count_; }
    // void setRegionCount(unsigned count) { region_count_ = count; }

    bool calculateForRegion(OccupancyMap &map, const glm::i16vec3 &region_key);  // NOLINT(google-runtime-references)

  private:
    void cacheGpuProgram(bool force);
    void releaseGpuProgram();

    void finishRegion(const glm::i16vec3 &region_key, OccupancyMap &map,  // NOLINT(google-runtime-references)
                      RoiRangeFill &query, GpuCache &gpu_cache,           // NOLINT(google-runtime-references)
                      GpuLayerCache &clearance_cache,                     // NOLINT(google-runtime-references)
                      const glm::ivec3 &batch_voxel_extents, const std::vector<gputil::Event> &upload_events);

    int invoke(const OccupancyMapDetail &map, RoiRangeFill &query,         // NOLINT(google-runtime-references)
               GpuCache &gpu_cache, GpuLayerCache &clearance_layer_cache,  // NOLINT(google-runtime-references)
               const glm::ivec3 &input_data_extents, const std::vector<gputil::Event> &upload_events);

    /// Key for the lower extents corner of the global work group. All other GPU threads can resolve their key by
    /// adjusting this key using their 3D global ID.
    gputil::Buffer gpu_corner_voxel_key_;
    /// Array of keys of the uploaded regions. May be used to resolve indexing to regions and vise-versa.
    gputil::Buffer gpu_region_keys_;
    /// Memory offsets into the GPU cache memory holding voxel occupancy values. Order matches @c gpu_region_keys_.
    gputil::Buffer gpu_occupancy_region_offsets_;
    /// Buffer holding the clearance values for the region we are working on.
    gputil::Buffer gpu_region_clearance_buffer_;
    /// Buffer of int4 used to propagate obstacles.
    gputil::Buffer gpu_work_[2];
    gputil::Device gpu_;
    gputil::Kernel seed_kernel_;
    gputil::Kernel seed_outer_kernel_;
    gputil::Kernel propagate_kernel_;
    gputil::Kernel migrate_kernel_;
    GpuProgramRef *program_ref_ = nullptr;
    glm::vec3 axis_scaling_ = glm::vec3(1.0f);
    float search_radius_ = 0.0f;
    unsigned query_flags_ = 0;
    unsigned region_count_ = 0;
  };
}  // namespace ohm

#endif  // OHMGPU_ROIRANGEFILL_H
