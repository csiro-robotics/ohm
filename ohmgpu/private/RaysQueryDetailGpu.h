// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGpuConfig.h"

#include "GpuMapDetail.h"

#include <ohm/private/RaysQueryDetail.h>

namespace ohm
{
struct ohmgpu_API RaysQueryDetailGpu : public RaysQueryDetail
{
  // Ray/key buffer upload event pairs.
  /// Events for key_buffers
  gputil::Event key_upload_event;
  /// Buffers for start/end voxel keys for each ray pair: GpuKey
  gputil::Buffer key_buffer;
  /// Events for ray_buffers
  gputil::Event ray_upload_event;
  /// Buffers of rays to process float3 pairs. Coordinates are local to the centre of the start voxel for each pair.
  gputil::Buffer ray_buffer;

  gputil::Event region_key_upload_event;
  gputil::Buffer region_key_buffer;
  gputil::Event query_event;

  VoxelUploadInfo occupancy_upload_info;
  /// Vector used to group/sort rays when @c group_rays is `true`.
  std::vector<RayItem> upload_rays;

  // output buffers
  gputil::Buffer ranges_buffer;
  gputil::Buffer unobserved_volumes_buffer;
  gputil::Buffer terminal_states_buffer;
  gputil::Event ranges_event;
  gputil::Event unobserved_volumes_event;
  gputil::Event terminal_states_event;

  GpuProgramRef *g_program_ref = nullptr;
  gputil::Kernel query_kernel;

  /// Number of rays (origin/sample pairs) in the ray_buffer.
  unsigned ray_counts = 0;

  /// Set of processing regions.
  GpuMapDetail::RegionSet regions;

  /// Used as @c GpuLayerCache::upload() @c batchMarker argument.
  unsigned batch_marker = 1;  // Will cycle odd numbers to avoid zero.
  bool cached_sub_voxel_program = false;
};
}  // namespace ohm
