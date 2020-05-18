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

  class GpuNdtMap : public GpuMap
  {
  public:
    GpuNdtMap(OccupancyMap *map, bool borrowed_map = true, unsigned expected_element_count = 2048,
              size_t gpu_mem_size = 0u);

    const NdtMap &ndtMap() const;

    size_t integrateRays(const glm::dvec3 *rays, size_t element_count,
                         unsigned region_update_flags = kRfDefault) override;

  protected:
    GpuNdtMapDetail *detail();
    const GpuNdtMapDetail *detail() const;

    void cacheGpuProgram(bool with_voxel_mean, bool force) override;

    void finaliseBatch(unsigned region_update_flags) override;
    void releaseGpuProgram() override;
  };
}  // namespace ohm

#endif  // GPUNDTMAP_H
