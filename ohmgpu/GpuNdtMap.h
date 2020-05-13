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
  struct GpuNdtMapDetail;

  class GpuNdtMap : public GpuMap
  {
  public:
    GpuNdtMap(OccupancyMap *map, bool borrowed_map = true, unsigned expected_element_count = 2048,
              size_t gpu_mem_size = 0u);

  protected:
    GpuNdtMapDetail *detail();
    const GpuNdtMapDetail *detail() const;

    void finaliseBatch(unsigned region_update_flags) override;
  };
}  // namespace ohm

#endif  // GPUNDTMAP_H
