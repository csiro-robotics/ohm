// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MEM_REGION_H
#define MEM_REGION_H

#include "gpuConfig.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace gputil
{
  class MemRegion
  {
  public:
    size_t offset;
    size_t byte_count;

    inline bool operator<(const MemRegion &other) const
    {
      return offset < other.offset || offset == other.offset && byte_count < other.byte_count;
    }


    inline bool overlaps(const MemRegion &other) const
    {
      const size_t start_a = offset;
      const size_t end_a = offset + byte_count;
      const size_t start_b = other.offset;
      const size_t end_b = other.offset + other.byte_count - 1;

      // Overlap if any end point falls within the other region.
      // Only test non-zero regions.
      return byte_count && other.byte_count && (start_a <= start_b && start_b <= end_a) ||
             (start_b <= start_a && start_a <= end_b) || (start_a <= end_b && end_b <= end_a) ||
             (start_b <= end_a && end_a <= end_b);
    }

    inline MemRegion &merge(const MemRegion &other)
    {
      const size_t end_a = offset + byte_count;
      const size_t end_b = other.offset + other.byte_count;
      offset = std::min(offset, other.offset);
      byte_count = std::max(end_a, end_b) - offset;
      return *this;
    }

    /// Sorts and merged the list @p regions. The merged regions are reduced to zero size, but left in the list.
    /// @param regions The list to sort and merge.
    static void mergeRegionList(std::vector<MemRegion> &regions);  // NOLINT(google-runtime-references)
  };
}  // namespace gputil

#endif  // MEM_REGION_H
