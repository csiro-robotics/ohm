// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuMemRegion.h"

using namespace gputil;

void MemRegion::mergeRegionList(std::vector<MemRegion> &regions)
{
  if (regions.empty())
  {
    return;
  }

  // Process the dirty list, copying regions back to the device.
  // First sort and merge the dirty list.
  std::sort(regions.begin(), regions.end());

  // Merge regions in the dirty list.
  auto merge_iter = regions.begin();
  for (auto next_iter = merge_iter + 1; merge_iter != regions.end() && next_iter != regions.end();)
  {
    if (!merge_iter->overlaps(*next_iter))
    {
      // Move on to the next block.
      merge_iter = next_iter;
      ++next_iter;
    }
    else
    {
      // Merge the two items.
      merge_iter->merge(*next_iter);
      // Zero out the merged item.
      next_iter->byte_count = 0;
      // Next item
      ++next_iter;
    }
  }
}
