// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_QUERYDETAIL_H
#define OHM_QUERYDETAIL_H

#include "OhmConfig.h"

#include "ohm/Key.h"

#include <limits>
#include <vector>

namespace ohm
{
  class OccupancyMap;

  /// Pimpl data for @c Query objects .
  struct ohm_API QueryDetail
  {
    /// The map to perform the query on.
    OccupancyMap *map = nullptr;
    /// The voxels intersected by the query, identified by their @c Key (if used).
    std::vector<Key> intersected_voxels;
    /// Distances associated with the @c intersected_voxels (if used).
    std::vector<float> ranges;
    /// Number of results in @c intersected_voxels and/or @c ranges .
    size_t number_of_results = 0;
    /// @c QueryFlag values for the query.
    unsigned query_flags = 0;

    /// Virtual destructor.
    virtual ~QueryDetail() = default;
  };

  /// Query result helper identifying the closest result index.
  struct ohm_API ClosestResult
  {
    size_t index = 0;                                 ///< Index into the results arrays.
    float range = std::numeric_limits<float>::max();  ///< (Closest) range value.
  };
}  // namespace ohm

#endif  // OHM_QUERYDETAIL_H
