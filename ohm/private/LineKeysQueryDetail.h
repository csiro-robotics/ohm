
// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_LINEKEYSQUERYDETAIL_H_
#define OHM_LINEKEYSQUERYDETAIL_H_

#include "OhmConfig.h"

#include "QueryDetail.h"

namespace ohm
{
  /// Pimpl data for @c LineKeysQuery
  struct ohm_API LineKeysQueryDetail : QueryDetail
  {
    std::vector<glm::dvec3> rays;  ///< Ray origin/end point pairs to query for.
    /// Results vector, identifying the offsets for each ray into @c intersected_voxels where the results for that ray
    /// begin.
    std::vector<size_t> result_indices;
    /// Results vector, indentifying the number of voxels for each ray in @c intersected_voxels.
    std::vector<size_t> result_counts;
  };
}  // namespace ohm

#endif  // OHM_LINEKEYSQUERYDETAIL_H_
