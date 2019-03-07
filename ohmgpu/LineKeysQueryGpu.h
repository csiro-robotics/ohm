// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_LINEKEYSQUERY_H
#define OHMGPU_LINEKEYSQUERY_H

#include "OhmGpuConfig.h"

#include <ohm/LineKeysQuery.h>

#include <glm/fwd.hpp>

namespace ohm
{
  struct LineKeysQueryDetailGpu;

  /// This query calculates the voxels intersecting a batch of rays.
  ///
  /// The results are similar to those of @c OccupancyMap::calculateSegmentKeys() (identical when using CPU),
  /// but supports batched and GPU based calculation. The GPU calculation is generally only marginally faster
  /// than CPU, but GPU supports @c executeAsync(), which is not supported for CPU.
  ///
  /// In practice, this query isn't very useful as the GPU performance gains are minimal.
  ///
  /// General usage is:
  /// - Initialise the query object setting the map and the GPU flag if required.
  /// - Call @c setRays() to define the ray start/end point pairs.
  /// - Call @c execute() or @c executeAsync() followed by @c wait() (GPU only).
  /// - Process results (see below).
  ///
  /// The @c numberOfResults() will match the number of rays (@c pointCount given to @c setRays()) and for
  /// each of these results, there is an entry in @c resultIndices() and @c resultCounts(). The indices indicate the
  /// offsets into @c intersectedVoxels() for each ray in the same order they appear in @c setRays(). The
  /// counts identify how many voxels are present for the current line counting from the associated index.
  /// There should always be at least one voxel per ray for the start/end voxel. More generally, the first voxel
  /// is the ray start voxel and the last voxel is the ray end voxel.
  class ohmgpu_API LineKeysQueryGpu : public LineKeysQuery
  {
  protected:
    /// Constructor used for inherited objects. This supports deriving @p LineKeysQueryDetail into
    /// more specialised forms.
    /// @param detail pimple style data structure. When null, a @c LineKeysQueryDetail is allocated by
    /// this method.
    LineKeysQueryGpu(LineKeysQueryDetailGpu *detail);

  public:
    /// Construct a new query using the given parameters.
    /// @param map The map to operate on. Only the voxel resolution and region sizes are used.
    /// @param query_flags Flags controlling the query behaviour. See @c QueryFlag and @c LineKeysQuery::Flag.
    LineKeysQueryGpu(ohm::OccupancyMap &map, unsigned query_flags = 0u);

    /// Construct a new query using the given parameters.
    /// @param query_flags Flags controlling the query behaviour. See @c QueryFlag and @c LineKeysQuery::Flag.
    LineKeysQueryGpu(unsigned query_flags = 0);

    /// Destructor.
    ~LineKeysQueryGpu();
    
  protected:
    bool onExecute() override;
    bool onExecuteAsync() override;
    void onReset(bool hard_reset) override;
    bool onWaitAsync(unsigned timeout_ms) override;

    LineKeysQueryDetailGpu *imp();
    const LineKeysQueryDetailGpu *imp() const;
  };
}  // namespace ohm


#endif  // OHMGPU_LINEKEYSQUERY_H
