// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_TRIANGLENEIGHBOURS_H
#define OHM_TRIANGLENEIGHBOURS_H

#include "OhmConfig.h"

#include <cstdint>

namespace ohm
{
  /// Stores the neighbours information for a triangle.
  ///
  /// Identifies the @p neighbour triangle indices (multiply by 3 when indexing HeighmapMesh::triangles()) and the
  /// edge ID of the shared edge in the neighbour. The index into @c neighbours [0, 2] identifies the edge ID in the
  /// current triangle.
  ///
  /// @todo Move into its own file.
  struct ohm_API TriangleNeighbours
  {
    /// Indices to the neighbouring triangles.
    /// ~0u for each open edge (-1).
    ///
    /// Ordered by the shared edge as {v[0], v[1]}, {v[1], v[2]}, {v[2], v[0]}.
    unsigned neighbours[3];
    /// Identifies the shared edge indices in each neighbour triangle. For example, @c neighbour_edge_indices[0]
    /// contains edge information for neighbour[0]. The value of [0, 2] indicates which vertex pairing in neighbour[0]
    /// identifies the shared edge. -1 is used for empty edges.
    int8_t neighbour_edge_indices[3];
  };
}

#endif // OHM_TRIANGLENEIGHBOURS_H
