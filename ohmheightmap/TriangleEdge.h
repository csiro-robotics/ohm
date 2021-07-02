// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMHEIGHTMAP_TRIANGLEEDGE_H
#define OHMHEIGHTMAP_TRIANGLEEDGE_H

#include "OhmHeightmapConfig.h"

#include <algorithm>

namespace ohm
{
/// A helper for tracking edges in mesh generation. Stores edge vertices such that v0 < v1 regardless of the input
/// ordering. This allows the edge array to be sorted to clump the edge pairings where two triangles share an edge.
/// Only two triangles can share one edge because we build the mesh without T intersections.
struct ohmheightmap_API TriangleEdge
{
  unsigned v0;                   ///< The lower magnitude vertex index.
  unsigned v1;                   ///< The higher magnitude vertex index.
  unsigned triangle;             ///< The index of the triangle which generated this edge.
  unsigned triangle_edge_index;  ///< The index of this edge in @p triangle.

  /// Constructor; v0 and v1 will be reordered such that v0 < v1.
  /// @param v0 First vertex index.
  /// @param v1 Second vertex index.
  /// @param triangle Triangle index.
  /// @param edge_index The index of the edge in @p triangle [0, 2].
  inline TriangleEdge(unsigned v0, unsigned v1, unsigned triangle, unsigned edge_index)
    : triangle(triangle)
    , triangle_edge_index(edge_index)
  {
    this->v0 = std::min(v0, v1);
    this->v1 = std::max(v0, v1);
  }

  /// Comparison operator for sorting.
  inline bool operator<(const TriangleEdge &other) const { return v0 < other.v0 || v0 == other.v0 && v1 < other.v1; }
};
}  // namespace ohm

#endif  // OHMHEIGHTMAP_TRIANGLEEDGE_H
