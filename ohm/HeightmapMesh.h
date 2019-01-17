// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPMESH_H
#define HEIGHTMAPMESH_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

#include <memory>

namespace ohm
{
  class Aabb;
  class Heightmap;
  class HeightmapMeshDetail;
  class PlyMesh;

  /// Stores the neighbours information for a triangle.
  ///
  /// Identifies the @p neighbour triangle indices (multiply by 3 when indexing HeighmapMesh::triangles()) and the
  /// edge ID of the shared edge in the neighbour. The index into @c neighbours [0, 2] identifies the edge ID in the
  /// current triangle.
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

  class ohm_API HeightmapMesh
  {
  public:
    /// Vertex normal generation mode.
    enum NormalsMode
    {
      /// Average each vertex triangle normals.
      kNormalsAverage,
      /// Select the "worst" triangle normal where "worst" is the least horizontal.
      kNormalsWorst,
    };

    HeightmapMesh(NormalsMode normals_mode = kNormalsAverage);
    ~HeightmapMesh();

    NormalsMode normalsMode();
    void setNormalsMode(NormalsMode mode);

    bool buildMesh(const Heightmap &heightmap);

    /// Get the number of vertices in the generated mesh.
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return The mesh vertex count.
    size_t vertexCount() const;

    /// Get the number of triangles in the generated mesh.
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return The mesh triangle count.
    size_t triangleCount() const;

    /// Get the vertex array of the generated mesh. The vertex count is given by @c vertexCount().
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return A pointer to the vertex array.
    const glm::dvec3 *vertices() const;

    /// Get the vertex normal array of the generated mesh. The vertex count is given by @c vertexCount().
    ///
    /// Note: vertex normals are single precision.
    ///
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return A pointer to the vertex array.
    const glm::vec3 *vertexNormals() const;

    /// Get the triangle normal array of the generated mesh. The triangle count is given by @p triangleCount().
    ///
    /// Note: triangle normals are single precision.
    ///
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return A pointer to the vertex array.
    const glm::vec3 *triangleNormals() const;

    /// Get triangle index array of the generated mesh. The number of triangles is given by @c triangleCount()
    ///
    /// The returned array comes in index triples (three vertices per triangle), so the number of elements in the
    /// returned array is <tt>3 * triangleCount()</tt>.
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return A pointer to the triangle index array.
    const unsigned *triangles() const;

    /// Get per triangle neighbour information. The number of triangles is given by @c triangleCount().
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return A pointer to the triangle index array.
    const TriangleNeighbours *triangleNeighbours() const;

    /// Get the axis aligned bounding box enclosing the resulting mesh.
    ///
    /// Only valid when @c buildMesh() is successful.
    ///
    /// @return The mesh axis aligned bounding box.
    const Aabb &meshBoundingBox() const;

    /// Extract the heightmap mesh into a @c PlyMesh object for saving to file.
    ///
    /// Note the resulting mesh only supports single precision, so the default behaviour is to relocate all vertices
    /// such that they are relative to the @c meshBoundingBox() minimum extents. To preserve the original coordinates,
    /// pass false to @c offset_by_extents.
    ///
    /// @param[in,out] mesh The mesh object to populate. Mesh cleared first.
    /// @param offset_by_extents When true, relocate vertices relative to @c meshBoundingBox() minimum extents.
    /// @return True on success, false when empty.
    bool extractPlyMesh(PlyMesh &mesh, bool offset_by_extents = true);

  private:
    std::unique_ptr<HeightmapMeshDetail> imp_;
  };
}

#endif // HEIGHTMAPMESH_H
