// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMHEIGHTMAP_HEIGHTMAPMESH_H
#define OHMHEIGHTMAP_HEIGHTMAPMESH_H

#include "OhmHeightmapConfig.h"

#include "HeightmapVoxelType.h"

#include <glm/fwd.hpp>

#include <algorithm>
#include <functional>
#include <memory>

namespace ohm
{
class Aabb;
class Heightmap;
class HeightmapMeshDetail;
class Key;
class PlyMesh;
struct TriangleNeighbours;

/// A utility class for generating a triangle mesh from a @c Heightmap occupancy map.
///
/// The mesh is generated in @c buildMesh() and provides the following data:
/// - Number of vertices
/// - Number of triangles (number of indices is 3 * triangleCount())
/// - Double precision vertices
/// - Single precision per vertex normals, generated by @c NormalsMode
/// - Single precision per triangle normals.
/// - Mesh axis aligned bounds.
/// - Triangle neighbour information: see @c TriangleNeighbours.
class ohmheightmap_API HeightmapMesh
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

  /// Optional modifier function which can be applied to a voxel before moving it the mesh.
  /// @param voxel The current heightmap voxel.
  /// @param voxel_type The initial voxel type.
  /// @param voxel_position The voxel position. May be modified.
  /// @param clearance The voxel overhead clearance. May be modified.
  /// @return The @c HeightmapVoxelType after filtering or @p voxel_type if no modification is required.
  using MeshVoxelModifier = std::function<HeightmapVoxelType(const Key &, HeightmapVoxelType, glm::dvec3 *, float *)>;

  /// Construct a heightmap mesh using the given vertex normal generation mode.
  /// @param normals_mode The initial vertex generation mode.
  explicit HeightmapMesh(NormalsMode normals_mode = kNormalsAverage);

  /// Destructor.
  ~HeightmapMesh();

  /// Query the active vertex normal generation mode.
  /// @return The vertex normal mode.
  NormalsMode normalsMode();

  /// Set the active vertex normal generation mode.
  /// @param mode The new mode.
  void setNormalsMode(NormalsMode mode);

  /// Build a mesh from @p heightmap. This clears previous data.
  ///
  /// Possible failure conditions:
  /// - The heightmap occupancy map does not contain the required voxel layers.
  ///
  /// @param heightmap The heightmap to generate a mesh for.
  /// @param voxel_modifier Optional modifier function applied to each voxel before moving it to the mesh.
  /// @return True on success.
  bool buildMesh(const Heightmap &heightmap, const MeshVoxelModifier &voxel_modifier = MeshVoxelModifier());

  /// Get the voxel resolution fo the heightmap from which the last mesh was generated.
  /// @return The heightmap voxel resolution.
  double resolution() const;

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

  /// Get the axis aligned bounding box enclosing the resulting mesh and the 2D grid of voxels which generated it.
  /// The box will be slightly larger than the mesh to enclose the 2D voxel grid which generated it. The height axis
  /// tightly encloses the vertices used to generate the mesh.
  ///
  /// Only valid when @c buildMesh() is successful.
  ///
  /// @return The mesh axis aligned bounding box.
  const Aabb &meshBoundingBox() const;

  /// Get the axis aligned bounding box which tightly encloses the vertices of the mesh. Whereas @c meshBoundingBox()
  /// encloses the generating voxels, this version tightly encompasses the vertices.
  ///
  /// Only valid when @c buildMesh() is successful.
  ///
  /// @return The mesh axis aligned bounding box.
  const Aabb &tightMeshBoundingBox() const;

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
}  // namespace ohm

#endif  // OHMHEIGHTMAP_HEIGHTMAPMESH_H
