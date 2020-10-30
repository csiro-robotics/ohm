//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#ifndef OHMUTIL_PLYMESH_H
#define OHMUTIL_PLYMESH_H

#include "OhmUtilExport.h"

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "Colour.h"

namespace ohm
{
/// Represents a mesh which can be saved to Ply. Intended as a debugging device.
class ohmutil_API PlyMesh
{
public:
  static const uint32_t kDefaultColour = 0xffffffff;  ///< Default colour (white)
  using VertexType = glm::vec3;                       ///< Vertex typedef.

  /// Constructor.
  PlyMesh();
  /// Typical destructor.
  ~PlyMesh();

  /// Clear the current mesh content. Memory is preserved.
  void clear();

  /// Add an array of vertices to the mesh.
  /// @param verts Array of vertices to add.
  /// @param count Number of vertices to add.
  /// @param colours Optional colours for the vertices. Enables colour export if specified.
  /// @return The index of the first vertex in @p verts after adding.
  unsigned addVertices(const glm::vec3 *verts, unsigned count, const Colour *colours = nullptr);
  /// Add a single vertex to the mesh
  /// @param vert The vertex to add
  /// @return The index of the added vertex.
  inline unsigned addVertex(const glm::vec3 &vert) { return addVertices(&vert, 1, nullptr); }
  /// Add a single coloured vertex to the mesh
  /// @param vert The vertex to add
  /// @param colour Colour value for the vertex.
  /// @return The index of the added vertex.
  inline unsigned addVertex(const glm::vec3 &vert, const Colour &colour) { return addVertices(&vert, 1, &colour); }

  /// @overload
  unsigned addVertices(const glm::dvec3 *verts, unsigned count, const Colour *colours = nullptr);
  /// @overload
  inline unsigned addVertex(const glm::dvec3 &vert) { return addVertices(&vert, 1, nullptr); }
  /// @overload
  inline unsigned addVertex(const glm::dvec3 &vert, const Colour &colour) { return addVertices(&vert, 1, &colour); }

  /// Set the normal for a vertex in the mesh.
  /// @param vertex_index Index of the vertex to add a normal for. Must be in range.
  /// @param normal The vertex normal value.
  void setNormal(unsigned vertex_index, const glm::vec3 &normal);
  /// @overload
  void setNormal(unsigned vertex_index, const glm::dvec3 &normal);

  /// Add edge data to the ply mesh
  /// @param edge_indices Vertex index pairs for the set of edges to add. Expects `edge_count * 2` elements.
  /// Vertex indices must be in range.
  /// @param edge_count Number of edges being added.
  /// @param colours Colour values for the added edges. Expects @p edge_count items or null to not add edge colours.
  void addEdges(const unsigned *edge_indices, unsigned edge_count, const Colour *colours = nullptr);
  /// @overload
  inline void addEdge(unsigned v0, unsigned v1)
  {
    const unsigned vi[2] = { v0, v1 };
    return addEdges(vi, 1, nullptr);
  }
  /// Add a single edge to the mesh
  /// @param v0 Index of the first vertex in the edge.
  /// @param v1 Index of the second vertex in the edge.
  /// @param colour Colour value for the edge.
  inline void addEdge(unsigned v0, unsigned v1, const Colour &colour)
  {
    const unsigned vi[2] = { v0, v1 };
    return addEdges(vi, 1, &colour);
  }
  /// Add two vertices to the mesh and connect them as a coloured edge
  /// @param v0 First vertex in the edge.
  /// @param v1 Second vertex in the edge
  /// @param colour Colour value for the edge.
  void addEdge(const glm::vec3 &v0, const glm::vec3 &v1, const Colour &colour);
  /// @overload
  void addEdge(const glm::dvec3 &v0, const glm::dvec3 &v1, const Colour &colour);

  /// Add indexed triangles to the mesh
  /// @param triangle_indices Triangle index triples for the triangles to add. Expects `triangle_count * 3` elements.
  /// Vertex indices must be in range.
  /// @param triangle_count Number of triangles being added.
  /// @param colours Colour values for the added triangles. Expects @p triangle_count items or null to not add edge
  /// colours.
  void addTriangles(const unsigned *triangle_indices, unsigned triangle_count, const Colour *colours = nullptr);
  /// @overload
  inline void addTriangle(unsigned v0, unsigned v1, unsigned v2)
  {
    const unsigned vi[3] = { v0, v1, v2 };
    return addTriangles(vi, 1, nullptr);
  }
  /// Add a single triangle to the mesh
  /// @param v0 Index of the first vertex in the triangle.
  /// @param v1 Index of the second vertex in the triangle.
  /// @param v2 Index of the third vertex in the triangle.
  /// @param colour Colour value for the triangle.
  inline void addTriangle(unsigned v0, unsigned v1, unsigned v2, const Colour &colour)
  {
    const unsigned vi[3] = { v0, v1, v2 };
    return addTriangles(vi, 1, &colour);
  }
  /// Add three vertices to the mesh and connect them as a coloured triangle
  /// @param v0 First vertex in the triangle.
  /// @param v1 Second vertex in the triangle.
  /// @param v2 Third vertex in the triangle.
  /// @param colour Colour value for triangle.
  void addTriangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const Colour &colour);
  /// @overload
  void addTriangle(const glm::dvec3 &v0, const glm::dvec3 &v1, const glm::dvec3 &v2, const Colour &colour);

  /// Add an indexed polygon to the mesh.
  /// @param indices Vertex indices for the polygon. The expected number of elements at this address is determined by
  /// the value of @p order . Vertex indices must be in range.
  /// @param order The number of vertices in the polygon, therefore the number of elements at @p indices .
  /// @param colour Colour value for the polygon.
  void addPolygon(const unsigned *indices, unsigned order, const Colour &colour);
  /// Add a polygon to the mesh adding new vertices for the polygon at the same time.
  /// @param verts Vertex positions for the polygon. The expected number of elements at this address is determined by
  /// the value of @p order .
  /// @param order The number of vertices in the polygon, therefore the number of elements at @p indices .
  /// @param colour Colour value for the polygon.
  void addPolygon(const glm::vec3 *verts, unsigned order, const Colour &colour);
  /// @overload
  void addPolygon(const glm::dvec3 *verts, unsigned order, const Colour &colour);

  /// Add a triangle with support for vertex mapping.
  ///
  /// This function adds the triangle defined by @p verts, but re-uses existing vertices.
  /// A previously added vertex is used if a vertex with an id matching @p vertIds[n] exists.
  /// In this case the previous value is used and the value for @p vert[n] is ignored.
  ///
  /// This can be used to add parts from another indexed mesh. Triangles and vertices can be added
  /// from the original mesh using this function rather than added vertices and triangles separately.
  /// The @p vertIds will be the indices from the original mesh.
  ///
  /// @param verts An array of three vertex positions.
  /// @param vert_ids An array of three vertex Ids for @p verts.
  /// @param colour An optional colour for the face. Single item only, for the face not the vertices.
  void addMappedTriangle(const glm::vec3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);
  /// @overload
  void addMappedTriangle(const glm::dvec3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);

  /// Similar to @c addMappedTriangle() supporting arbitrary order polygons.
  /// @param verts An array of vertex positions. The expected number of elements at this address is determined by
  /// the value of @p order .
  /// @param vert_ids An array of vertex Ids for @p verts.
  /// @param order The number of vertices in the polygon, therefore the number of elements at @p indices .
  /// @param colour An optional colour for the face. Single item only, for the face not the vertices.
  void addMappedPolygon(const glm::vec3 *verts, const unsigned *vert_ids, unsigned order, const Colour *colour);
  /// @overload
  void addMappedPolygon(const glm::dvec3 *verts, const unsigned *vert_ids, unsigned order, const Colour *colour);

  /// Add an edge with support for vertex mapping.
  ///
  /// This function behaves much as @c addMappedTriangle(), but for edges.
  /// @param verts An array of two vertex positions.
  /// @param vert_ids An array of two vertex Ids for @p verts.
  /// @param colour An optional colour for the edge. Single item only, for the edge not the vertices.
  void addMappedEdge(const glm::vec3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);
  /// @overload
  inline void addMappedEdge(const glm::vec3 *verts, const unsigned *vert_ids, const Colour &colour)
  {
    return addMappedEdge(verts, vert_ids, &colour);
  }
  /// @overload
  void addMappedEdge(const glm::dvec3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);
  /// @overload
  inline void addMappedEdge(const glm::dvec3 *verts, const unsigned *vert_ids, const Colour &colour)
  {
    return addMappedEdge(verts, vert_ids, &colour);
  }

  /// Add a comment to the PLY file. Comment will be written on calling @c save() .
  /// @param comment The comment string.
  void addComment(const char *comment);
  /// Get a previously added comment string.
  /// @param index Index of the comment string. Must be in range `[0, commentCount())`
  /// @return The requested comment string.
  const char *getComment(unsigned index) const;
  /// Query the number of comment strings which have been added using @c addComment() .
  /// @return The number of added comment strings.
  unsigned commentCount() const;
  /// Remove all added comments.
  void clearComments();

  /// Save the PLY mesh to the given @p out_path .
  /// @param out_path File path specifying where to save. String should include the `.ply` extension.
  /// @param binary Save in binary PLY format? False for ASCII.
  /// @return True on succcess.
  bool save(const char *out_path, bool binary) const;
  /// Save the PLY mesh using a @c FILE object.
  /// @param outfile @c FILE object to save to.
  /// @param binary Save in binary PLY format? False for ASCII.
  /// @return True on succcess.
  bool save(FILE *outfile, bool binary) const;
  /// Save the PLY mesh to the given @c stream .
  /// @param stream The output stream to save to.
  /// @param binary Save in binary PLY format? False for ASCII.
  /// @return True on succcess.
  bool save(std::ostream &stream, bool binary) const;

  /// API adaptor for either @c std::ostream or @c FILE .
  ///
  /// Specialisations are internal. Declaration is here to support the declarations below.
  template <typename T>
  class FileWrapper
  {
  public:
    /// Check if the file open?
    /// @return True if open.
    inline bool isOpen() const { return false; }
    /// Write text to the file using @c printf() style formatting.
    /// @param format @c printf format string
    inline void printf(const char *format, ...) {}
    /// Write binary data to the file.
    /// @param ptr Pointer to the data to write.
    /// @param element_size Size of a single item at @p ptr
    /// @param element_count Number of elements at @p ptr to write.
    inline void write(const void *ptr, size_t element_size, size_t element_count) {}
  };

private:
  template <typename T>
  bool save(FileWrapper<T> &out, bool binary) const;

  template <typename VEC3>
  unsigned addVerticesT(const VEC3 *verts, unsigned count, const Colour *colours);

  template <typename VEC3>
  void setNormalT(unsigned vertex_index, const VEC3 &normal);

  template <typename VEC3>
  void addEdgeT(const VEC3 &v0, const VEC3 &v1, const Colour &colour);

  template <typename VEC3>
  void addTriangleT(const VEC3 &v0, const VEC3 &v1, const VEC3 &v2, const Colour &colour);

  template <typename VEC3>
  void addPolygonT(const VEC3 *verts, unsigned order, const Colour &colour);

  template <typename VEC3>
  void addMappedTriangleT(const VEC3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);
  template <typename VEC3>
  void addMappedPolygonT(const VEC3 *verts, const unsigned *vert_ids, unsigned order, const Colour *colour);
  template <typename VEC3>
  void addMappedEdgeT(const VEC3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);

  struct Vertex  // NOLINT(cppcoreguidelines-pro-type-member-init)
  {
    VertexType point;
    Colour colour;
  };

  struct Edge  // NOLINT(cppcoreguidelines-pro-type-member-init)
  {
    uint32_t v[2];
    Colour colour;
  };

  struct Tri  // NOLINT(cppcoreguidelines-pro-type-member-init)
  {
    uint32_t v[3];
    Colour colour;
  };

  struct Poly  // NOLINT(cppcoreguidelines-pro-type-member-init)
  {
    size_t indices_start;  ///< Index into _polygonIndices for first index.
    unsigned order;        ///< Number of indices in the polygon (and in _polygonIndices).
    Colour colour;
  };

  std::vector<Vertex> vertices_;
  std::vector<VertexType> normals_;
  std::vector<Edge> edges_;
  std::vector<Tri> triangles_;
  std::vector<Poly> polygons_;
  std::vector<uint32_t> polygon_indices_;  ///< Stores polygon indices.
  std::vector<std::string> comments_;      ///< List of comments to add to the file.
  bool vertex_colours_;
  bool edge_colours_;
  bool face_colours_;

  std::unordered_map<unsigned, unsigned> *index_mapper_;
};

}  // namespace ohm

#endif  // OHMUTIL_PLYMESH_H
