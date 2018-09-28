//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#ifndef PLYMESH_H
#define PLYMESH_H

#include "ohmutilexport.h"

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>
#include <ostream>

#include "colour.h"

/// Represents a mesh which can be saved to Ply. Intended as a debugging device.
class ohmutil_API PlyMesh
{
public:
  static const uint32_t DefaultColour = 0xffffffff;
  typedef glm::vec3 VertexType;

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
  inline unsigned addVertex(const glm::vec3 &vert) { return addVertices(&vert, 1, nullptr); }
  inline unsigned addVertex(const glm::vec3 &vert, const Colour &colour) { return addVertices(&vert, 1, &colour); }

  void setNormal(unsigned vertexIndex, const glm::vec3 &normal);

  void addEdges(const unsigned *edgeIndices, unsigned edgeCount, const Colour *colours = nullptr);
  inline void addEdge(unsigned v0, unsigned v1) { const unsigned vi[2] = { v0, v1 }; return addEdges(vi, 1, nullptr); }
  inline void addEdge(unsigned v0, unsigned v1, const Colour &colour) { const unsigned vi[2] = { v0, v1 }; return addEdges(vi, 1, &colour); }
  void addEdge(const glm::vec3 &v0, const glm::vec3 &v1, const Colour &colour);

  void addTriangles(const unsigned *triangleIndices, unsigned triangleCount, const Colour *colours = nullptr);
  inline void addTriangle(unsigned v0, unsigned v1, unsigned v2) { const unsigned vi[3] = { v0, v1, v2 }; return addTriangles(vi, 1, nullptr); }
  inline void addTriangle(unsigned v0, unsigned v1, unsigned v2, const Colour &colour) { const unsigned vi[3] = { v0, v1, v2 }; return addTriangles(vi, 1, &colour); }
  void addTriangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const Colour &colour);

  void addPolygon(const unsigned *indices, unsigned order, const Colour &colour);
  void addPolygon(const glm::vec3 *verts, unsigned order, const Colour &colour);

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
  /// @param vertIds An array of three vertex Ids for @p verts.
  /// @param colour An optional colour for the face. Single item only, for the face not the vertices.
  void addMappedTriangle(const glm::vec3 *verts, const unsigned *vertIds, const Colour *colour = nullptr);

  void addMappedPolygon(const glm::vec3 *verts, const unsigned *vertIds, unsigned order, const Colour *colour);

  /// Add an edge with support for vertex mapping.
  ///
  /// This function behaves much as @c addMappedTriangle(), but for edges.
  /// @param verts An array of two vertex positions.
  /// @param vertIds An array of two vertex Ids for @p verts.
  /// @param colour An optional colour for the edge. Single item only, for the edge not the vertices.
  void addMappedEdge(const glm::vec3 *verts, const unsigned *vertIds, const Colour *colour = nullptr);
  inline void addMappedEdge(const glm::vec3 *verts, const unsigned *vertIds, const Colour &colour)
  {
    return addMappedEdge(verts, vertIds, &colour);
  }

  void addComment(const char *comment);
  const char *getComment(unsigned index) const;
  unsigned commentCount() const;
  void clearComments();

  bool save(const char *outPath, bool binary) const;
  bool save(FILE *outfile, bool binary) const;
  bool save(std::ostream &out, bool binary) const;

  template <typename T>
  class FileWrapper
  {
  public:
    inline bool isOpen() const { return false; }
    inline void printf(const char *format, ...) { }
    inline void write(const void *ptr, size_t elementSize, size_t elementCount) { }
  };

private:

  template <typename T>
  bool save(FileWrapper<T> &out, bool binary) const;

  struct Vertex
  {
    VertexType point;
    Colour colour;
  };

  struct Edge
  {
    uint32_t v[2];
    Colour colour;
  };

  struct Tri
  {
    uint32_t v[3];
    Colour colour;
  };

  struct Poly
  {
    size_t indicesStart;  ///< Index into _polygonIndices for first index.
    unsigned order;       ///< Number of indices in the polygon (and in _polygonIndices).
    Colour colour;
  };

  std::vector<Vertex> _vertices;
  std::vector<VertexType> _normals;
  std::vector<Edge> _edges;
  std::vector<Tri> _triangles;
  std::vector<Poly> _polygons;
  std::vector<uint32_t> _polygonIndices;  ///< Stores polygon indices.
  std::vector<std::string> _comments;     ///< List of comments to add to the file.
  bool _vertexColours;
  bool _edgeColours;
  bool _faceColours;

  std::unordered_map<unsigned, unsigned> *_indexMapper;
};

#endif  // PLYMESH_H
