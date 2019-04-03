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
    static const uint32_t kDefaultColour = 0xffffffff;
    using VertexType = glm::vec3;

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

    unsigned addVertices(const glm::dvec3 *verts, unsigned count, const Colour *colours = nullptr);
    inline unsigned addVertex(const glm::dvec3 &vert) { return addVertices(&vert, 1, nullptr); }
    inline unsigned addVertex(const glm::dvec3 &vert, const Colour &colour) { return addVertices(&vert, 1, &colour); }

    void setNormal(unsigned vertex_index, const glm::vec3 &normal);
    void setNormal(unsigned vertex_index, const glm::dvec3 &normal);

    void addEdges(const unsigned *edge_indices, unsigned edge_count, const Colour *colours = nullptr);
    inline void addEdge(unsigned v0, unsigned v1)
    {
      const unsigned vi[2] = { v0, v1 };
      return addEdges(vi, 1, nullptr);
    }
    inline void addEdge(unsigned v0, unsigned v1, const Colour &colour)
    {
      const unsigned vi[2] = { v0, v1 };
      return addEdges(vi, 1, &colour);
    }
    void addEdge(const glm::vec3 &v0, const glm::vec3 &v1, const Colour &colour);
    void addEdge(const glm::dvec3 &v0, const glm::dvec3 &v1, const Colour &colour);

    void addTriangles(const unsigned *triangle_indices, unsigned triangle_count, const Colour *colours = nullptr);
    inline void addTriangle(unsigned v0, unsigned v1, unsigned v2)
    {
      const unsigned vi[3] = { v0, v1, v2 };
      return addTriangles(vi, 1, nullptr);
    }
    inline void addTriangle(unsigned v0, unsigned v1, unsigned v2, const Colour &colour)
    {
      const unsigned vi[3] = { v0, v1, v2 };
      return addTriangles(vi, 1, &colour);
    }
    void addTriangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const Colour &colour);
    void addTriangle(const glm::dvec3 &v0, const glm::dvec3 &v1, const glm::dvec3 &v2, const Colour &colour);

    void addPolygon(const unsigned *indices, unsigned order, const Colour &colour);
    void addPolygon(const glm::vec3 *verts, unsigned order, const Colour &colour);
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
    void addMappedTriangle(const glm::dvec3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);

    void addMappedPolygon(const glm::vec3 *verts, const unsigned *vert_ids, unsigned order, const Colour *colour);
    void addMappedPolygon(const glm::dvec3 *verts, const unsigned *vert_ids, unsigned order, const Colour *colour);

    /// Add an edge with support for vertex mapping.
    ///
    /// This function behaves much as @c addMappedTriangle(), but for edges.
    /// @param verts An array of two vertex positions.
    /// @param vert_ids An array of two vertex Ids for @p verts.
    /// @param colour An optional colour for the edge. Single item only, for the edge not the vertices.
    void addMappedEdge(const glm::vec3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);
    inline void addMappedEdge(const glm::vec3 *verts, const unsigned *vert_ids, const Colour &colour)
    {
      return addMappedEdge(verts, vert_ids, &colour);
    }
    void addMappedEdge(const glm::dvec3 *verts, const unsigned *vert_ids, const Colour *colour = nullptr);
    inline void addMappedEdge(const glm::dvec3 *verts, const unsigned *vert_ids, const Colour &colour)
    {
      return addMappedEdge(verts, vert_ids, &colour);
    }

    void addComment(const char *comment);
    const char *getComment(unsigned index) const;
    unsigned commentCount() const;
    void clearComments();

    bool save(const char *out_path, bool binary) const;
    bool save(FILE *outfile, bool binary) const;
    bool save(std::ostream &out, bool binary) const;

    template <typename T>
    class FileWrapper
    {
    public:
      inline bool isOpen() const { return false; }
      inline void printf(const char * /*format*/, ...) {}
      inline void write(const void * /*ptr*/, size_t /*element_size*/, size_t /*element_count*/) {}
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
