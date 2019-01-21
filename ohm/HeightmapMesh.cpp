// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapMesh.h"

// Must come first for various include issues.
#include "3rdparty/delaunator.hpp"

#include <ohm/Aabb.h>
#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Voxel.h>

#include <ohmutil/PlyMesh.h>
#include <ohmutil/Profile.h>

#include <glm/ext.hpp>
#include <glm/glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include <algorithm>
#include <vector>

using namespace ohm;

namespace ohm
{
  /// A helper for tracking edges in mesh generation. Stores edge vertices such that v0 < v1 regardless of the input
  /// ordering. This allows the edge array to be sorted to clump the edge pairings where two triangles share an edge.
  /// Only two triangles can share one edge because we build the mesh without T intersections.
  struct Edge
  {
    unsigned v0;                   ///< The lower magnitude vertex index.
    unsigned v1;                   ///< The higher magnitude vertex index.
    unsigned triangle;             ///< The index of the triangle which generated this edge.
    unsigned triangle_edge_index;  ///< The index of this edge in @p triangle.

    inline Edge(unsigned v0, unsigned v1, unsigned triangle, unsigned edge_index)
      : triangle(triangle)
      , triangle_edge_index(edge_index)
    {
      this->v0 = std::min(v0, v1);
      this->v1 = std::max(v0, v1);
    }

    inline bool operator<(const Edge &other) const { return v0 < other.v0 || v0 == other.v0 && v1 < other.v1; }
  };

  class HeightmapMeshDetail
  {
  public:
    std::vector<glm::dvec3> vertices;
    std::vector<glm::vec3> vertex_normals;
    std::vector<glm::vec3> tri_normals;
    std::vector<unsigned> triangles;
    std::vector<TriangleNeighbours> triangle_neighbours;
    std::vector<Edge> edges;
    std::vector<double> coords_2d;
    HeightmapMesh::NormalsMode normals_mode = HeightmapMesh::kNormalsAverage;
    Aabb mesh_extents = Aabb(0.0);

    void clear()
    {
      vertices.clear();
      vertex_normals.clear();
      tri_normals.clear();
      triangles.clear();
      triangle_neighbours.clear();
      edges.clear();
      coords_2d.clear();
      mesh_extents = Aabb(0.0);
    }
  };
}  // namespace ohm


HeightmapMesh::HeightmapMesh(NormalsMode normals_mode)
  : imp_(new HeightmapMeshDetail)
{
  imp_->normals_mode = normals_mode;
}


HeightmapMesh::~HeightmapMesh()
{}


HeightmapMesh::NormalsMode HeightmapMesh::normalsMode()
{
  return imp_->normals_mode;
}


void HeightmapMesh::setNormalsMode(NormalsMode mode)
{
  imp_->normals_mode = mode;
}


bool HeightmapMesh::buildMesh(const Heightmap &heightmap)
{
  PROFILE(HeightmapMesh_buildMesh);
  // Populate the coordinates.
  imp_->clear();

  // Walk heightmap voxels.
  const OccupancyMap &heightmap_occupancy = heightmap.heightmap();
  const MapLayer *heightmap_layer = heightmap_occupancy.layout().layer(HeightmapVoxel::kHeightmapLayer);
  const int heightmap_layer_index = heightmap_layer->layerIndex();

  if (heightmap_layer_index < 0)
  {
    // Fail.
    return false;
  }

  const glm::dvec3 up = heightmap.upAxisNormal();
  const glm::vec3 upf(up);
  glm::dvec3 point;
  glm::dvec3 min_map_ext, max_map_ext;

  min_map_ext = glm::dvec3(std::numeric_limits<double>::max());
  max_map_ext = glm::dvec3(-std::numeric_limits<double>::max());

  // Build vertices and map extents.
  for (auto voxel_iter = heightmap_occupancy.begin(); voxel_iter != heightmap_occupancy.end(); ++voxel_iter)
  {
    const VoxelConst &voxel = *voxel_iter;
    if (voxel.isOccupied())
    {
      const ohm::HeightmapVoxel *height_info = voxel.layerContent<const ohm::HeightmapVoxel *>(heightmap_layer_index);
      point = voxel.position() + double(height_info->height) * up;
      imp_->coords_2d.push_back(point.x);
      imp_->coords_2d.push_back(point.y);
      imp_->vertices.push_back(point);

      // Adjust to modified extents with height.
      min_map_ext.x = std::min(point.x, min_map_ext.x);
      min_map_ext.y = std::min(point.y, min_map_ext.y);
      min_map_ext.z = std::min(point.z, min_map_ext.z);

      max_map_ext.x = std::max(point.x, max_map_ext.x);
      max_map_ext.y = std::max(point.y, max_map_ext.y);
      max_map_ext.z = std::max(point.z, max_map_ext.z);
    }
  }

  if (imp_->vertices.empty())
  {
    min_map_ext = max_map_ext = glm::dvec3(0.0);
  }

  imp_->mesh_extents = Aabb(min_map_ext, max_map_ext);

  // Triangulate using Delaunay triangulation.
  delaunator::Delaunator delaunay(imp_->coords_2d);

  // Prime vertex normal calculations.
  imp_->vertex_normals.clear();
  imp_->vertex_normals.reserve(imp_->vertices.size());
  for (size_t i = 0; i < imp_->vertices.size(); ++i)
  {
    imp_->vertex_normals.push_back(glm::vec3(0.0f));
  }

  // Extract Delaunay triangles into imp_->triangles.
  imp_->triangles.resize(delaunay.triangles.size());
  if (!delaunay.triangles.empty())
  {
    imp_->triangles.clear();
    imp_->triangles.reserve(delaunay.triangles.size());
    imp_->tri_normals.reserve(delaunay.triangles.size() / 3);
    imp_->triangle_neighbours.reserve(delaunay.triangles.size() / 3);
    imp_->edges.reserve(delaunay.triangles.size());

    glm::dvec3 tri[3];
    glm::vec3 normal;
    unsigned indices[3];
    unsigned tri_count = 0;
    TriangleNeighbours neighbour_info;

    // Initialise empty neighbour information.
    neighbour_info.neighbours[0] = neighbour_info.neighbours[1] = neighbour_info.neighbours[2] = ~0u;
    neighbour_info.neighbour_edge_indices[0] = neighbour_info.neighbour_edge_indices[1] =
      neighbour_info.neighbour_edge_indices[2] = -1;

    for (size_t i = 0; i < delaunay.triangles.size(); i += 3)
    {
      indices[0] = unsigned(delaunay.triangles[i + 0]);
      indices[1] = unsigned(delaunay.triangles[i + 1]);
      indices[2] = unsigned(delaunay.triangles[i + 2]);
      tri[0] = imp_->vertices[indices[0]];
      tri[1] = imp_->vertices[indices[1]];
      tri[2] = imp_->vertices[indices[2]];

      // Calculate the triangle normal.
      normal = glm::triangleNormal(tri[0], tri[1], tri[2]);

      // Adjust winding to match the heightmap axis.
      if (glm::dot(normal, glm::vec3(up)) < 0)
      {
        std::swap(indices[1], indices[2]);
        normal *= -1.0f;
      }

      imp_->triangles.push_back(indices[0]);
      imp_->triangles.push_back(indices[1]);
      imp_->triangles.push_back(indices[2]);

      imp_->triangle_neighbours.push_back(neighbour_info);

      imp_->edges.push_back(Edge(indices[0], indices[1], tri_count, 0));
      imp_->edges.push_back(Edge(indices[1], indices[2], tri_count, 1));
      imp_->edges.push_back(Edge(indices[2], indices[0], tri_count, 2));

      // Vertex normals generated by considering all faces.
      if (imp_->normals_mode == kNormalsAverage)
      {
        imp_->vertex_normals[indices[0]] += normal;
        imp_->vertex_normals[indices[1]] += normal;
        imp_->vertex_normals[indices[2]] += normal;
      }
      else if (imp_->normals_mode == kNormalsWorst)
      {
        // Vertex normals by least horizontal.
        for (int j = 0; j < 3; ++j)
        {
          const glm::vec3 existing_normal = imp_->vertex_normals[indices[j]];
          const float existing_dot = glm::dot(existing_normal, upf);
          const float new_dot = glm::dot(normal, upf);
          if (existing_normal == glm::vec3(0.0f) || existing_dot > new_dot)
          {
            // No existing normal or existing is more horizontal. Override.
            imp_->vertex_normals[indices[j]] = normal;
          }
        }
      }

      imp_->tri_normals.push_back(normal);
      ++tri_count;
    }

    if (!imp_->edges.empty())
    {
      // Use the vertex to edges to build the triangle neighbour information.
      // First sort to ensure all triangle mappings for a vertex are adjacent.
      std::sort(imp_->edges.begin(), imp_->edges.end());

      // Edges should end up paired and we know from how the mesh is built that there can only be either one or two
      // triangles per edge.
      const Edge *previous_edge = &imp_->edges[0];
      const Edge *current_edge;
      for (size_t i = 1; i < imp_->edges.size(); ++i)
      {
        current_edge = &imp_->edges[i];
        if (current_edge->v0 == previous_edge->v0 && current_edge->v1 == previous_edge->v1)
        {
          // We have a shared edge. Update neighbour information for both triangles.
          TriangleNeighbours &n0 = imp_->triangle_neighbours[current_edge->triangle];
          TriangleNeighbours &n1 = imp_->triangle_neighbours[previous_edge->triangle];

          n0.neighbours[current_edge->triangle_edge_index] = previous_edge->triangle;
          n0.neighbour_edge_indices[current_edge->triangle_edge_index] = previous_edge->triangle_edge_index;

          n1.neighbours[previous_edge->triangle_edge_index] = current_edge->triangle;
          n1.neighbour_edge_indices[previous_edge->triangle_edge_index] = current_edge->triangle_edge_index;

          // Done. Make sure we move iteration on the pair.
          ++i;
        }
        // else We have an open edge and need do nothing.
        previous_edge = current_edge;
      }
    }
  }

  // Normalise data stored in vertex_normals to get the final normals.
  for (auto &vertex_normal : imp_->vertex_normals)
  {
    vertex_normal = glm::normalize(vertex_normal);
  }

  return true;
}


size_t HeightmapMesh::vertexCount() const
{
  return imp_->vertices.size();
}


size_t HeightmapMesh::triangleCount() const
{
  return imp_->tri_normals.size();
}


const glm::dvec3 *HeightmapMesh::vertices() const
{
  return imp_->vertices.data();
}


const glm::vec3 *HeightmapMesh::vertexNormals() const
{
  return imp_->vertex_normals.data();
}


const glm::vec3 *HeightmapMesh::triangleNormals() const
{
  return imp_->tri_normals.data();
}


const unsigned *HeightmapMesh::triangles() const
{
  return imp_->triangles.data();
}


const Aabb &HeightmapMesh::meshBoundingBox() const
{
  return imp_->mesh_extents;
}


const TriangleNeighbours *HeightmapMesh::triangleNeighbours() const
{
  return imp_->triangle_neighbours.data();
}


bool HeightmapMesh::extractPlyMesh(PlyMesh &mesh, bool offset_by_extents)
{
  if (triangleCount() == 0)
  {
    return false;
  }

  mesh.clear();

  const glm::dvec3 offset = (offset_by_extents) ? imp_->mesh_extents.minExtents() : glm::dvec3(0.0);

  for (const glm::dvec3 &vert : imp_->vertices)
  {
    // Offset by mesh min extents to support single precision convertion.
    mesh.addVertex(glm::vec3(vert - offset));
  }

  mesh.addTriangles(imp_->triangles.data(), unsigned(imp_->triangles.size() / 3));

  return true;
}
