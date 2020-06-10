// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapMesh.h"

// Must come first for various include issues.
#include "delaunator.hpp"

#include <ohm/Aabb.h>
#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/TriangleEdge.h>
#include <ohm/TriangleNeighbours.h>
#include <ohm/Voxel.h>

#include <ohmutil/PlyMesh.h>
#include <ohmutil/Profile.h>

#include <glm/ext.hpp>
#include <glm/glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include <vector>

using namespace ohm;

namespace ohm
{
  class HeightmapMeshDetail
  {
  public:
    std::vector<glm::dvec3> vertices;
    std::vector<glm::vec3> vertex_normals;
    std::vector<glm::vec3> tri_normals;
    std::vector<unsigned> triangles;
    std::vector<TriangleNeighbours> triangle_neighbours;
    std::vector<TriangleEdge> edges;
    std::vector<double> coords_2d;
    HeightmapMesh::NormalsMode normals_mode = HeightmapMesh::kNormalsAverage;
    /// Loose mesh extents enclosing the generating 2D voxels. The extents are tight along the height axis.
    Aabb loose_mesh_extents = Aabb(0.0);
    /// Tight mesh extents exactly enclosing the mesh vertices.
    Aabb tight_mesh_extents = Aabb(0.0);
    double resolution = 0.0;

    void clear()
    {
      vertices.clear();
      vertex_normals.clear();
      tri_normals.clear();
      triangles.clear();
      triangle_neighbours.clear();
      edges.clear();
      coords_2d.clear();
      loose_mesh_extents = tight_mesh_extents = Aabb(0.0);
      resolution = 0.0;
    }
  };
}  // namespace ohm


HeightmapMesh::HeightmapMesh(NormalsMode normals_mode)
  : imp_(new HeightmapMeshDetail)
{
  imp_->normals_mode = normals_mode;
}


HeightmapMesh::~HeightmapMesh() = default;


HeightmapMesh::NormalsMode HeightmapMesh::normalsMode()
{
  return imp_->normals_mode;
}


void HeightmapMesh::setNormalsMode(NormalsMode mode)
{
  imp_->normals_mode = mode;
}


bool HeightmapMesh::buildMesh(const Heightmap &heightmap, const MeshVoxelModifier &voxel_modifier)
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

  imp_->resolution = heightmap_occupancy.resolution();

  const glm::dvec3 up = heightmap.upAxisNormal();
  const glm::vec3 upf(up);
  const double heightmap_resolution = heightmap_occupancy.resolution();
  glm::dvec3 point;
  glm::dvec3 min_map_ext, max_map_ext;
  glm::dvec3 min_vert_ext, max_vert_ext;
  glm::dvec3 voxel_centre;

  min_map_ext = min_vert_ext = glm::dvec3(std::numeric_limits<double>::max());
  max_map_ext = max_vert_ext = glm::dvec3(-std::numeric_limits<double>::max());

  // Build vertices and map extents.
  for (auto voxel_iter = heightmap_occupancy.begin(); voxel_iter != heightmap_occupancy.end(); ++voxel_iter)
  {
    const VoxelConst &voxel = *voxel_iter;
    HeightmapVoxel voxel_info;
    auto voxel_type = heightmap.getHeightmapVoxelInfo(voxel, &point, &voxel_info);
    if (voxel_modifier)
    {
      voxel_type = voxel_modifier(voxel, voxel_type, &point, &voxel_info.clearance);
    }

    if (voxel_type != HeightmapVoxelType::kUnknown && voxel_type != HeightmapVoxelType::kVacant)
    {
      imp_->coords_2d.push_back(point.x);
      imp_->coords_2d.push_back(point.y);
      imp_->vertices.push_back(point);

      // Adjust tight extents
      min_vert_ext.x = std::min(point.x, min_vert_ext.x);
      min_vert_ext.y = std::min(point.y, min_vert_ext.y);
      min_vert_ext.z = std::min(point.z, min_vert_ext.z);

      max_vert_ext.x = std::max(point.x, max_vert_ext.x);
      max_vert_ext.y = std::max(point.y, max_vert_ext.y);
      max_vert_ext.z = std::max(point.z, max_vert_ext.z);

      // Adjust loose extents
      voxel_centre = voxel.centreGlobal();
      min_map_ext.x = std::min(voxel_centre.x - 0.5 * heightmap_resolution, min_map_ext.x);
      min_map_ext.y = std::min(voxel_centre.y - 0.5 * heightmap_resolution, min_map_ext.y);
      min_map_ext.z = std::min(voxel_centre.z - 0.5 * heightmap_resolution, min_map_ext.z);

      max_map_ext.x = std::max(voxel_centre.x + 0.5 * heightmap_resolution, max_map_ext.x);
      max_map_ext.y = std::max(voxel_centre.y + 0.5 * heightmap_resolution, max_map_ext.y);
      max_map_ext.z = std::max(voxel_centre.z + 0.5 * heightmap_resolution, max_map_ext.z);
    }
  }

  if (!imp_->vertices.empty())
  {
    // Fixup the loose extents along the height axis. This matches that of the tight extents.
    const int up_axis_index = heightmap.upAxisIndex();
    min_map_ext[up_axis_index] = min_vert_ext[up_axis_index];
    max_map_ext[up_axis_index] = max_vert_ext[up_axis_index];
  }
  else
  {
    min_map_ext = max_map_ext = min_vert_ext = max_vert_ext = glm::dvec3(0.0);
  }

  imp_->loose_mesh_extents = Aabb(min_map_ext, max_map_ext);
  imp_->tight_mesh_extents = Aabb(min_vert_ext, max_vert_ext);

  // Need at least 3 points to triangulate.
  if (imp_->coords_2d.size() < 3)
  {
    return false;
  }

  // Triangulate using Delaunay triangulation.
  try
  {
    delaunator::Delaunator delaunay(imp_->coords_2d);

    // Prime vertex normal calculations.
    imp_->vertex_normals.clear();
    imp_->vertex_normals.reserve(imp_->vertices.size());
    for (size_t i = 0; i < imp_->vertices.size(); ++i)
    {
      imp_->vertex_normals.emplace_back(glm::vec3(0.0f));
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
      TriangleNeighbours neighbour_info{};

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

        imp_->edges.emplace_back(TriangleEdge(indices[0], indices[1], tri_count, 0));
        imp_->edges.emplace_back(TriangleEdge(indices[1], indices[2], tri_count, 1));
        imp_->edges.emplace_back(TriangleEdge(indices[2], indices[0], tri_count, 2));

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
        const TriangleEdge *previous_edge = &imp_->edges[0];
        const TriangleEdge *current_edge;
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
  catch (const std::runtime_error &)
  {
    // Triangulation has failed. Can come about due to co-linear coordinates.
  }

  return false;
}


double HeightmapMesh::resolution() const
{
  return imp_->resolution;
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
  return imp_->loose_mesh_extents;
}


const Aabb &HeightmapMesh::tightMeshBoundingBox() const
{
  return imp_->tight_mesh_extents;
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

  const glm::dvec3 offset = (offset_by_extents) ? imp_->loose_mesh_extents.minExtents() : glm::dvec3(0.0);

  for (const glm::dvec3 &vert : imp_->vertices)
  {
    // Offset by mesh min extents to support single precision convertion.
    mesh.addVertex(glm::vec3(vert - offset));
  }

  mesh.addTriangles(imp_->triangles.data(), unsigned(imp_->triangles.size() / 3));

  return true;
}
