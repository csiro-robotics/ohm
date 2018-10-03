//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#include "PlyMesh.h"

#include "SafeIO.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>

#ifndef ZU
#if defined(_MSC_VER)
#define ZU "Iu"
#else
#define ZU "zu"
#endif
#endif // ZU

namespace
{
  bool isBigEndian()
  {
    union
    {
      uint32_t i;
      char c[4];
    } bint = {0x01020304};

    return bint.c[0] == 1;
  }
}

// File wrapper specialisations.
template <>
class PlyMesh::FileWrapper<FILE>
{
public:
  inline FileWrapper(FILE *file, bool close_file) : file_(file), close_file_(close_file) {}
  inline ~FileWrapper() { if (file_ && close_file_) { fclose(file_); } }

  inline bool isOpen() const { return file_ != nullptr; }

  void printf(const char *format, ...)
  {
    va_list args;
    va_start(args, format);
    vfprintf(file_, format, args);
    va_end(args);
  }


  void write(const void *ptr, size_t element_size, size_t element_count = 1u)
  {
    fwrite(ptr, element_size, element_count, file_);
  }

private:
  FILE *file_;
  bool close_file_;
};


template <>
class PlyMesh::FileWrapper<std::ostream>
{
public:
  inline FileWrapper(std::ostream &stream)
    : stream_(&stream)
  {
    buffer_ = new char[kBufferSize];
  }

  inline ~FileWrapper()
  {
    delete [] buffer_;
  }

  inline bool isOpen() const { return stream_ != nullptr; }

  void printf(const char *format, ...)
  {
    int written = 0;
    va_list args;
    va_start(args, format);
    written = vsnprintf(buffer_, kBufferSize, format, args);
    va_end(args);
    buffer_[std::min<unsigned>(written, kBufferSize)] = '\0';
    *stream_ << buffer_;
  }


  void write(const void *ptr, size_t element_size, size_t element_count = 1u)
  {
    stream_->write(static_cast<const std::ostream::char_type *>(ptr), element_size * element_count);
  }

private:
  static const unsigned kBufferSize = 1024u;  // Far to big for usage here.
  std::ostream *stream_;
  char *buffer_;
};


PlyMesh::PlyMesh()
  : index_mapper_(nullptr)
  , vertex_colours_(false)
  , edge_colours_(false)
  , face_colours_(false)
{

}


PlyMesh::~PlyMesh()
{
  delete index_mapper_;
  index_mapper_ = nullptr;
}


void PlyMesh::clear()
{
  delete index_mapper_;
  index_mapper_ = nullptr;
  vertices_.clear();
  edges_.clear();
  triangles_.clear();
  vertex_colours_ = edge_colours_ = face_colours_ = false;
}


void PlyMesh::addEdges(const unsigned *edge_indices, unsigned edge_count, const Colour *colours)
{
  for (unsigned i = 0; i < edge_count; ++i)
  {
    Edge e;
    e.v[0] = edge_indices[i * 2 + 0];
    e.v[1] = edge_indices[i * 2 + 1];
    if (index_mapper_)
    {
      e.v[0] = (*index_mapper_)[e.v[0]];
      e.v[1] = (*index_mapper_)[e.v[1]];
    }
    e.colour = Colour(255, 255, 255);
    if (colours)
    {
      edge_colours_ = true;
      e.colour = colours[i];
    }
    edges_.push_back(e);
  }
}


void PlyMesh::addEdge(const glm::vec3 &v0, const glm::vec3 &v1, const Colour &colour)
{
  unsigned i0, i1;
  i0 = unsigned(vertices_.size());
  addVertices(&v0, 1);
  i1 = unsigned(vertices_.size());
  addVertices(&v1, 1);
  addEdge(i0, i1, colour);
}


void PlyMesh::addTriangles(const unsigned *triangle_indices, unsigned triangle_count, const Colour *colours)
{
  for (unsigned i = 0; i < triangle_count; ++i)
  {
    Tri t;
    t.v[0] = triangle_indices[i * 3 + 0];
    t.v[1] = triangle_indices[i * 3 + 1];
    t.v[2] = triangle_indices[i * 3 + 2];
    if (index_mapper_)
    {
      t.v[0] = (*index_mapper_)[t.v[0]];
      t.v[1] = (*index_mapper_)[t.v[1]];
      t.v[2] = (*index_mapper_)[t.v[2]];
    }
    t.colour = Colour(255, 255, 255);
    if (colours)
    {
      face_colours_ = true;
      t.colour = colours[i];
    }
    triangles_.push_back(t);
  }
}


void PlyMesh::addTriangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const Colour &colour)
{
  unsigned i0, i1, i2;
  i0 = unsigned(vertices_.size());
  addVertices(&v0, 1);
  i1 = unsigned(vertices_.size());
  addVertices(&v1, 1);
  i2 = unsigned(vertices_.size());
  addVertices(&v2, 1);
  addTriangle(i0, i1, i2, colour);
}


void PlyMesh::addPolygon(const unsigned *indices, unsigned order, const Colour &colour)
{
  if (order == 0)
  {
    return;
  }

  Poly poly;
  poly.indices_start = polygon_indices_.size();
  poly.order = order;
  poly.colour = colour;

  // Use push_back() for indices, not resize() to allow vector
  // to increase in size in a more expansive way (reduce reallocation).
  for (unsigned i = 0; i < order; ++i)
  {
    polygon_indices_.push_back(indices[i]);
  }

  face_colours_ = face_colours_ || colour != Colour::kColours[Colour::kWhite];

  polygons_.push_back(poly);
}


void PlyMesh::addComment(const char *comment)
{
  comments_.push_back(comment);
}


const char *PlyMesh::getComment(unsigned index) const
{
  if (index >= comments_.size())
  {
    return nullptr;
  }

  return comments_[index].c_str();
}


unsigned PlyMesh::commentCount() const
{
  return unsigned(comments_.size());
}


void PlyMesh::clearComments()
{
  comments_.clear();
}


bool PlyMesh::save(const char *out_path, bool binary) const
{
  FILE *cfile = nullptr;
  if (fopen_s(&cfile, out_path, "wb"))
  {
    return false;
  }
  FileWrapper<FILE> out(cfile, true);
  return save(out, binary);
}



bool PlyMesh::save(FILE *file, bool binary) const
{
  FileWrapper<FILE> out(file, false);
  return save(out, binary);
}



bool PlyMesh::save(std::ostream &stream, bool binary) const
{
  FileWrapper<std::ostream> out(stream);
  return save(out, binary);
}



unsigned PlyMesh::addVertices(const glm::vec3 *verts, unsigned count, const Colour *colours)
{
  unsigned index = unsigned(vertices_.size());
  for (unsigned i = 0; i < count; ++i)
  {
    Vertex v;
    v.point = VertexType(verts[i]);
    v.colour = Colour(255, 255, 255);
    if (colours)
    {
      vertex_colours_ = true;
      v.colour = colours[i];
    }
    vertices_.push_back(v);
  }
  return index;
}


void PlyMesh::setNormal(unsigned vertex_index, const glm::vec3 &normal)
{
  if (normals_.size() <= vertex_index)
  {
    normals_.reserve(std::max<size_t>(vertex_index + 1, vertices_.size()));
    while (normals_.size() <= vertex_index)
    {
      normals_.push_back(VertexType(0, 0, 0));
    }
  }

  normals_[vertex_index] = normal;
}


void PlyMesh::addPolygon(const glm::vec3 *verts, unsigned order, const Colour &colour)
{
  if (!order)
  {
    return;
  }

  Poly poly;
  poly.indices_start = polygon_indices_.size();
  poly.order = order;
  poly.colour = colour;

  // Use push_back() for indices, not resize() to allow vector
  // to increase in size in a more expansive way (reduce reallocation).
  const unsigned index_offset = unsigned(vertices_.size());
  addVertices(verts, order);
  for (unsigned i = 0; i < order; ++i)
  {
    polygon_indices_.push_back(index_offset + i);
  }

  face_colours_ = face_colours_ || colour != Colour::kColours[Colour::kWhite];

  polygons_.push_back(poly);
}


void PlyMesh::addMappedTriangle(const glm::vec3 *verts, const unsigned *vert_ids, const Colour *colour)
{
  if (!index_mapper_)
  {
    index_mapper_ = new std::unordered_map < unsigned, unsigned > ;
  }

  Tri tri;
  tri.colour = (colour) ? *colour : Colour(kDefaultColour);
  face_colours_ = face_colours_ || colour != nullptr;
  for (unsigned i = 0; i < 3; ++i)
  {
    auto search = index_mapper_->find(vert_ids[i]);
    if (search == index_mapper_->end())
    {
      tri.v[i] = uint32_t(vertices_.size());
      addVertex(verts[i]);
    }
    else
    {
      tri.v[i] = search->second;
    }
  }
  triangles_.push_back(tri);
}


void PlyMesh::addMappedPolygon(const glm::vec3 *verts, const unsigned *vert_ids, unsigned order, const Colour *colour)
{
  if (!index_mapper_)
  {
    index_mapper_ = new std::unordered_map < unsigned, unsigned > ;
  }

  Poly poly;
  poly.indices_start = polygon_indices_.size();
  poly.order = 0;
  poly.colour = (colour) ? *colour : Colour(kDefaultColour);
  face_colours_ = face_colours_ || colour != nullptr;
  for (unsigned i = 0; i < order; ++i)
  {
    auto search = index_mapper_->find(vert_ids[i]);
    if (search == index_mapper_->end())
    {
      polygon_indices_.push_back(uint32_t(vertices_.size()));
      addVertex(verts[i]);
    }
    else
    {
      polygon_indices_.push_back(search->second);
    }
  }
  polygons_.push_back(poly);
}


void PlyMesh::addMappedEdge(const glm::vec3 *verts, const unsigned *vert_ids, const Colour *colour)
{
  if (!index_mapper_)
  {
    index_mapper_ = new std::unordered_map < unsigned, unsigned > ;
  }

  Edge edge;
  edge.colour = (colour) ? *colour : Colour(kDefaultColour);
  edge_colours_ = edge_colours_ || colour != nullptr;
  for (unsigned i = 0; i < 2; ++i)
  {
    auto search = index_mapper_->find(vert_ids[i]);
    if (search == index_mapper_->end())
    {
      edge.v[i] = uint32_t(vertices_.size());
      addVertex(verts[i]);
    }
    else
    {
      edge.v[i] = search->second;
    }
  }
  edges_.push_back(edge);
}


template <typename T>
bool PlyMesh::save(FileWrapper<T> &out, bool binary) const
{
  if (!out.isOpen())
  {
    return false;
  }
  bool with_normals = false;

  out.printf("ply\n");
  if (binary)
  {
    if (isBigEndian())
    {
      out.printf("format binary_big_endian 1.0\n");
    }
    else
    {
      out.printf("format binary_little_endian 1.0\n");
    }
  }
  else
  {
    out.printf("format ascii 1.0\n");
  }
  out.printf("comment Exported by Voxmesh PlyMesh\n");

  for (const std::string &comment : comments_)
  {
    out.printf("comment %s\n", comment.c_str());
  }

  // Write vertex info.
  out.printf("element vertex %" ZU "\n", vertices_.size());
  out.printf("property float x\n");
  out.printf("property float y\n");
  out.printf("property float z\n");
  if (!normals_.empty())
  {
    with_normals = true;
    out.printf("property float nx\n");
    out.printf("property float ny\n");
    out.printf("property float nz\n");
  }
  if (vertex_colours_)
  {
    out.printf("property uchar red\n");
    out.printf("property uchar green\n");
    out.printf("property uchar blue\n");
  }

  if (!triangles_.empty() || !polygons_.empty())
  {
    out.printf("element face %" ZU "\n", triangles_.size() + polygons_.size());
    out.printf("property list uchar int vertex_indices\n");
    if (face_colours_)
    {
      out.printf("property uchar red\n");
      out.printf("property uchar green\n");
      out.printf("property uchar blue\n");
    }
  }

  if (!edges_.empty())
  {
    out.printf("element edge %" ZU "\n", edges_.size());
    out.printf("property int vertex1\n");
    out.printf("property int vertex2\n");
    if (edge_colours_)
    {
      out.printf("property uchar red\n");
      out.printf("property uchar green\n");
      out.printf("property uchar blue\n");
    }
  }

  out.printf("end_header\n");

  // Write vertices.
  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    const Vertex &v = vertices_[i];
    VertexType n = (with_normals && i < normals_.size()) ? normals_[i] : VertexType(0, 0, 0);
    if (binary)
    {
      out.write(&v.point.x, sizeof(float), 1u);
      out.write(&v.point.y, sizeof(float), 1u);
      out.write(&v.point.z, sizeof(float), 1u);

      if (with_normals)
      {
        out.write(&n.x, sizeof(float), 1u);
        out.write(&n.y, sizeof(float), 1u);
        out.write(&n.z, sizeof(float), 1u);
      }

      if (vertex_colours_)
      {
        out.write(&v.colour.r(), 1u, 1u);
        out.write(&v.colour.g(), 1u, 1u);
        out.write(&v.colour.b(), 1u, 1u);
      }
    }
    else
    {
      out.printf("%f %f %f", v.point.x, v.point.y, v.point.z);
      if (with_normals)
      {
        out.printf(" %f %f %f", n.x, n.y, n.z);
      }
      if (vertex_colours_)
      {
        out.printf(" %d %d %d", unsigned(v.colour.r()), unsigned(v.colour.g()), unsigned(v.colour.b()));
      }
      out.printf("\n");
    }
  }

  // Write triangle faces.
  for (size_t i = 0; i < triangles_.size(); ++i)
  {
    const Tri &t = triangles_[i];
    if (binary)
    {
      unsigned char vc = 3;
      out.write(&vc, sizeof(vc), 1u);
      out.write(t.v, sizeof(t.v[0]), 3u);
      if (face_colours_)
      {
        out.write(&t.colour.r(), 1u, 1u);
        out.write(&t.colour.g(), 1u, 1u);
        out.write(&t.colour.b(), 1u, 1u);
      }
    }
    else
    {
      const unsigned vc = 3;
      out.printf("%u %u %u %u", vc, t.v[0], t.v[1], t.v[2]);
      if (face_colours_)
      {
        out.printf(" %d %d %d", unsigned(t.colour.r()), unsigned(t.colour.g()), unsigned(t.colour.b()));
      }
      out.printf("\n");
    }
  }

  // Write non triangle faces.
  for (size_t i = 0; i < polygons_.size(); ++i)
  {
    const Poly &poly = polygons_[i];
    if (binary)
    {
      unsigned char vc = poly.order;
      out.write(&vc, sizeof(vc), 1u);
      out.write(&polygon_indices_[poly.indices_start], sizeof(polygon_indices_.front()), poly.order);
      if (face_colours_)
      {
        out.write(&poly.colour.r(), 1u, 1u);
        out.write(&poly.colour.g(), 1u, 1u);
        out.write(&poly.colour.b(), 1u, 1u);
      }
    }
    else
    {
      out.printf("%u", poly.order);
      for (unsigned j = 0; j < poly.order; ++j)
      {
        out.printf(" %u", polygon_indices_[poly.indices_start + j]);
      }
      if (face_colours_)
      {
        out.printf(" %d %d %d", unsigned(poly.colour.r()), unsigned(poly.colour.g()), unsigned(poly.colour.b()));
      }
      out.printf("\n");
    }
  }

  // Write edges/lines.
  for (size_t i = 0; i < edges_.size(); ++i)
  {
    const Edge &e = edges_[i];
    if (binary)
    {
      out.write(e.v, sizeof(e.v[0]), 2u);
      if (edge_colours_)
      {
        out.write(&e.colour.r(), 1u, 1u);
        out.write(&e.colour.g(), 1u, 1u);
        out.write(&e.colour.b(), 1u, 1u);
      }
    }
    else
    {
      out.printf("%u %u", e.v[0], e.v[1]);
      if (edge_colours_)
      {
        out.printf(" %d %d %d", unsigned(e.colour.r()), unsigned(e.colour.g()), unsigned(e.colour.b()));
      }
      out.printf("\n");
    }
  }

  return true;
}


template bool PlyMesh::save<FILE>(FileWrapper<FILE> &out, bool binary) const;
template bool PlyMesh::save<std::ostream>(FileWrapper<std::ostream> &out, bool binary) const;
