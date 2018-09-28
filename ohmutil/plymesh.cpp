//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#include "plymesh.h"

#include "safeio.h"

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
  inline FileWrapper(FILE *file, bool closeFile) : _file(file), _closeFile(closeFile) {}
  inline ~FileWrapper() { if (_file && _closeFile) { fclose(_file); } }

  inline bool isOpen() const { return _file != nullptr; }

  void printf(const char *format, ...)
  {
    va_list args;
    va_start(args, format);
    vfprintf(_file, format, args);
    va_end(args);
  }


  void write(const void *ptr, size_t elementSize, size_t elementCount = 1u)
  {
    fwrite(ptr, elementSize, elementCount, _file);
  }

private:
  FILE *_file;
  bool _closeFile;
};


template <>
class PlyMesh::FileWrapper<std::ostream>
{
public:
  inline FileWrapper(std::ostream &stream)
    : _stream(&stream)
  {
    _buffer = new char[BufferSize];
  }

  inline ~FileWrapper()
  {
    delete [] _buffer;
  }

  inline bool isOpen() const { return _stream != nullptr; }

  void printf(const char *format, ...)
  {
    int written = 0;
    va_list args;
    va_start(args, format);
    written = vsnprintf(_buffer, BufferSize, format, args);
    va_end(args);
    _buffer[std::min<unsigned>(written, BufferSize)] = '\0';
    *_stream << _buffer;
  }


  void write(const void *ptr, size_t elementSize, size_t elementCount = 1u)
  {
    _stream->write(static_cast<const std::ostream::char_type *>(ptr), elementSize * elementCount);
  }

private:
  static const unsigned BufferSize;
  std::ostream *_stream;
  char *_buffer;
};

const unsigned PlyMesh::FileWrapper<std::ostream>::BufferSize = 1024u;  // Far to big for usage here.


PlyMesh::PlyMesh()
  : _indexMapper(nullptr)
  , _vertexColours(false)
  , _edgeColours(false)
  , _faceColours(false)
{

}


PlyMesh::~PlyMesh()
{
  delete _indexMapper;
  _indexMapper = nullptr;
}


void PlyMesh::clear()
{
  delete _indexMapper;
  _indexMapper = nullptr;
  _vertices.clear();
  _edges.clear();
  _triangles.clear();
  _vertexColours = _edgeColours = _faceColours = false;
}


void PlyMesh::addEdges(const unsigned *edgeIndices, unsigned edgeCount, const Colour *colours)
{
  for (unsigned i = 0; i < edgeCount; ++i)
  {
    Edge e;
    e.v[0] = edgeIndices[i * 2 + 0];
    e.v[1] = edgeIndices[i * 2 + 1];
    if (_indexMapper)
    {
      e.v[0] = (*_indexMapper)[e.v[0]];
      e.v[1] = (*_indexMapper)[e.v[1]];
    }
    e.colour = Colour(255, 255, 255);
    if (colours)
    {
      _edgeColours = true;
      e.colour = colours[i];
    }
    _edges.push_back(e);
  }
}


void PlyMesh::addEdge(const glm::vec3 &v0, const glm::vec3 &v1, const Colour &colour)
{
  unsigned i0, i1;
  i0 = unsigned(_vertices.size());
  addVertices(&v0, 1);
  i1 = unsigned(_vertices.size());
  addVertices(&v1, 1);
  addEdge(i0, i1, colour);
}


void PlyMesh::addTriangles(const unsigned *triangleIndices, unsigned triangleCount, const Colour *colours)
{
  for (unsigned i = 0; i < triangleCount; ++i)
  {
    Tri t;
    t.v[0] = triangleIndices[i * 3 + 0];
    t.v[1] = triangleIndices[i * 3 + 1];
    t.v[2] = triangleIndices[i * 3 + 2];
    if (_indexMapper)
    {
      t.v[0] = (*_indexMapper)[t.v[0]];
      t.v[1] = (*_indexMapper)[t.v[1]];
      t.v[2] = (*_indexMapper)[t.v[2]];
    }
    t.colour = Colour(255, 255, 255);
    if (colours)
    {
      _faceColours = true;
      t.colour = colours[i];
    }
    _triangles.push_back(t);
  }
}


void PlyMesh::addTriangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const Colour &colour)
{
  unsigned i0, i1, i2;
  i0 = unsigned(_vertices.size());
  addVertices(&v0, 1);
  i1 = unsigned(_vertices.size());
  addVertices(&v1, 1);
  i2 = unsigned(_vertices.size());
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
  poly.indicesStart = _polygonIndices.size();
  poly.order = order;
  poly.colour = colour;

  // Use push_back() for indices, not resize() to allow vector
  // to increase in size in a more expansive way (reduce reallocation).
  for (unsigned i = 0; i < order; ++i)
  {
    _polygonIndices.push_back(indices[i]);
  }

  _faceColours = _faceColours || colour != Colour::Colours[Colour::White];

  _polygons.push_back(poly);
}


void PlyMesh::addComment(const char *comment)
{
  _comments.push_back(comment);
}


const char *PlyMesh::getComment(unsigned index) const
{
  if (index >= _comments.size())
  {
    return nullptr;
  }

  return _comments[index].c_str();
}


unsigned PlyMesh::commentCount() const
{
  return unsigned(_comments.size());
}


void PlyMesh::clearComments()
{
  _comments.clear();
}


bool PlyMesh::save(const char *outPath, bool binary) const
{
  FILE *cfile = nullptr;
  if (fopen_s(&cfile, outPath, "wb"))
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
  unsigned index = unsigned(_vertices.size());
  for (unsigned i = 0; i < count; ++i)
  {
    Vertex v;
    v.point = VertexType(verts[i]);
    v.colour = Colour(255, 255, 255);
    if (colours)
    {
      _vertexColours = true;
      v.colour = colours[i];
    }
    _vertices.push_back(v);
  }
  return index;
}


void PlyMesh::setNormal(unsigned vertexIndex, const glm::vec3 &normal)
{
  if (_normals.size() <= vertexIndex)
  {
    _normals.reserve(std::max<size_t>(vertexIndex + 1, _vertices.size()));
    while (_normals.size() <= vertexIndex)
    {
      _normals.push_back(VertexType(0, 0, 0));
    }
  }

  _normals[vertexIndex] = normal;
}


void PlyMesh::addPolygon(const glm::vec3 *verts, unsigned order, const Colour &colour)
{
  if (!order)
  {
    return;
  }

  Poly poly;
  poly.indicesStart = _polygonIndices.size();
  poly.order = order;
  poly.colour = colour;

  // Use push_back() for indices, not resize() to allow vector
  // to increase in size in a more expansive way (reduce reallocation).
  unsigned indexOffset = unsigned(_vertices.size());
  addVertices(verts, order);
  for (unsigned i = 0; i < order; ++i)
  {
    _polygonIndices.push_back(indexOffset + i);
  }

  _faceColours = _faceColours || colour != Colour::Colours[Colour::White];

  _polygons.push_back(poly);
}


void PlyMesh::addMappedTriangle(const glm::vec3 *verts, const unsigned *vertIds, const Colour *colour)
{
  if (!_indexMapper)
  {
    _indexMapper = new std::unordered_map < unsigned, unsigned > ;
  }

  Tri tri;
  tri.colour = (colour) ? *colour : Colour(DefaultColour);
  _faceColours = _faceColours || colour != nullptr;
  for (unsigned i = 0; i < 3; ++i)
  {
    auto search = _indexMapper->find(vertIds[i]);
    if (search == _indexMapper->end())
    {
      tri.v[i] = uint32_t(_vertices.size());
      addVertex(verts[i]);
    }
    else
    {
      tri.v[i] = search->second;
    }
  }
  _triangles.push_back(tri);
}


void PlyMesh::addMappedPolygon(const glm::vec3 *verts, const unsigned *vertIds, unsigned order, const Colour *colour)
{
  if (!_indexMapper)
  {
    _indexMapper = new std::unordered_map < unsigned, unsigned > ;
  }

  Poly poly;
  poly.indicesStart = _polygonIndices.size();
  poly.order = 0;
  poly.colour = (colour) ? *colour : Colour(DefaultColour);
  _faceColours = _faceColours || colour != nullptr;
  for (unsigned i = 0; i < order; ++i)
  {
    auto search = _indexMapper->find(vertIds[i]);
    if (search == _indexMapper->end())
    {
      _polygonIndices.push_back(uint32_t(_vertices.size()));
      addVertex(verts[i]);
    }
    else
    {
      _polygonIndices.push_back(search->second);
    }
  }
  _polygons.push_back(poly);
}


void PlyMesh::addMappedEdge(const glm::vec3 *verts, const unsigned *vertIds, const Colour *colour)
{
  if (!_indexMapper)
  {
    _indexMapper = new std::unordered_map < unsigned, unsigned > ;
  }

  Edge edge;
  edge.colour = (colour) ? *colour : Colour(DefaultColour);
  _edgeColours = _edgeColours || colour != nullptr;
  for (unsigned i = 0; i < 2; ++i)
  {
    auto search = _indexMapper->find(vertIds[i]);
    if (search == _indexMapper->end())
    {
      edge.v[i] = uint32_t(_vertices.size());
      addVertex(verts[i]);
    }
    else
    {
      edge.v[i] = search->second;
    }
  }
  _edges.push_back(edge);
}


template <typename T>
bool PlyMesh::save(FileWrapper<T> &out, bool binary) const
{
  if (!out.isOpen())
  {
    return false;
  }
  bool withNormals = false;

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

  for (const std::string &comment : _comments)
  {
    out.printf("comment %s\n", comment.c_str());
  }

  // Write vertex info.
  out.printf("element vertex %" ZU "\n", _vertices.size());
  out.printf("property float x\n");
  out.printf("property float y\n");
  out.printf("property float z\n");
  if (!_normals.empty())
  {
    withNormals = true;
    out.printf("property float nx\n");
    out.printf("property float ny\n");
    out.printf("property float nz\n");
  }
  if (_vertexColours)
  {
    out.printf("property uchar red\n");
    out.printf("property uchar green\n");
    out.printf("property uchar blue\n");
  }

  if (!_triangles.empty() || !_polygons.empty())
  {
    out.printf("element face %" ZU "\n", _triangles.size() + _polygons.size());
    out.printf("property list uchar int vertex_indices\n");
    if (_faceColours)
    {
      out.printf("property uchar red\n");
      out.printf("property uchar green\n");
      out.printf("property uchar blue\n");
    }
  }

  if (!_edges.empty())
  {
    out.printf("element edge %" ZU "\n", _edges.size());
    out.printf("property int vertex1\n");
    out.printf("property int vertex2\n");
    if (_edgeColours)
    {
      out.printf("property uchar red\n");
      out.printf("property uchar green\n");
      out.printf("property uchar blue\n");
    }
  }

  out.printf("end_header\n");

  // Write vertices.
  for (size_t i = 0; i < _vertices.size(); ++i)
  {
    const Vertex &v = _vertices[i];
    VertexType n = (withNormals && i < _normals.size()) ? _normals[i] : VertexType(0, 0, 0);
    if (binary)
    {
      out.write(&v.point.x, sizeof(float), 1u);
      out.write(&v.point.y, sizeof(float), 1u);
      out.write(&v.point.z, sizeof(float), 1u);

      if (withNormals)
      {
        out.write(&n.x, sizeof(float), 1u);
        out.write(&n.y, sizeof(float), 1u);
        out.write(&n.z, sizeof(float), 1u);
      }

      if (_vertexColours)
      {
        out.write(&v.colour.r(), 1u, 1u);
        out.write(&v.colour.g(), 1u, 1u);
        out.write(&v.colour.b(), 1u, 1u);
      }
    }
    else
    {
      out.printf("%f %f %f", v.point.x, v.point.y, v.point.z);
      if (withNormals)
      {
        out.printf(" %f %f %f", n.x, n.y, n.z);
      }
      if (_vertexColours)
      {
        out.printf(" %d %d %d", unsigned(v.colour.r()), unsigned(v.colour.g()), unsigned(v.colour.b()));
      }
      out.printf("\n");
    }
  }

  // Write triangle faces.
  for (size_t i = 0; i < _triangles.size(); ++i)
  {
    const Tri &t = _triangles[i];
    if (binary)
    {
      unsigned char vc = 3;
      out.write(&vc, sizeof(vc), 1u);
      out.write(t.v, sizeof(t.v[0]), 3u);
      if (_faceColours)
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
      if (_faceColours)
      {
        out.printf(" %d %d %d", unsigned(t.colour.r()), unsigned(t.colour.g()), unsigned(t.colour.b()));
      }
      out.printf("\n");
    }
  }

  // Write non triangle faces.
  for (size_t i = 0; i < _polygons.size(); ++i)
  {
    const Poly &poly = _polygons[i];
    if (binary)
    {
      unsigned char vc = poly.order;
      out.write(&vc, sizeof(vc), 1u);
      out.write(&_polygonIndices[poly.indicesStart], sizeof(_polygonIndices.front()), poly.order);
      if (_faceColours)
      {
        out.write(&poly.colour.r(), 1u, 1u);
        out.write(&poly.colour.g(), 1u, 1u);
        out.write(&poly.colour.b(), 1u, 1u);
      }
    }
    else
    {
      out.printf("%u", poly.order);
      for (unsigned i = 0; i < poly.order; ++i)
      {
        out.printf(" %u", _polygonIndices[poly.indicesStart + i]);
      }
      if (_faceColours)
      {
        out.printf(" %d %d %d", unsigned(poly.colour.r()), unsigned(poly.colour.g()), unsigned(poly.colour.b()));
      }
      out.printf("\n");
    }
  }

  // Write edges/lines.
  for (size_t i = 0; i < _edges.size(); ++i)
  {
    const Edge &e = _edges[i];
    if (binary)
    {
      out.write(e.v, sizeof(e.v[0]), 2u);
      if (_edgeColours)
      {
        out.write(&e.colour.r(), 1u, 1u);
        out.write(&e.colour.g(), 1u, 1u);
        out.write(&e.colour.b(), 1u, 1u);
      }
    }
    else
    {
      out.printf("%u %u", e.v[0], e.v[1]);
      if (_edgeColours)
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
