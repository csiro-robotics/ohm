// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapImage.h"

#include "3rdparty/delaunator.hpp"

#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Voxel.h>

#include <ohmutil/PlyMesh.h>
#include <ohmutil/Profile.h>

#include <3esservermacros.h>

#include <mutex>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>

#include <glm/ext.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

using namespace ohm;

namespace
{
  // Fragment shader colour by input.
  const char *normals_fragment_shader = "#version 330 core\n"
                                        "in vec3 v_normal;\n"
                                        "in float v_depth;\n"
                                        "// Ouput data\n"
                                        "out vec3 colour;\n"
                                        "void main()\n"
                                        "{\n"
                                        // "  colour = vec3(v_depth, v_depth, v_depth);\n"
                                        "  colour = 0.5 * (v_normal + vec3(1, 1, 1));\n"
                                        "}";
  // const char *depth_fragment_shader = "#version 330 core\n"
  //                                     "in vec3 v_colour;\n"
  //                                     "in float v_depth;\n"
  //                                     "// Ouput data\n"
  //                                     "out vec3 color;\n"
  //                                     "void main()\n"
  //                                     "{\n"
  //                                     "  color = vec3(v_depth, v_depth, v_depth);\n"
  //                                     "}";

  // Colour by depth.
  // const char *fragment_shader = "// frag\n"
  //                               "in float v_depth;\n"
  //                               "void main()\n"
  //                               "{\n"
  //                               "  gl_FragColor = vec4( vec3(v_depth), 1.0 );\n"
  //                               "}\n";

  // Vertex shader.
  const char *vertex_shader = "#version 330 core\n"
                              "// Input vertex data, different for all executions of this shader.\n"
                              "layout(location = 0) in vec3 vertexPosition_modelspace;\n"
                              "layout(location = 1) in vec3 vertexNormal_modelspace;\n"
                              "uniform mat4 MVP;\n"
                              "uniform mat4 V;\n"
                              "uniform mat4 M;\n"
                              "out vec3 v_normal;\n"
                              "out float v_depth;\n"
                              "void main()\n"
                              "{\n"
                              "  gl_Position = gl_Position =  MVP * vec4(vertexPosition_modelspace,1);\n"
                              "  v_normal =  (V * M * vec4(vertexNormal_modelspace,0)).xyz;\n"
                              // "  v_normal =  vertexNormal_modelspace;\n"
                              "  v_depth = 1.0 - (1.0 + gl_Position.z/gl_Position.w) / 2.0;\n"
                              "}\n";

  const char *quad_fragment_shader = "#version 330 core\n"
                                     "// Ouput data\n"
                                     "layout(location = 0) out vec4 color;\n"
                                     "uniform sampler2D render_texture;\n"
                                     "in vec2 UV;\n"
                                     "void main()\n"
                                     "{\n"
                                     "  color = texture(render_texture, UV);\n"
                                     "}";

  // Full screen quad vertex shader.
  const char *quad_vertex_shader = "#version 330 core\n"
                                   "// Input vertex data, different for all executions of this shader.\n"
                                   "layout(location = 0) in vec3 vertexPosition_modelspace;\n"
                                   "// Output data ; will be interpolated for each fragment.\n"
                                   "out vec2 UV;\n"
                                   "void main()\n"
                                   "{\n"
                                   "    gl_Position = vec4(vertexPosition_modelspace, 1);\n"
                                   "    UV = (vertexPosition_modelspace.xy + vec2(1, 1)) / 2.0;\n"
                                   "}\n";

  // The fullscreen quad's FBO
  static const GLfloat g_quad_vertex_buffer_data[] =  //
    {
      -1.0f, -1.0f, 0.0f,  //
      1.0f,  -1.0f, 0.0f,  //
      -1.0f, 1.0f,  0.0f,  //
      -1.0f, 1.0f,  0.0f,  //
      1.0f,  -1.0f, 0.0f,  //
      1.0f,  1.0f,  0.0f,  //
    };

  // Load vertex and fragment shader strings into a program.
  GLuint loadShaders(const char * /*name*/, const char *vertex, const char *fragment)
  {
    // Create the shaders
    GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

    // Read the Vertex Shader code from the file
    std::string VertexShaderCode(vertex);

    // Read the Fragment Shader code from the file
    std::string FragmentShaderCode(fragment);

    GLint Result = GL_FALSE;
    int InfoLogLength;

    // Compile Vertex Shader
    // printf("Compiling shader vertex : %s\n", name);
    char const *VertexSourcePointer = VertexShaderCode.c_str();
    glShaderSource(VertexShaderID, 1, &VertexSourcePointer, NULL);
    glCompileShader(VertexShaderID);

    // Check Vertex Shader
    glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0)
    {
      std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
      glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
      // printf("%s\n", &VertexShaderErrorMessage[0]);
    }

    // Compile Fragment Shader
    // printf("Compiling shader fragment: %s\n", name);
    char const *FragmentSourcePointer = FragmentShaderCode.c_str();
    glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer, NULL);
    glCompileShader(FragmentShaderID);

    // Check Fragment Shader
    glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0)
    {
      std::vector<char> FragmentShaderErrorMessage(InfoLogLength + 1);
      glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
      // printf("%s\n", &FragmentShaderErrorMessage[0]);
    }

    // Link the program
    // printf("Linking program\n");
    GLuint ProgramID = glCreateProgram();
    glAttachShader(ProgramID, VertexShaderID);
    glAttachShader(ProgramID, FragmentShaderID);
    glLinkProgram(ProgramID);

    // Check the program
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
    glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0)
    {
      std::vector<char> ProgramErrorMessage(InfoLogLength + 1);
      glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
      // printf("%s\n", &ProgramErrorMessage[0]);
    }

    glDetachShader(ProgramID, VertexShaderID);
    glDetachShader(ProgramID, FragmentShaderID);

    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    return ProgramID;
  }
}  // namespace


namespace
{
  std::mutex shared_init_guard;
  volatile unsigned shared_ref_count = 0;

  void sharedInit()
  {
    std::unique_lock<std::mutex> guard(shared_init_guard);

    if (shared_ref_count == 0)
    {
      glfwInit();
    }

    ++shared_ref_count;
  }

  void sharedRelease()
  {
    std::unique_lock<std::mutex> guard(shared_init_guard);

    if (shared_ref_count)
    {
      --shared_ref_count;
      if (shared_ref_count == 0)
      {
        glfwTerminate();
      }
    }
  }
}  // namespace

namespace ohm
{
  struct HeightmapImageDetail
  {
    const Heightmap *heightmap;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> vertex_normals;
    std::vector<unsigned> indices;
    std::vector<glm::vec3> tri_normals;
    std::vector<double> coords_2d;

    HeightmapImage::BitmapInfo image_info;
    std::vector<uint8_t> image;

    HeightmapImage::ImageType type = HeightmapImage::kImageNormals;
    unsigned pixels_per_voxel = 1;
    HeightmapImage::NormalsMode normals_mode = HeightmapImage::kNormalsWorst;

    struct RenderData
    {
      GLFWwindow *window = nullptr;
      GLuint map_program_id = 0;
      GLuint quad_program_id = 0;
      GLuint quad_vertex_buffer = 0;
      GLuint fbo_tex_id = 0xffffffffu;
      // Debug visualisation flag.
      bool show_window = false;

      bool init();

      ~RenderData() { clear(); }

      void clear();
    } render_data;
  };


  bool HeightmapImageDetail::RenderData::init()
  {
    PROFILE(HeightmapImageDetail_init);
    if (window)
    {
      return true;
    }

    ::sharedInit();

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, show_window ? GLFW_TRUE : GLFW_FALSE);

    window = glfwCreateWindow(640, 480, "", NULL, NULL);

    if (!window)
    {
      sharedRelease();
      return false;
    }

    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true;  // Needed for core profile
    if (glewInit() != GLEW_OK)
    {
      std::cerr << "Failed to initialize GLEW" << std::endl;
      clear();
      return false;
    }

    // Black background
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);

    map_program_id = loadShaders("map_shader", vertex_shader, normals_fragment_shader);

    // Set the list of draw buffers.
    GLenum quad_draw_buffers[1] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, quad_draw_buffers);  // "1" is the size of quad_draw_buffers

    glGenBuffers(1, &quad_vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);

    // Create and compile our GLSL program from the shaders
    quad_program_id = loadShaders("fbo", quad_vertex_shader, quad_fragment_shader);
    fbo_tex_id = glGetUniformLocation(quad_program_id, "render_texture");

    return true;
  }

  void HeightmapImageDetail::RenderData::clear()
  {
    PROFILE(HeightmapImageDetail_clear);
    if (map_program_id)
    {
      glDeleteProgram(map_program_id);
      map_program_id = 0;
    }

    if (quad_program_id)
    {
      glDeleteProgram(quad_program_id);
      quad_program_id = 0;
    }

    if (quad_vertex_buffer)
    {
      glDeleteBuffers(1, &quad_vertex_buffer);
    }

    fbo_tex_id = 0xffffffffu;

    if (window)
    {
      glfwDestroyWindow(window);
      window = nullptr;

      sharedRelease();
    }
  }
}  // namespace ohm


HeightmapImage::HeightmapImage(const Heightmap &heightmap, ImageType type, NormalsMode normals_mode,
                               unsigned pixels_per_voxel)
  : imp_(new HeightmapImageDetail)
{
  PROFILE(HeightmapImageDetail);
  imp_->heightmap = &heightmap;
  imp_->type = type;
  imp_->normals_mode = normals_mode;
  imp_->pixels_per_voxel = pixels_per_voxel;
  imp_->render_data.init();
}


HeightmapImage::~HeightmapImage()
{
  PROFILE(_HeightmapImageDetail);
}


HeightmapImage::ImageType HeightmapImage::imageType()
{
  return imp_->type;
}


void HeightmapImage::setImageType(ImageType type)
{
  imp_->type = type;
}


HeightmapImage::NormalsMode HeightmapImage::normalsMode()
{
  return imp_->normals_mode;
}


void HeightmapImage::setNormalsMode(NormalsMode mode)
{
  imp_->normals_mode = mode;
}


unsigned HeightmapImage::pixelsPerVoxel()
{
  return imp_->pixels_per_voxel;
}


void HeightmapImage::setPixelsPerVoxel(unsigned ppv)
{
  imp_->pixels_per_voxel = ppv;
}


const uint8_t *HeightmapImage::bitmap(BitmapInfo *info) const
{
  *info = imp_->image_info;
  return (!imp_->image.empty()) ? imp_->image.data() : nullptr;
}


bool HeightmapImage::generateBitmap()
{
  PROFILE(HeightmapImage_generateBitmap);
  triangulate();

  // Render the mesh to a depth buffer.
  return renderHeightMesh(imp_->image_info.image_extents.minExtents(), imp_->image_info.image_extents.maxExtents(),
                          imp_->type, imp_->heightmap->heightmap().resolution());
}


void HeightmapImage::triangulate()
{
  PROFILE(HeightmapImage_triangulate);
  // Populate the coordinates.
  imp_->vertices.clear();
  imp_->vertex_normals.clear();
  imp_->indices.clear();
  imp_->tri_normals.clear();
  imp_->coords_2d.clear();

  // Walk heightmap voxels.
  const OccupancyMap &heightmap = imp_->heightmap->heightmap();
  const MapLayer *heightmap_layer = heightmap.layout().layer(HeightmapVoxel::kHeightmapLayer);
  const unsigned heightmap_layer_index = heightmap_layer->layerIndex();

  const glm::dvec3 up = imp_->heightmap->upAxisNormal();
  const glm::vec3 upf(up);
  glm::dvec3 point;
  glm::dvec3 min_map_ext, max_map_ext;

  heightmap.calculateExtents(min_map_ext, max_map_ext);

  for (auto voxel_iter = heightmap.begin(); voxel_iter != heightmap.end(); ++voxel_iter)
  {
    const VoxelConst &voxel = *voxel_iter;
    if (voxel.isOccupied())
    {
      const ohm::HeightmapVoxel *height_info = voxel.layerContent<const ohm::HeightmapVoxel *>(heightmap_layer_index);
      point = voxel.centreGlobal() + double(height_info->height) * up;
      imp_->coords_2d.push_back(point.x);
      imp_->coords_2d.push_back(point.y);
      imp_->vertices.push_back(glm::vec3(point - min_map_ext));

      min_map_ext.x = std::min(point.x, min_map_ext.x);
      min_map_ext.y = std::min(point.y, min_map_ext.y);
      min_map_ext.z = std::min(point.z, min_map_ext.z);

      max_map_ext.x = std::max(point.x, max_map_ext.x);
      max_map_ext.y = std::max(point.y, max_map_ext.y);
      max_map_ext.z = std::max(point.z, max_map_ext.z);
    }
  }

  imp_->image_info.image_extents = Aabb(min_map_ext, max_map_ext);

  // Triangulate.
  delaunator::Delaunator delaunay(imp_->coords_2d);

  imp_->vertex_normals.clear();
  imp_->vertex_normals.reserve(imp_->vertices.size());
  for (size_t i = 0; i < imp_->vertices.size(); ++i)
  {
    imp_->vertex_normals.push_back(glm::vec3(0.0f));
  }

  // Extract indices into imp_->indices.
  imp_->indices.resize(delaunay.triangles.size());
  if (!delaunay.triangles.empty())
  {
    imp_->indices.clear();
    imp_->indices.reserve(delaunay.triangles.size());
    imp_->tri_normals.reserve(delaunay.triangles.size() / 3);

    glm::vec3 tri[3];
    glm::vec3 normal;
    unsigned indices[3];
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

      // Adjust winding to suit rendering clipping.
      if (glm::dot(normal, glm::vec3(up)) < 0)
      {
        std::swap(indices[1], indices[2]);
        normal *= -1.0f;
      }
      imp_->indices.push_back(indices[0]);
      imp_->indices.push_back(indices[1]);
      imp_->indices.push_back(indices[2]);

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
    }
  }

  // Normalise data stored in vertex_normals to get the final normals.
  for (auto &vertex_normal : imp_->vertex_normals)
  {
    vertex_normal = glm::normalize(vertex_normal);
  }
}


bool HeightmapImage::renderHeightMesh(const glm::dvec3 &min_ext_spatial, const glm::dvec3 &max_ext_spatial,
                                      ImageType type, double voxel_resolution)
{
  PROFILE(HeightmapImage_renderHeightMesh);
  if (imp_->vertices.empty() || imp_->indices.empty())
  {
    return false;
  }

  // Get local extents. Vertices are always relative to min_ext_spatial
  glm::vec3 min_ext_vertices = glm::vec3(0.0f);
  glm::vec3 max_ext_vertices = glm::vec3(max_ext_spatial - min_ext_spatial);

  // {
  //   PlyMesh mesh;
  //   mesh.addVertices(imp_->vertices.data(), unsigned(imp_->vertices.size()));
  //   mesh.addTriangles(imp_->indices.data(), unsigned(imp_->indices.size() / 3));
  //   mesh.save("hm-mesh.ply", true);
  // }

  TES_TRIANGLES(g_3es, TES_COLOUR(White), glm::value_ptr(*imp_->vertices.data()), unsigned(imp_->vertices.size()),
                sizeof(*imp_->vertices.data()), imp_->indices.data(), unsigned(imp_->indices.size()));
  TES_SERVER_UPDATE(g_3es, 0.0f);
  TES_SERVER_UPDATE(g_3es, 0.0f);

  //----------------------------------------------------------------------------
  // Initialise GLFW
  //----------------------------------------------------------------------------

  // Resolve horizontal and vertical axes.
  // Set axes[0] and axes[1] to index the horizontal axes (across the heightmap) and axes[2] to th vertical.
  int axes[3];
  switch (imp_->heightmap->upAxisIndex())
  {
  case ohm::Heightmap::AxisNegZ:
    axes[0] = 1;
    axes[1] = 0;
    axes[2] = 2;
    break;
  case ohm::Heightmap::AxisNegY:
    axes[0] = 2;
    axes[1] = 0;
    axes[2] = 1;
    break;
  case ohm::Heightmap::AxisNegX:
    axes[0] = 2;
    axes[1] = 1;
    axes[2] = 0;
    break;
  case ohm::Heightmap::AxisX:
    axes[0] = 1;
    axes[1] = 2;
    axes[2] = 0;
    break;
  case ohm::Heightmap::AxisY:
    axes[0] = 0;
    axes[1] = 2;
    axes[2] = 1;
    break;
  default:
  case ohm::Heightmap::AxisZ:
    axes[0] = 0;
    axes[1] = 1;
    axes[2] = 2;
    break;
  }

  // Open a window and create its OpenGL context
  const unsigned render_width = std::max(
    1u, imp_->pixels_per_voxel * unsigned((max_ext_spatial[axes[0]] - min_ext_spatial[axes[0]]) / voxel_resolution));
  const unsigned render_height = std::max(
    1u, imp_->pixels_per_voxel * unsigned((max_ext_spatial[axes[1]] - min_ext_spatial[axes[1]]) / voxel_resolution));
  glfwSetWindowSize(imp_->render_data.window, render_width, render_height);

  //----------------------------------------------------------------------------
  // Rendering setup.
  //----------------------------------------------------------------------------
  glfwMakeContextCurrent(imp_->render_data.window);

  //----------------------------------------------------------------------------
  // Render data setup
  //----------------------------------------------------------------------------
  GLuint vertex_array_id;
  glGenVertexArrays(1, &vertex_array_id);
  glBindVertexArray(vertex_array_id);

  static_assert(sizeof(*imp_->indices.data()) == sizeof(GLuint), "GLuint/indices type size mismatch");

  GLuint vertex_buffer;
  glGenBuffers(1, &vertex_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
  glBufferData(GL_ARRAY_BUFFER, imp_->vertices.size() * sizeof(*imp_->vertices.data()), imp_->vertices.data(),
               GL_STATIC_DRAW);

  GLuint normals_buffer;
  glGenBuffers(1, &normals_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
  glBufferData(GL_ARRAY_BUFFER, imp_->vertex_normals.size() * sizeof(*imp_->vertex_normals.data()),
               imp_->vertex_normals.data(), GL_STATIC_DRAW);

  // Generate a buffer for the indices as well
  GLuint index_buffer;
  glGenBuffers(1, &index_buffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, imp_->indices.size() * sizeof(*imp_->indices.data()), imp_->indices.data(),
               GL_STATIC_DRAW);

  //----------------------------------------------------------------------------
  // FBO setup.
  //----------------------------------------------------------------------------
  // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
  GLuint frame_buffer_id = 0;
  glGenFramebuffers(1, &frame_buffer_id);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_id);

  // The texture we're going to render to
  GLuint render_texture;
  glGenTextures(1, &render_texture);

  // "Bind" the newly created texture : all future texture functions will modify this texture
  glBindTexture(GL_TEXTURE_2D, render_texture);

  // Give an empty image to OpenGL ( the last "0" means "empty" )
  if (imp_->type == kImageNormals)
  {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, render_width, render_height, 0, GL_RGB, GL_FLOAT, 0);
  }
  else
  {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, render_width, render_height, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
  }

  // Poor filtering
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // The depth buffer
  GLuint depth_render_buffer;
  glGenRenderbuffers(1, &depth_render_buffer);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_render_buffer);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, render_width, render_height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_render_buffer);

  // Alternative : Depth texture. Slower, but you can sample it later in your shader
  GLuint depth_texture;
  glGenTextures(1, &depth_texture);
  glBindTexture(GL_TEXTURE_2D, depth_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, render_width, render_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // Set "render_texture" as our colour attachment #0
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, render_texture, 0);

  // Depth texture alternative :
  glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_texture, 0);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // Always check that our framebuffer is ok
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
  {
    return false;
  }

  //----------------------------------------------------------------------------
  // Camera setup
  //----------------------------------------------------------------------------
  glUseProgram(imp_->render_data.map_program_id);

  // Get a handle for our "MVP" uniform
  GLuint mvp_matrix_id = glGetUniformLocation(imp_->render_data.map_program_id, "MVP");
  GLuint v_matrix_id = glGetUniformLocation(imp_->render_data.map_program_id, "V");
  GLuint m_matrix_id = glGetUniformLocation(imp_->render_data.map_program_id, "M");

  const float near_clip = 0.0f;
  const float camera_offset = 0.0f;
  // Near and far clip planes require sufficient buffering to exceed the min/max extents range.
  // So near is at 1.0f (to avoid some depth precision issues), far is near clip (1) + range + camera_offset (2)
  const glm::mat4 ProjectionMatrix =
    glm::ortho(-0.5f * max_ext_vertices[axes[0]] - min_ext_vertices[axes[0]],
               0.5f * max_ext_vertices[axes[0]] - min_ext_vertices[axes[0]],
               -0.5f * max_ext_vertices[axes[1]] - min_ext_vertices[axes[1]],
               0.5f * max_ext_vertices[axes[1]] - min_ext_vertices[axes[1]], near_clip,
               near_clip + camera_offset + max_ext_vertices[axes[2]] - min_ext_vertices[axes[2]]);
  // Look down from above.
  glm::vec3 eye = 0.5f * glm::vec3(min_ext_vertices + max_ext_vertices);
  glm::vec3 target = eye;
  eye[axes[2]] = max_ext_vertices[axes[2]] + camera_offset;
  target[axes[2]] = 0;
  glm::vec3 view_up(0.0f);
  view_up[axes[1]] = (imp_->heightmap->upAxis() >= 0) ? 1.0f : -1.0f;
  const glm::mat4 view_matrix = glm::lookAt(eye, target, view_up);

  const glm::mat4 model_matrix = glm::mat4(1.0);
  const glm::mat4 mvp_matrix = ProjectionMatrix * view_matrix * model_matrix;

  // Send our transformation to the currently bound shader,
  // in the "mvp_matrix" uniform
  glUniformMatrix4fv(mvp_matrix_id, 1, GL_FALSE, &mvp_matrix[0][0]);
  glUniformMatrix4fv(m_matrix_id, 1, GL_FALSE, &model_matrix[0][0]);
  glUniformMatrix4fv(v_matrix_id, 1, GL_FALSE, &view_matrix[0][0]);

  // Render to our framebuffer
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_id);

  glViewport(0, 0, render_width, render_height);

  // Clear the screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //-------------------------------------------
  // Render heightmap to FBO
  //-------------------------------------------
  glUseProgram(imp_->render_data.map_program_id);

  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
  glVertexAttribPointer(0,         // attribute 0. No particular reason for 0, but must match the layout in the shader.
                        3,         // size
                        GL_FLOAT,  // type
                        GL_FALSE,  // normalized?
                        0,         // stride
                        (void *)0  // array buffer offset
  );

  // Normals stream.
  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
  glVertexAttribPointer(1,         // attribute
                        3,         // size
                        GL_FLOAT,  // type
                        GL_TRUE,   // normalized?
                        0,         // stride
                        (void *)0  // array buffer offset
  );

  // Index buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

  // Draw the triangles !
  glDrawElements(GL_TRIANGLES,                   // mode
                 GLsizei(imp_->indices.size()),  // count
                 GL_UNSIGNED_INT,                // type
                 (void *)0                       // element array buffer offset
  );

  glDisableVertexAttribArray(0);

  // Render to window
  if (imp_->render_data.show_window)
  {
    // Set the list of draw buffers.
    GLenum quad_draw_buffers[1] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, quad_draw_buffers);  // "1" is the size of quad_draw_buffers

    do
    {
      // Render to the screen
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      // Render on the whole framebuffer, complete from the lower left corner to the upper right
      glViewport(0, 0, render_width, render_height);

      // Clear the screen
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      // Use our shader
      glUseProgram(imp_->render_data.quad_program_id);

      // Bind our texture in Texture Unit 0
      glActiveTexture(GL_TEXTURE0);
      if (type != kImageHeights)
      {
        glBindTexture(GL_TEXTURE_2D, render_texture);
      }
      else
      {
        glBindTexture(GL_TEXTURE_2D, depth_texture);
      }
      // Set our "render_texture" sampler to use Texture Unit 0
      glUniform1i(imp_->render_data.fbo_tex_id, 0);

      // 1rst attribute buffer : vertices
      glEnableVertexAttribArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, imp_->render_data.quad_vertex_buffer);
      glVertexAttribPointer(0,  // attribute 0. No particular reason for 0, but must match the layout in the shader.
                            3,  // size
                            GL_FLOAT,  // type
                            GL_FALSE,  // normalized?
                            0,         // stride
                            (void *)0  // array buffer offset
      );

      // Draw the triangles !
      glDrawArrays(GL_TRIANGLES, 0, 6);  // 2*3 indices starting at 0 -> 2 triangles

      glDisableVertexAttribArray(0);

      // Swap buffers
      glfwSwapBuffers(imp_->render_data.window);
      glfwPollEvents();
    } while (glfwGetKey(imp_->render_data.window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
             glfwWindowShouldClose(imp_->render_data.window) == 0);
  }

  imp_->image_info.image_width = render_width;
  imp_->image_info.image_height = render_height;
  imp_->image_info.image_extents = Aabb(min_ext_spatial, max_ext_spatial);
  imp_->image_info.type = imp_->type;
  if (type == kImageNormals)
  {
    // Read pixels:
    imp_->image_info.bpp = 3 * sizeof(float);
    imp_->image_info.byte_count = imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp;
    imp_->image.resize(imp_->image_info.byte_count);
    glBindTexture(GL_TEXTURE_2D, render_texture);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, imp_->image.data());
  }
  else if (type == kImageNormals888)
  {
    // Read pixels:
    imp_->image_info.bpp = 3;
    imp_->image_info.byte_count = imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp;
    imp_->image.resize(imp_->image_info.byte_count);
    glBindTexture(GL_TEXTURE_2D, render_texture);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, imp_->image.data());
  }
  else
  {
    // We are reading the depth buffer, which is float format. We size the image bytes appropriately.
    imp_->image_info.bpp = sizeof(float);
    imp_->image_info.byte_count = imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp;
    imp_->image.resize(imp_->image_info.byte_count);
    glBindTexture(GL_TEXTURE_2D, depth_texture);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, imp_->image.data());
  }

  // Cleanup
  glDeleteBuffers(1, &vertex_buffer);
  glDeleteBuffers(1, &normals_buffer);
  glDeleteBuffers(1, &index_buffer);
  glDeleteVertexArrays(1, &vertex_array_id);

  glDeleteFramebuffers(1, &frame_buffer_id);
  glDeleteTextures(1, &render_texture);
  glDeleteTextures(1, &depth_texture);
  glDeleteRenderbuffers(1, &depth_render_buffer);

  return 0;
}
