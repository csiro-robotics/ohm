// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapImage.h"

#include <ohm/HeightmapMesh.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Voxel.h>

#include <ohmutil/Colour.h>
#include <ohmutil/Profile.h>

#include <3esservermacros.h>

#include <iostream>
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
  /// Relevant texture formats.
  enum AttachmentFormat
  {
    /// RGB 3 bytes per pixel, uint8_t per channel.
    kAfRgb8,
    /// RGB 12 bytes per pixel, float32 per channel.
    kAfRgb32f,
    /// Mono 4 bytes per pixel, float32 per channel.
    kAfMono32f
  };

  // Fragment shader colour by input.
  const char *normals_fragment_shader = "#version 330 core\n"
                                        "in vec3 v_normal;\n"
                                        "in vec3 v_colour;\n"
                                        "in float v_depth;\n"
                                        "// Ouput data\n"
                                        "out vec3 colour;\n"
                                        "void main()\n"
                                        "{\n"
                                        // "  colour = vec3(v_depth, v_depth, v_depth);\n"
                                        "  colour = 0.5 * (v_normal + vec3(1, 1, 1));\n"
                                        "}";
  const char *colours_fragment_shader = "#version 330 core\n"
                                        "in vec3 v_normal;\n"
                                        "in vec3 v_colour;\n"
                                        "in float v_depth;\n"
                                        "// Ouput data\n"
                                        "out vec3 colour;\n"
                                        "void main()\n"
                                        "{\n"
                                        "  colour = v_colour;\n"
                                        "}";

  // Vertex shader.
  const char *vertex_shader = "#version 330 core\n"
                              "// Input vertex data, different for all executions of this shader.\n"
                              "layout(location = 0) in vec3 vertexPosition_modelspace;\n"
                              "layout(location = 1) in vec3 vertexNormal_modelspace;\n"
                              "layout(location = 2) in vec3 vertexColour;\n"
                              "uniform mat4 MVP;\n"
                              "uniform mat4 V;\n"
                              "uniform mat4 M;\n"
                              "out vec3 v_normal;\n"
                              "out vec3 v_colour;\n"
                              "out float v_depth;\n"
                              "void main()\n"
                              "{\n"
                              "  gl_Position = gl_Position =  MVP * vec4(vertexPosition_modelspace,1);\n"
                              "  v_normal =  (V * M * vec4(vertexNormal_modelspace,0)).xyz;\n"
                              // "  v_normal =  vertexNormal_modelspace;\n"
                              "  v_colour = vertexColour;\n"
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
  const GLfloat kQuadVertexBufferData[] =  //
    {
      -1.0f, -1.0f, 0.0f,  //
      1.0f,  -1.0f, 0.0f,  //
      -1.0f, 1.0f,  0.0f,  //
      -1.0f, 1.0f,  0.0f,  //
      1.0f,  -1.0f, 0.0f,  //
      1.0f,  1.0f,  0.0f,  //
    };

  // Load vertex and fragment shader strings into a program.
  GLuint loadShaders(const char * /*name*/, const char *vertex_shader_code, const char *fragment_shader_code)
  {
    struct OnExit
    {
      std::function<void()> on_exit;
      OnExit(const std::function<void()> &on_exit)
        : on_exit(on_exit)
      {}
      ~OnExit() { on_exit(); }
    };

    // Create the shaders
    GLuint vertex_shader_id = 0;
    GLuint fragment_shader_id = 0;
    GLuint program_id = 0;
    GLint compile_success = GL_FALSE;

    // Setup error cleanup handling.
    OnExit on_exit(  //
      [&]()          //
      {
        if (!compile_success)
        {
          if (program_id)
          {
            if (vertex_shader_id)
            {
              glDetachShader(program_id, vertex_shader_id);
            }
            if (fragment_shader_id)
            {
              glDetachShader(program_id, fragment_shader_id);
            }
            glDeleteProgram(program_id);
          }

          if (vertex_shader_id)
          {
            glDeleteShader(vertex_shader_id);
            vertex_shader_id = 0;
          }

          if (fragment_shader_id)
          {
            glDeleteShader(fragment_shader_id);
            fragment_shader_id = 0;
          }
        }
      });

    int info_msg_length = 0;

    // Compile Vertex Shader
    // printf("Compiling shader vertex : %s\n", name);
    vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader_id, 1, &vertex_shader_code, nullptr);
    glCompileShader(vertex_shader_id);

    // Check Vertex Shader
    glGetShaderiv(vertex_shader_id, GL_COMPILE_STATUS, &compile_success);
    glGetShaderiv(vertex_shader_id, GL_INFO_LOG_LENGTH, &info_msg_length);
    if (info_msg_length > 0)
    {
      std::vector<char> error_msg(info_msg_length + 1);
      glGetShaderInfoLog(vertex_shader_id, info_msg_length, nullptr, error_msg.data());
      std::cerr << error_msg.data() << std::endl;
    }

    if (!compile_success)
    {
      return 0;
    }

    // Compile Fragment Shader
    // printf("Compiling shader fragment: %s\n", name);
    fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader_id, 1, &fragment_shader_code, nullptr);
    glCompileShader(fragment_shader_id);

    // Check Fragment Shader
    glGetShaderiv(fragment_shader_id, GL_COMPILE_STATUS, &compile_success);
    glGetShaderiv(fragment_shader_id, GL_INFO_LOG_LENGTH, &info_msg_length);
    if (info_msg_length > 0)
    {
      std::vector<char> error_msg(info_msg_length + 1);
      glGetShaderInfoLog(fragment_shader_id, info_msg_length, nullptr, &error_msg[0]);
      std::cerr << error_msg.data() << std::endl;
    }

    if (!compile_success)
    {
      return 0;
    }

    // Link the program
    // printf("Linking program\n");
    program_id = glCreateProgram();
    glAttachShader(program_id, vertex_shader_id);
    glAttachShader(program_id, fragment_shader_id);
    glLinkProgram(program_id);

    // Check the program
    glGetProgramiv(program_id, GL_LINK_STATUS, &compile_success);
    glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &info_msg_length);
    if (info_msg_length > 0)
    {
      std::vector<char> error_msg(info_msg_length + 1);
      glGetProgramInfoLog(program_id, info_msg_length, nullptr, &error_msg[0]);
      std::cerr << error_msg.data() << std::endl;
    }

    if (!compile_success)
    {
      return 0;
    }

    glDetachShader(program_id, vertex_shader_id);
    glDetachShader(program_id, fragment_shader_id);

    glDeleteShader(vertex_shader_id);
    vertex_shader_id = 0;
    glDeleteShader(fragment_shader_id);
    fragment_shader_id = 0;

    return program_id;
  }


  template <typename T>
  class ColourConverter
  {
  public:
    static glm::vec3 vec3(const T &c) { return glm::vec3(c); }
    static glm::vec4 vec4(const T &c) { return glm::vec4(c); }
  };

  template <>
  class ColourConverter<ohm::Colour>
  {
  public:
    static glm::vec3 vec3(const Colour &c) { return glm::vec3(c.rf(), c.gf(), c.bf()); }
    static glm::vec4 vec4(const Colour &c) { return glm::vec4(vec3(c), 1.0f); }
  };
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

  void textureBufferInfo(GLint texture_id)
  {
    struct TexParam
    {
      int id;
      const char *name;
    };

    static const TexParam params[] =  //
      {                               //
        { GL_TEXTURE_WIDTH, "width" },
        { GL_TEXTURE_HEIGHT, "height" },
        { GL_TEXTURE_DEPTH, "depth" },
        { GL_TEXTURE_INTERNAL_FORMAT, "format-internal" },
        { GL_TEXTURE_RED_TYPE, "red-type" },
        { GL_TEXTURE_GREEN_TYPE, "green-type" },
        { GL_TEXTURE_BLUE_TYPE, "blue-type" },
        { GL_TEXTURE_ALPHA_TYPE, "alpha-type" },
        { GL_TEXTURE_DEPTH_TYPE, "depth-type" },
        { GL_TEXTURE_RED_SIZE, "red-size" },
        { GL_TEXTURE_GREEN_SIZE, "green-size" },
        { GL_TEXTURE_BLUE_SIZE, "blue-size" },
        { GL_TEXTURE_ALPHA_SIZE, "alpha-size" },
        { GL_TEXTURE_DEPTH_SIZE, "depth-size" },
        { GL_TEXTURE_COMPRESSED, "compressed" },
        { GL_TEXTURE_COMPRESSED_IMAGE_SIZE, "compressed-size" },
        { GL_TEXTURE_BUFFER_OFFSET, "buffer-offset" },
        { GL_TEXTURE_BUFFER_SIZE, "buffer-size" }
      };
    static const size_t param_count = sizeof(params) / sizeof(params[0]);

    glBindTexture(GL_TEXTURE_2D, texture_id);
    int param_value = 0;
    printf("texture-info:\n");
    for (size_t i = 0; i < param_count; ++i)
    {
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, params[i].id, &param_value);
      printf("  %s: %d\n", params[i].name, param_value);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  unsigned makeMultipleOf(unsigned value, unsigned of)
  {
    const unsigned overflow = value % of;
    if (overflow)
    {
      return value + of - overflow;
    }
    return value;
  }
}  // namespace

namespace ohm
{
  struct HeightmapImageDetail
  {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> vertex_normals;
    std::vector<glm::vec3> vertex_colours;

    HeightmapImage::BitmapInfo image_info;
    std::vector<uint8_t> image;

    HeightmapImage::ImageType desired_type = HeightmapImage::kImageNormals;
    HeightmapImage::ImageType generated_type = HeightmapImage::kImageNormals;
    unsigned pixels_per_voxel = 1;

    struct RenderData
    {
      GLFWwindow *window = nullptr;
      GLuint mesh_normals_program_id = 0;
      GLuint mesh_colours_program_id = 0;
      GLuint quad_program_id = 0;
      GLuint quad_vertex_buffer = 0;
      GLuint fbo_tex_id = 0xffffffffu;
      // Debug visualisation flag.
      bool show_window = false;
      bool block_on_show_window = false;

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

    window = glfwCreateWindow(64, 64, "ohm", nullptr, nullptr);

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

    mesh_normals_program_id = loadShaders("mesh_normals_shader", vertex_shader, normals_fragment_shader);
    mesh_colours_program_id = loadShaders("mesh_colours_shader", vertex_shader, colours_fragment_shader);

    // Set the list of draw buffers.
    GLenum quad_draw_buffers[1] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, quad_draw_buffers);  // "1" is the size of quad_draw_buffers

    glGenBuffers(1, &quad_vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kQuadVertexBufferData), kQuadVertexBufferData, GL_STATIC_DRAW);

    // Create and compile our GLSL program from the shaders
    quad_program_id = loadShaders("fbo", quad_vertex_shader, quad_fragment_shader);
    fbo_tex_id = glGetUniformLocation(quad_program_id, "render_texture");

    return true;
  }

  void HeightmapImageDetail::RenderData::clear()
  {
    PROFILE(HeightmapImageDetail_clear);
    if (mesh_normals_program_id)
    {
      glDeleteProgram(mesh_normals_program_id);
      mesh_normals_program_id = 0;
    }

    if (mesh_colours_program_id)
    {
      glDeleteProgram(mesh_colours_program_id);
      mesh_colours_program_id = 0;
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


HeightmapImage::HeightmapImage(ImageType type, unsigned pixels_per_voxel)
  : imp_(new HeightmapImageDetail)
{
  PROFILE(HeightmapImageDetail);
  imp_->desired_type = imp_->generated_type = type;
  imp_->pixels_per_voxel = pixels_per_voxel;
  imp_->render_data.init();
}


HeightmapImage::~HeightmapImage()
{
  PROFILE(_HeightmapImageDetail);
}


HeightmapImage::ImageType HeightmapImage::desiredImageType()
{
  return imp_->desired_type;
}


void HeightmapImage::setDesiredImageType(ImageType type)
{
  imp_->desired_type = type;
}


unsigned HeightmapImage::pixelsPerVoxel()
{
  return imp_->pixels_per_voxel;
}


void HeightmapImage::setPixelsPerVoxel(unsigned ppv)
{
  imp_->pixels_per_voxel = ppv;
}


bool HeightmapImage::showWindow() const
{
  return imp_->render_data.show_window;
}


void HeightmapImage::setShowWindow(bool show_window)
{
  if (imp_->render_data.show_window != show_window)
  {
    imp_->render_data.show_window = show_window;
    if (show_window)
    {
      glfwShowWindow(imp_->render_data.window);
    }
    else
    {
      glfwHideWindow(imp_->render_data.window);
    }
  }
}


bool HeightmapImage::blockOnShowWindow() const
{
  return imp_->render_data.block_on_show_window;
}


void HeightmapImage::setBlockOnShowWindow(bool block)
{
  imp_->render_data.block_on_show_window = block;
}


const uint8_t *HeightmapImage::bitmap(BitmapInfo *info) const
{
  *info = imp_->image_info;
  return (!imp_->image.empty()) ? imp_->image.data() : nullptr;
}


bool HeightmapImage::generateBitmap(const Aabb &extents, double resolution, const glm::dvec3 *vertices,
                                    size_t vertex_count, const unsigned *indices, size_t index_count,
                                    const glm::dvec3 *vertex_normals, const Colour *colours, UpAxis up_axis)
{
  PROFILE(HeightmapImage_generateBitmap);
  // Render the mesh to a depth buffer.
  return renderHeightMesh(imp_->desired_type, extents, resolution, vertices, vertex_count, indices, index_count,
                          vertex_normals, colours, up_axis);
}


bool HeightmapImage::generateBitmap(const Aabb &extents, double resolution, const glm::dvec3 *vertices,
                                    size_t vertex_count, const unsigned *indices, size_t index_count,
                                    const glm::dvec3 *vertex_normals, const glm::vec3 *colours, UpAxis up_axis)
{
  PROFILE(HeightmapImage_generateBitmap);
  // Render the mesh to a depth buffer.
  return renderHeightMesh(imp_->desired_type, extents, resolution, vertices, vertex_count, indices, index_count,
                          vertex_normals, colours, up_axis);
}


bool HeightmapImage::generateBitmap(const Aabb &extents, double resolution, const glm::dvec3 *vertices,
                                    size_t vertex_count, const unsigned *indices, size_t index_count,
                                    const glm::dvec3 *vertex_normals, const glm::dvec3 *colours, UpAxis up_axis)
{
  PROFILE(HeightmapImage_generateBitmap);
  // Render the mesh to a depth buffer.
  return renderHeightMesh(imp_->desired_type, extents, resolution, vertices, vertex_count, indices, index_count,
                          vertex_normals, colours, up_axis);
}


bool HeightmapImage::generateBitmap(const HeightmapMesh &mesh, UpAxis up_axis)
{
  PROFILE(HeightmapImage_generateBitmap);
  const glm::vec3 *colours_ptr = nullptr;
#if 0
  std::vector<glm::vec3> colours(mesh.vertexCount());
  for (size_t i = 0; i < colours.size(); ++i)
  {
    colours[i] = (mesh.vertices()[i] - mesh.meshBoundingBox().minExtents()) / (mesh.meshBoundingBox().maxExtents() - mesh.meshBoundingBox().minExtents());
  }
  colours_ptr = colours.data();
#endif  // #
  return renderHeightMesh(imp_->desired_type, mesh.meshBoundingBox(), mesh.resolution(), mesh.vertices(),
                          mesh.vertexCount(), mesh.triangles(), mesh.triangleCount() * 3, mesh.vertexNormals(),
                          colours_ptr, up_axis);
}


template <typename NORMAL_VEC3, typename COLOUR_VEC>
bool HeightmapImage::renderHeightMesh(ImageType image_type, const Aabb &spatial_extents, double voxel_resolution,
                                      const glm::dvec3 *vertices, size_t vertex_count, const unsigned *indices,
                                      size_t index_count, const NORMAL_VEC3 *vertex_normals, const COLOUR_VEC *colours,
                                      UpAxis up_axis)
{
  PROFILE(HeightmapImage_renderHeightMesh);
  if (vertex_count == 0 || index_count == 0)
  {
    return false;
  }

  // First convert vertices to single precision coordinates local to spatial_extents.minExtents(). We need to render on
  // single precision.
  const glm::dvec3 *end_vertex = vertices + vertex_count;
  imp_->vertices.clear();
  imp_->vertices.reserve(vertex_count);
  for (auto v = vertices; v < end_vertex; ++v)
  {
    imp_->vertices.push_back(glm::vec3(*v - spatial_extents.minExtents()));
  }

  imp_->vertex_normals.clear();
  imp_->vertex_normals.reserve(vertex_count);
  if (vertex_normals)
  {
    const NORMAL_VEC3 *end_normal = vertex_normals + vertex_count;
    for (auto n = vertex_normals; n < end_normal; ++n)
    {
      imp_->vertex_normals.push_back(glm::vec3(*n));
    }
  }
  else
  {
    for (size_t i = 0; i < vertex_count; ++i)
    {
      imp_->vertex_normals.push_back(glm::vec3(0, 0, 1));
    }
  }

  imp_->vertex_colours.clear();
  imp_->vertex_colours.reserve(vertex_count);
  if (colours)
  {
    const COLOUR_VEC *end_colour = colours + vertex_count;
    for (auto c = colours; c < end_colour; ++c)
    {
      imp_->vertex_colours.push_back(ColourConverter<COLOUR_VEC>::vec3(*c));
    }
  }
  else
  {
    for (size_t i = 0; i < vertex_count; ++i)
    {
      imp_->vertex_colours.push_back(glm::vec3(1.0f));
    }
  }

  // Get local extents. Vertices are always relative to spatial_extents.minExtents()
  glm::vec3 min_ext_vertices = glm::vec3(0.0f);
  glm::vec3 max_ext_vertices = glm::vec3(spatial_extents.maxExtents() - spatial_extents.minExtents());

  TES_TRIANGLES(g_3es, TES_COLOUR(White), glm::value_ptr(*imp_->vertices.data()), unsigned(imp_->vertices.size()),
                sizeof(*imp_->vertices.data()), indices, unsigned(index_count));
  TES_SERVER_UPDATE(g_3es, 0.0f);
  TES_SERVER_UPDATE(g_3es, 0.0f);

  //----------------------------------------------------------------------------
  // Initialise GLFW
  //----------------------------------------------------------------------------

  // Resolve horizontal and vertical axes.
  // Set axes[0] and axes[1] to index the horizontal axes (across the heightmap) and axes[2] to th vertical.
  int axes[3];
  switch (up_axis)
  {
  case UpAxis::kNegZ:
    axes[0] = 1;
    axes[1] = 0;
    axes[2] = 2;
    break;
  case UpAxis::kNegY:
    axes[0] = 2;
    axes[1] = 0;
    axes[2] = 1;
    break;
  case UpAxis::kNegX:
    axes[0] = 2;
    axes[1] = 1;
    axes[2] = 0;
    break;
  case UpAxis::kX:
    axes[0] = 1;
    axes[1] = 2;
    axes[2] = 0;
    break;
  case UpAxis::kY:
    axes[0] = 0;
    axes[1] = 2;
    axes[2] = 1;
    break;
  default:
  case UpAxis::kZ:
    axes[0] = 0;
    axes[1] = 1;
    axes[2] = 2;
    break;
  }

  // Open a window and create its OpenGL context
  const unsigned pixels_per_voxel = std::max(imp_->pixels_per_voxel, 1u);
  const unsigned target_width =
    pixels_per_voxel * unsigned(std::ceil(spatial_extents.diagonal()[axes[0]] / voxel_resolution));
  const unsigned target_height =
    pixels_per_voxel * unsigned(std::ceil(spatial_extents.diagonal()[axes[1]] / voxel_resolution));

  // For some reason it seems the either the buffer size must be a multiple of 16, or the individual dimensions of 8.
  // Making both of 8 addresses both.
  // Otherwise the render texture reports the correct size, but overruns when getting the image back.
  const unsigned render_width = makeMultipleOf(target_width, 8);
  const unsigned render_height = makeMultipleOf(target_height, 8);

  // Adjust the extents to cover the additional pixels.
  max_ext_vertices[axes[0]] += float(render_width - target_width) / float(pixels_per_voxel) * float(voxel_resolution);
  max_ext_vertices[axes[1]] += float(render_height - target_height) / float(pixels_per_voxel) * float(voxel_resolution);

  //----------------------------------------------------------------------------
  // Rendering setup.
  //----------------------------------------------------------------------------

  glfwSetWindowSize(imp_->render_data.window, render_width, render_height);
  glfwMakeContextCurrent(imp_->render_data.window);
  glViewport(0, 0, render_width, render_height);

  //----------------------------------------------------------------------------
  // Render data setup
  //----------------------------------------------------------------------------
  GLuint vertex_array_id;
  glGenVertexArrays(1, &vertex_array_id);
  glBindVertexArray(vertex_array_id);

  static_assert(sizeof(*indices) == sizeof(GLuint), "GLuint/indices type size mismatch");

  if (imp_->vertex_normals.empty())
  {
    return false;
  }

  // Bind vertex buffer
  GLuint vertex_buffer;
  glGenBuffers(1, &vertex_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
  glBufferData(GL_ARRAY_BUFFER, imp_->vertices.size() * sizeof(*imp_->vertices.data()), imp_->vertices.data(),
               GL_STATIC_DRAW);

  // Bind normals buffer.
  GLuint normals_buffer;
  glGenBuffers(1, &normals_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
  glBufferData(GL_ARRAY_BUFFER, vertex_count * sizeof(*imp_->vertex_normals.data()), imp_->vertex_normals.data(),
               GL_STATIC_DRAW);

  // Bind colour buffer.
  GLuint colours_buffer;
  glGenBuffers(1, &colours_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, colours_buffer);
  glBufferData(GL_ARRAY_BUFFER, vertex_count * sizeof(*imp_->vertex_colours.data()), imp_->vertex_colours.data(),
               GL_STATIC_DRAW);

  // Generate a buffer for the indices as well
  GLuint index_buffer;
  glGenBuffers(1, &index_buffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_count * sizeof(*indices), indices, GL_STATIC_DRAW);

  //----------------------------------------------------------------------------
  // FBO setup.
  //----------------------------------------------------------------------------
  // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
  GLuint frame_buffer_id = 0;
  glGenFramebuffers(1, &frame_buffer_id);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_id);

  // Setup a colour attachment texture.
  GLuint render_texture;
  AttachmentFormat render_texture_format = kAfRgb8;
  glGenTextures(1, &render_texture);

  // "Bind" the newly created texture : all future texture functions will modify this texture
  glBindTexture(GL_TEXTURE_2D, render_texture);

  if (image_type == kImageNormals && colours == nullptr)
  {
    render_texture_format = kAfRgb32f;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, render_width, render_height, 0, GL_RGB, GL_FLOAT, nullptr);
  }
  else
  {
    render_texture_format = kAfRgb8;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, render_width, render_height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
  }

  // Poor filtering
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glBindTexture(GL_TEXTURE_2D, 0);

  // // The depth buffer
  // GLuint depth_render_buffer;
  // glGenRenderbuffers(1, &depth_render_buffer);
  // glBindRenderbuffer(GL_RENDERBUFFER, depth_render_buffer);
  // glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, render_width, render_height);
  // glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_render_buffer);

  // Alternative : Depth texture. Slower, but you can sample it later in your shader
  GLuint depth_texture;
  AttachmentFormat depth_texture_format = kAfMono32f;
  glGenTextures(1, &depth_texture);
  glBindTexture(GL_TEXTURE_2D, depth_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, render_width, render_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
               nullptr);
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glBindTexture(GL_TEXTURE_2D, 0);

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

  const bool output_render_texture = (colours != nullptr || image_type != kImageHeights);
  GLuint output_texture_id = (output_render_texture) ? render_texture : depth_texture;
  AttachmentFormat output_texture_format = (output_render_texture) ? render_texture_format : depth_texture_format;
  GLenum output_format_type = (output_render_texture) ? GL_RGB : GL_DEPTH_COMPONENT;

  //----------------------------------------------------------------------------
  // Camera setup
  //----------------------------------------------------------------------------
  int mesh_program_id =
    (colours) ? imp_->render_data.mesh_colours_program_id : imp_->render_data.mesh_normals_program_id;
  glUseProgram(mesh_program_id);

  // Get a handle for our "MVP" uniform
  GLuint mvp_matrix_id = glGetUniformLocation(mesh_program_id, "MVP");
  GLuint v_matrix_id = glGetUniformLocation(mesh_program_id, "V");
  GLuint m_matrix_id = glGetUniformLocation(mesh_program_id, "M");

  const float near_clip = 0.0f;
  const float camera_offset = 0.0f;
  // Near and far clip planes require sufficient buffering to exceed the min/max extents range.
  // So near is at 1.0f (to avoid some depth precision issues), far is near clip (1) + range + camera_offset (2)
  const glm::mat4 projection_matrix =
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
  view_up[axes[1]] = (int(up_axis) >= 0) ? 1.0f : -1.0f;
  const glm::mat4 view_matrix = glm::lookAt(eye, target, view_up);

  const glm::mat4 model_matrix = glm::mat4(1.0);
  const glm::mat4 mvp_matrix = projection_matrix * view_matrix * model_matrix;

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
  glUseProgram(mesh_program_id);

  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
  glVertexAttribPointer(0,         // index
                        3,         // size
                        GL_FLOAT,  // type
                        GL_FALSE,  // normalized?
                        0,         // stride
                        nullptr    // array buffer offset
  );

  // Normals stream.
  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
  glVertexAttribPointer(1,         // index
                        3,         // size
                        GL_FLOAT,  // type
                        GL_TRUE,   // normalized?
                        0,         // stride
                        nullptr    // array buffer offset
  );

  // Vertex colour stream.
  glEnableVertexAttribArray(2);
  glBindBuffer(GL_ARRAY_BUFFER, colours_buffer);
  glVertexAttribPointer(2,         // index
                        3,         // size
                        GL_FLOAT,  // type
                        GL_FALSE,  // normalized?
                        0,         // stride
                        nullptr    // array buffer offset
  );

  // Index buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

  // Draw the triangles !
  glDrawElements(GL_TRIANGLES,          // mode
                 GLsizei(index_count),  // count
                 GL_UNSIGNED_INT,       // type
                 nullptr                // element array buffer offset
  );

  glDisableVertexAttribArray(0);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

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
      glBindTexture(GL_TEXTURE_2D, output_texture_id);
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
                            nullptr    // array buffer offset
      );

      // Draw the triangles !
      glDrawArrays(GL_TRIANGLES, 0, 6);  // 2*3 indices starting at 0 -> 2 triangles

      glDisableVertexAttribArray(0);

      // Swap buffers
      glfwSwapBuffers(imp_->render_data.window);
      glfwSwapBuffers(imp_->render_data.window);
      glfwPollEvents();
    } while (imp_->render_data.block_on_show_window &&
             glfwGetKey(imp_->render_data.window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
             glfwWindowShouldClose(imp_->render_data.window) == 0);
  }

  imp_->image_info.image_width = render_width;
  imp_->image_info.image_height = render_height;

  imp_->image_info.image_extents =
    Aabb(spatial_extents.minExtents(), spatial_extents.minExtents() + glm::dvec3(max_ext_vertices));
  imp_->image_info.type = (colours) ? kImageVertexColours888 : image_type;

  // Read pixels:
  switch (output_texture_format)
  {
  case kAfRgb8:
    imp_->image_info.bpp = 3;
    imp_->image_info.byte_count = imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp;
    imp_->image.resize(imp_->image_info.byte_count);
    // imp_->image.resize(imp_->image_info.byte_count * 2);
    // memset(imp_->image.data() + imp_->image_info.byte_count, 0xcf, imp_->image_info.byte_count);

    // textureBufferInfo(output_texture_id);
    glBindTexture(GL_TEXTURE_2D, output_texture_id);
    glGetTexImage(GL_TEXTURE_2D, 0, output_format_type, GL_UNSIGNED_BYTE, imp_->image.data());
    // if (imp_->image[imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp] != 0xcf)
    // {
    //   unsigned last_bad_index = imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp;
    //   while (last_bad_index < imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp * 2
    //   &&
    //          imp_->image[last_bad_index] != 0xcf)
    //   {
    //     ++last_bad_index;
    //   }
    //   printf("(%u,%u) x %u -> %u : prev(%u,%u) bad pixel to %u\n\n", render_width, render_height, image_size,
    //          image_size * imp_->image_info.bpp, prev_width, prev_height,
    //          (last_bad_index - imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp) /
    //            imp_->image_info.bpp);
    // }
    // else
    // {
    //   printf("(%u,%u) x %u -> %u : prev(%u,%u)\n",
    //          render_width, render_height, image_size, image_size * imp_->image_info.bpp, prev_width, prev_height);
    // }
    // savePng(imp_->image, render_width, render_height);
    break;
  case kAfRgb32f:
    imp_->image_info.bpp = 3 * sizeof(float);
    imp_->image_info.byte_count = imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp;
    imp_->image.resize(imp_->image_info.byte_count);
    glBindTexture(GL_TEXTURE_2D, output_texture_id);
    glGetTexImage(GL_TEXTURE_2D, 0, output_format_type, GL_FLOAT, imp_->image.data());
    break;
  case kAfMono32f:
    // We are reading the depth buffer, which is float format. We size the image bytes appropriately.
    imp_->image_info.bpp = sizeof(float);
    imp_->image_info.byte_count = imp_->image_info.image_width * imp_->image_info.image_height * imp_->image_info.bpp;
    imp_->image.resize(imp_->image_info.byte_count);
    glBindTexture(GL_TEXTURE_2D, output_texture_id);
    glGetTexImage(GL_TEXTURE_2D, 0, output_format_type, GL_FLOAT, imp_->image.data());
    break;
  default:
    // Unkown type;
    imp_->image_info.bpp = 0;
    imp_->image_info.byte_count = 0;
    imp_->image_info.image_width = 0;
    imp_->image_info.image_height = 0;
    imp_->image.resize(0);
    break;
  }

  // Cleanup
  glDeleteBuffers(1, &vertex_buffer);
  glDeleteBuffers(1, &normals_buffer);
  glDeleteBuffers(1, &colours_buffer);
  glDeleteBuffers(1, &index_buffer);
  glDeleteVertexArrays(1, &vertex_array_id);

  glDeleteFramebuffers(1, &frame_buffer_id);
  glDeleteTextures(1, &render_texture);
  glDeleteTextures(1, &depth_texture);
  // glDeleteRenderbuffers(1, &depth_render_buffer);

  return 0;
}
