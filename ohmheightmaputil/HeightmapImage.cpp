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

#include <3esservermacros.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>

#include <glm/ext.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#pragma GCC optimize("O0")

using namespace ohm;

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
  };
}  // namespace ohm

namespace
{
  // Fragment shader colour by input.
  const char *normals_fragment_shader = "#version 330 core\n"
                                        "in vec3 v_colour;\n"
                                        "in float v_depth;\n"
                                        "// Ouput data\n"
                                        "out vec3 color;\n"
                                        "void main()\n"
                                        "{\n"
                                        // "  color = vec3(v_depth, v_depth, v_depth);\n"
                                        "  color = v_colour;\n"
                                        "}";
  const char *depth_fragment_shader = "#version 330 core\n"
                                      "in vec3 v_colour;\n"
                                      "in float v_depth;\n"
                                      "// Ouput data\n"
                                      "out vec3 color;\n"
                                      "void main()\n"
                                      "{\n"
                                      "  color = vec3(v_depth, v_depth, v_depth);\n"
                                      "}";

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
                              "out vec3 v_colour;\n"
                              "out float v_depth;\n"
                              "void main()\n"
                              "{\n"
                              "  gl_Position = gl_Position =  MVP * vec4(vertexPosition_modelspace,1);\n"
                              "  v_colour = vertexNormal_modelspace;\n"
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

  // Load vertex and fragment shader strings into a program.
  GLuint loadShaders(const char *name, const char *vertex, const char *fragment)
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
    printf("Compiling shader vertex : %s\n", name);
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
      printf("%s\n", &VertexShaderErrorMessage[0]);
    }

    // Compile Fragment Shader
    printf("Compiling shader fragment: %s\n", name);
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
      printf("%s\n", &FragmentShaderErrorMessage[0]);
    }

    // Link the program
    printf("Linking program\n");
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
      printf("%s\n", &ProgramErrorMessage[0]);
    }

    glDetachShader(ProgramID, VertexShaderID);
    glDetachShader(ProgramID, FragmentShaderID);

    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    return ProgramID;
  }
}  // namespace


HeightmapImage::HeightmapImage(const Heightmap &heightmap)
  : imp_(new HeightmapImageDetail)
{
  imp_->heightmap = &heightmap;
}


HeightmapImage::~HeightmapImage()
{}


bool HeightmapImage::extractBitmap(uint8_t *buffer, size_t buffer_size, BitmapInfo *info)
{
  glm::dvec3 min_ext, max_ext;
  triangulate(&min_ext, &max_ext);

  // Render the mesh to a depth buffer.
  renderHeightMesh(min_ext, max_ext);
}


void HeightmapImage::triangulate(glm::dvec3 *min_ext, glm::dvec3 *max_ext)
{
  // Populate the coordinates.
  imp_->coords_2d.clear();

  // Walk heightmap voxels.
  const OccupancyMap &heightmap = imp_->heightmap->heightmap();
  const MapLayer *heightmap_layer = heightmap.layout().layer(HeightmapVoxel::kHeightmapLayer);
  const unsigned heightmap_layer_index = heightmap_layer->layerIndex();

  const glm::dvec3 up = imp_->heightmap->upAxisNormal();
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

  *min_ext = min_map_ext;
  *max_ext = max_map_ext;

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

      imp_->vertex_normals[indices[0]] += normal;
      imp_->vertex_normals[indices[1]] += normal;
      imp_->vertex_normals[indices[2]] += normal;

      imp_->tri_normals.push_back(normal);
    }
  }

  // Normalise data stored in vertex_normals to get the final normals.
  for (auto &vertex_normal : imp_->vertex_normals)
  {
    vertex_normal = glm::normalize(vertex_normal);
  }
}


bool HeightmapImage::renderHeightMesh(const glm::dvec3 &min_ext_a, const glm::dvec3 &max_ext_a)
{
  if (imp_->vertices.empty() || imp_->indices.empty())
  {
    return false;
  }

  glm::dvec3 min_ext = min_ext_a;
  glm::dvec3 max_ext = max_ext_a;
  min_ext = max_ext = imp_->vertices[0];
  for (const auto &v : imp_->vertices)
  {
    for (int i = 0; i < 3; ++i)
    {
      min_ext[i] = std::min<double>(v[i], min_ext[i]);
      max_ext[i] = std::max<double>(v[i], max_ext[i]);
    }
  }

  {
    PlyMesh mesh;
    mesh.addVertices(imp_->vertices.data(), unsigned(imp_->vertices.size()));
    mesh.addTriangles(imp_->indices.data(), unsigned(imp_->indices.size() / 3));
    mesh.save("hm-mesh.ply", true);
  }

  TES_TRIANGLES(g_3es, TES_COLOUR(White), glm::value_ptr(*vertices.data()), unsigned(vertices.size()),
                sizeof(*vertices.data()), indices.data(), unsigned(indices.size()));
  TES_SERVER_UPDATE(g_3es, 0.0f);
  TES_SERVER_UPDATE(g_3es, 0.0f);

  // Initialise GLFW
  if (!glfwInit())
  {
    fprintf(stderr, "Failed to initialize GLFW\n");
    getchar();
    return -1;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // Open a window and create its OpenGL context
  GLFWwindow *window = glfwCreateWindow(1024, 768, "Tutorial 02 - Red triangle", NULL, NULL);
  if (window == NULL)
  {
    fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 "
                    "version of the tutorials.\n");
    getchar();
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);

  // Initialize GLEW
  glewExperimental = true;  // Needed for core profile
  if (glewInit() != GLEW_OK)
  {
    fprintf(stderr, "Failed to initialize GLEW\n");
    getchar();
    glfwTerminate();
    return -1;
  }

  //----------------------------------------------------------------------------
  // Rendering setup.
  //----------------------------------------------------------------------------

  // We would expect width and height to be 1024 and 768
  int windowWidth = 1024;
  int windowHeight = 768;

  // But on MacOS X with a retina screen it'll be 1024*2 and 768*2, so we get the actual framebuffer size:
  glfwGetFramebufferSize(window, &windowWidth, &windowHeight);

  // Ensure we can capture the escape key being pressed below
  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

  // Dark blue background
  glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

  // // Enable depth test
  glEnable(GL_DEPTH_TEST);
  // Accept fragment if it closer to the camera than the former one
  glDepthFunc(GL_LESS);

  // Cull triangles which normal is not towards the camera
  // glEnable(GL_CULL_FACE);

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
  // Shader setup
  //----------------------------------------------------------------------------

  // Create and compile our GLSL program from the shaders
  GLuint map_program_id = loadShaders("map_shader", vertex_shader, normals_fragment_shader);

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
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, windowWidth, windowHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

  // Poor filtering
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // The depth buffer
  GLuint depth_render_buffer;
  glGenRenderbuffers(1, &depth_render_buffer);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_render_buffer);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, windowWidth, windowHeight);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_render_buffer);

  // Alternative : Depth texture. Slower, but you can sample it later in your shader
  GLuint depth_texture;
  glGenTextures(1, &depth_texture);
  glBindTexture(GL_TEXTURE_2D, depth_texture);
  glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT24, windowWidth, windowHeight, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // Set "render_texture" as our colour attachment #0
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, render_texture, 0);

  // Depth texture alternative :
  glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_texture, 0);


  // Set the list of draw buffers.
  GLenum quad_draw_buffers[1] = { GL_COLOR_ATTACHMENT0 };
  glDrawBuffers(1, quad_draw_buffers);  // "1" is the size of quad_draw_buffers

  // Always check that our framebuffer is ok
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
  {
    return false;
  }

  // The fullscreen quad's FBO
  static const GLfloat s_quad_vertex_buffer_data[] =  //
    {
      -1.0f, -1.0f, 0.0f,  //
      1.0f,  -1.0f, 0.0f,  //
      -1.0f, 1.0f,  0.0f,  //
      -1.0f, 1.0f,  0.0f,  //
      1.0f,  -1.0f, 0.0f,  //
      1.0f,  1.0f,  0.0f,  //
    };

  GLuint quad_vertex_buffer;
  glGenBuffers(1, &quad_vertex_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, quad_vertex_buffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(s_quad_vertex_buffer_data), s_quad_vertex_buffer_data, GL_STATIC_DRAW);

  // Create and compile our GLSL program from the shaders
  GLuint quad_program_id = loadShaders("fbo", quad_vertex_shader, quad_fragment_shader);
  GLuint fbo_tex_id = glGetUniformLocation(quad_program_id, "render_texture");

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  //----------------------------------------------------------------------------
  // Camera setup
  //----------------------------------------------------------------------------
  glUseProgram(map_program_id);

  // Get a handle for our "MVP" uniform
  GLuint mvp_matrix_id = glGetUniformLocation(map_program_id, "MVP");

  const float near_clip = 0.0f;
  const float camera_offset = 0.0f;
  // Near and far clip planes require sufficient buffering to exceed the min/max extents range.
  // So near is at 1.0f (to avoid some depth precision issues), far is near clip (1) + range + camera_offset (2)
  // const glm::mat4 ProjectionMatrix = glm::ortho(float(min_ext.x), float(max_ext.x),
  //                                               float(min_ext.y), float(max_ext.y),
  const glm::mat4 ProjectionMatrix = glm::ortho(
    -0.5f * float(max_ext.x - min_ext.x), 0.5f * float(max_ext.x - min_ext.x), -0.5f * float(max_ext.y - min_ext.y),
    0.5f * float(max_ext.y - min_ext.y), near_clip, near_clip + camera_offset + float(max_ext.z - min_ext.z));
  // Look down from above.
  glm::vec3 eye = 0.5f * glm::vec3(min_ext + max_ext);
  glm::vec3 target = eye;
  eye.z = max_ext.z + camera_offset;
  target.z = 0;
  const glm::vec3 up(0, 1, 0);
  const glm::mat4 ViewMatrix = glm::lookAt(eye, target, up);

  const glm::mat4 ModelMatrix = glm::mat4(1.0);
  const glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

  // Send our transformation to the currently bound shader,
  // in the "MVP" uniform
  glUniformMatrix4fv(mvp_matrix_id, 1, GL_FALSE, &MVP[0][0]);

  bool render_depth_buffer = false;
  do
  {
    // Render to our framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_id);

    glViewport(0, 0, windowWidth, windowHeight);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //-------------------------------------------
    // Render heightmap to FBO
    //-------------------------------------------
    glUseProgram(map_program_id);

    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glVertexAttribPointer(0,  // attribute 0. No particular reason for 0, but must match the layout in the shader.
                          3,  // size
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
    glDrawElements(GL_TRIANGLES,          // mode
                   imp_->indices.size(),  // count
                   GL_UNSIGNED_INT,       // type
                   (void *)0              // element array buffer offset
    );

    glDisableVertexAttribArray(0);

    //-------------------------------------------
    // Render fbo to screen
    //-------------------------------------------
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    // Render on the whole framebuffer, complete from the lower left corner to the upper right
    glViewport(0, 0, windowWidth, windowHeight);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use our shader
    glUseProgram(quad_program_id);

    // Bind our texture in Texture Unit 0
    glActiveTexture(GL_TEXTURE0);
    if (!render_depth_buffer)
    {
      glBindTexture(GL_TEXTURE_2D, render_texture);
    }
    else
    {
      glBindTexture(GL_TEXTURE_2D, depth_texture);
    }
    // Set our "render_texture" sampler to use Texture Unit 0
    glUniform1i(fbo_tex_id, 0);

    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vertex_buffer);
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
    glfwSwapBuffers(window);
    glfwPollEvents();

  }  // Check if the ESC key was pressed or the window was closed
  while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);

  // Cleanup
  glDeleteBuffers(1, &vertex_buffer);
	glDeleteBuffers(1, &normals_buffer);
  glDeleteBuffers(1, &index_buffer);
  glDeleteVertexArrays(1, &vertex_array_id);
  glDeleteProgram(map_program_id);

	glDeleteFramebuffers(1, &frame_buffer_id);
	glDeleteTextures(1, &render_texture);
	glDeleteTextures(1, &depth_texture);
	glDeleteRenderbuffers(1, &depth_render_buffer);
	glDeleteBuffers(1, &quad_vertex_buffer);
  glDeleteProgram(quad_program_id);

  // Close OpenGL window and terminate GLFW
  glfwTerminate();

  return 0;
}
