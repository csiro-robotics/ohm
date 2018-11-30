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

#pragma GCC optimize("O0")

using namespace ohm;

namespace ohm
{
  struct HeightmapImageDetail
  {
    const Heightmap *heightmap;
    std::vector<glm::dvec3> vertices;
    std::vector<std::size_t> indices;
    std::vector<double> coords_2d;
  };
}  // namespace ohm

namespace
{
  // Fragment shader colour by input.
  const char *fragment_shader = "#version 330 core\n"
                                "// Ouput data\n"
                                "out vec3 color;\n"
                                "void main()\n"
                                "{\n"
                                "  // Output color = red\n"
                                "  color = vec3(1, 1, 1);\n"
                                "}";

  // Colour by depth.
  // const char *fragment_shader = "// frag\n"
  //                               "in float v_depth;\n"
  //                               "void main()\n"
  //                               "{\n"
  //                               "  gl_FragColor = vec4( vec3(v_depth), 1.0 );\n"
  //                               "}\n";

  // Vertex shader.
  const char *vertex_shader =
    "#version 330 core\n"
    "// Input vertex data, different for all executions of this shader.\n"
    "layout(location = 0) in vec3 vertexPosition_modelspace;\n"
    "out float v_depth;\n"
    "void main()\n"
    "{\n"
    "  gl_Position.xyz = vertexPosition_modelspace;\n"
    "  gl_Position.w = 1.0;\n"
    "  v_depth = gl_Position.z/gl_Position.w; // maybe use: (1.0 + gl_Position.z/gl_Position.w) / 2.0;\n"
    "}\n";

  const char *quad_fragment_shader = "#version 330 core\n"
                                     "// Ouput data\n"
                                     "layout(location = 0) out vec4 color;\n"
                                     "uniform sampler2D renderedTexture;\n"
                                     "in vec2 UV;\n"
                                     "void main()\n"
                                     "{\n"
                                     "  color = texture(renderedTexture, UV);\n"
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
  glm::dvec3 min_point(std::numeric_limits<double>::max());
  glm::dvec3 max_point(-std::numeric_limits<double>::max());

  for (auto voxel_iter = heightmap.begin(); voxel_iter != heightmap.end(); ++voxel_iter)
  {
    const VoxelConst &voxel = *voxel_iter;
    if (voxel.isOccupied())
    {
      const ohm::HeightmapVoxel *height_info = voxel.layerContent<const ohm::HeightmapVoxel *>(heightmap_layer_index);
      point = voxel.centreGlobal() + double(height_info->height) * up;
      imp_->coords_2d.push_back(point.x);
      imp_->coords_2d.push_back(point.y);
      // imp_->coords_2d.push_back(point.z);
      imp_->vertices.push_back(point);

      min_point.x = std::min(point.x, min_point.x);
      min_point.y = std::min(point.y, min_point.y);
      min_point.z = std::min(point.z, min_point.z);

      max_point.x = std::max(point.x, max_point.x);
      max_point.y = std::max(point.y, max_point.y);
      max_point.z = std::max(point.z, max_point.z);
    }
  }

  *min_ext = min_point;
  *max_ext = max_point;

  // Triangulate.
  delaunator::Delaunator delaunay(imp_->coords_2d);

  // Extract indices into imp_->indices.
  imp_->indices.resize(delaunay.triangles.size());
  if (!delaunay.triangles.empty())
  {
    static_assert(sizeof(*imp_->indices.data()) == sizeof(*delaunay.triangles.data()), "Data size mismatch");
    memcpy(imp_->indices.data(), delaunay.triangles.data(),
           sizeof(*delaunay.triangles.data()) * delaunay.triangles.size());
  }
}


bool HeightmapImage::renderHeightMesh(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext)
{
  // Initialise GLFW
  if (!glfwInit())
  {
    fprintf(stderr, "Failed to initialize GLFW\n");
    getchar();
    return false;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // Open a window and create its OpenGL context
  GLFWwindow *window = glfwCreateWindow(1024, 768, "Tutorial 14 - Render To Texture", NULL, NULL);
  if (window == NULL)
  {
    fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 "
                    "version of the tutorials.\n");
    getchar();
    glfwTerminate();
    return false;
  }
  glfwMakeContextCurrent(window);

  // We would expect width and height to be 1024 and 768
  int windowWidth = 1024;
  int windowHeight = 768;
  // But on MacOS X with a retina screen it'll be 1024*2 and 768*2, so we get the actual framebuffer size:
  glfwGetFramebufferSize(window, &windowWidth, &windowHeight);

  // Initialize GLEW
  glewExperimental = true;  // Needed for core profile
  if (glewInit() != GLEW_OK)
  {
    fprintf(stderr, "Failed to initialize GLEW\n");
    getchar();
    glfwTerminate();
    return false;
  }

  // Ensure we can capture the escape key being pressed below
  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
  // Hide the mouse and enable unlimited mouvement
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // Set the mouse at the center of the screen
  glfwPollEvents();
  glfwSetCursorPos(window, 1024 / 2, 768 / 2);

  // Dark blue background
  glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

  // Enable depth test
  glEnable(GL_DEPTH_TEST);
  // Accept fragment if it closer to the camera than the former one
  glDepthFunc(GL_LESS);

  // Cull triangles which normal is not towards the camera
  glEnable(GL_CULL_FACE);

  GLuint VertexArrayID;
  glGenVertexArrays(1, &VertexArrayID);
  glBindVertexArray(VertexArrayID);

  // Create and compile our GLSL program from the shaders
  GLuint programID = loadShaders("fbo", vertex_shader, fragment_shader);

  // Get a handle for our "MVP" uniform
  GLuint MatrixID = glGetUniformLocation(programID, "MVP");
  GLuint ViewMatrixID = glGetUniformLocation(programID, "V");
  GLuint ModelMatrixID = glGetUniformLocation(programID, "M");

  // Read our .obj file
  std::vector<glm::vec3> vertices;
  std::vector<unsigned> indices;

  vertices.resize(imp_->vertices.size());
  indices.resize(imp_->indices.size());

  for (size_t i = 0; i < imp_->vertices.size(); ++i)
  {
    vertices[i] = glm::vec3(imp_->vertices[i] - min_ext);
  }

  for (size_t i = 0; i < imp_->indices.size(); i += 3)
  {
    // Flip winding for render.
    indices[i + 0] = unsigned(imp_->indices[i + 0]);
    indices[i + 2] = unsigned(imp_->indices[i + 1]);
    indices[i + 1] = unsigned(imp_->indices[i + 2]);
  }

  {
    PlyMesh mesh;
    mesh.addVertices(vertices.data(), unsigned(vertices.size()));
    mesh.addTriangles(indices.data(), unsigned(indices.size() / 3));
    mesh.save("hm-mesh.ply", true);
  }

  TES_TRIANGLES(g_3es, TES_COLOUR(White), glm::value_ptr(*vertices.data()), unsigned(vertices.size()),
                sizeof(*vertices.data()), indices.data(), unsigned(indices.size()));
  TES_SERVER_UPDATE(g_3es, 0.0f);
  TES_SERVER_UPDATE(g_3es, 0.0f);

  // Load it into a VBO
  GLuint vertexbuffer;
  glGenBuffers(1, &vertexbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

  // Generate a buffer for the indices as well
  GLuint elementbuffer;
  glGenBuffers(1, &elementbuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned), &indices[0], GL_STATIC_DRAW);

  // Get a handle for our "LightPosition" uniform
  glUseProgram(programID);

  // ---------------------------------------------
  // Render to Texture - specific code begins here
  // ---------------------------------------------

  // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
  GLuint FramebufferName = 0;
  glGenFramebuffers(1, &FramebufferName);
  glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

  // The texture we're going to render to
  GLuint renderedTexture;
  glGenTextures(1, &renderedTexture);

  // "Bind" the newly created texture : all future texture functions will modify this texture
  glBindTexture(GL_TEXTURE_2D, renderedTexture);

  // Give an empty image to OpenGL ( the last "0" means "empty" )
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, windowWidth, windowHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

  // Poor filtering
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // The depth buffer
  GLuint depthrenderbuffer;
  glGenRenderbuffers(1, &depthrenderbuffer);
  glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, windowWidth, windowHeight);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);

  // Alternative : Depth texture. Slower, but you can sample it later in your shader
  GLuint depthTexture;
  glGenTextures(1, &depthTexture);
  glBindTexture(GL_TEXTURE_2D, depthTexture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 1024, 768, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // Set "renderedTexture" as our colour attachement #0
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

  // Depth texture alternative :
  glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);

  // Set the list of draw buffers.
  GLenum DrawBuffers[2] = { GL_COLOR_ATTACHMENT0, GL_DEPTH_ATTACHMENT };
  glDrawBuffers(2, DrawBuffers);  // "1" is the size of DrawBuffers

  // Always check that our framebuffer is ok
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
  {
    return false;
  }

  // The fullscreen quad's FBO
  static const GLfloat g_quad_vertex_buffer_data[] = {
    -1.0f, -1.0f, 0.0f, 1.0f, -1.0f, 0.0f, -1.0f, 1.0f, 0.0f, -1.0f, 1.0f, 0.0f, 1.0f, -1.0f, 0.0f, 1.0f, 1.0f, 0.0f,
  };

  GLuint quad_vertexbuffer;
  glGenBuffers(1, &quad_vertexbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);

  // Create and compile our GLSL program from the shaders
  GLuint quad_programID = loadShaders("quad", quad_vertex_shader, quad_fragment_shader);
  GLuint texID = glGetUniformLocation(quad_programID, "renderedTexture");

  std::vector<uint32_t> pixels;
  do
  {
    // Render to our framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
    // Render on the whole framebuffer, complete from the lower left corner to the upper right
    glViewport(0, 0, windowWidth, windowHeight);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use our shader
    glUseProgram(programID);

    // Compute the MVP matrix from keyboard and mouse input
    glm::mat4 ProjectionMatrix =
      glm::ortho(0.0f, 2.0f * float(max_ext.x - min_ext.x), 0.0f, 2.0f * float(max_ext.y - min_ext.y), 1.0f, 1000.0f);
    // Look down from above.
    // glm::mat4 ViewMatrix = glm::lookAt(glm::vec3(0, -100, 0),
    //                                    glm::vec3(0.0f), glm::vec3(0, 0, 1));
    glm::mat4 ViewMatrix = glm::lookAt(glm::vec3(0, 0, max_ext.z - min_ext.z + 1.0f),
                                       glm::vec3(0.5 * (max_ext - min_ext) - min_ext), glm::vec3(0, 1, 0));
    glm::mat4 ModelMatrix = glm::mat4(1.0);
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

    // Send our transformation to the currently bound shader,
    // in the "MVP" uniform
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
    glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
    glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

    // Bind our texture in Texture Unit 0
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, Texture);
    // // Set our "myTextureSampler" sampler to use Texture Unit 0
    // glUniform1i(TextureID, 0);

    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(0,         // attribute
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void *)0  // array buffer offset
    );

    // Index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

    // Draw the triangles !
    glDrawElements(GL_TRIANGLES,     // mode
                   indices.size(),   // count
                   GL_UNSIGNED_INT,  // type
                   (void *)0         // element array buffer offset
    );

    glDisableVertexAttribArray(0);

    // Render to the screen
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    // Render on the whole framebuffer, complete from the lower left corner to the upper right
    glViewport(0, 0, windowWidth, windowHeight);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use our shader
    glUseProgram(quad_programID);

    // Bind our texture in Texture Unit 0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, renderedTexture);
    // Set our "renderedTexture" sampler to use Texture Unit 0
    glUniform1i(texID, 0);

    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
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

    {
      // glGetTextureImage(renderTexture, );
      pixels.resize(windowWidth * windowHeight);
      glGetTextureImage(depthTexture, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT_8_8_8_8, pixels.size(), pixels.data());

      for (size_t p = 0; p < pixels.size(); ++p)
      {
        if (pixels[p] != 0)
        {
          int stopme = 1;
        }
      }
    }

  }  // Check if the ESC key was pressed or the window was closed
  while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);

  // Cleanup VBO and shader
  glDeleteBuffers(1, &vertexbuffer);
  glDeleteBuffers(1, &elementbuffer);
  glDeleteProgram(programID);

  glDeleteFramebuffers(1, &FramebufferName);
  glDeleteTextures(1, &renderedTexture);
  glDeleteTextures(1, &depthTexture);
  glDeleteRenderbuffers(1, &depthrenderbuffer);
  glDeleteBuffers(1, &quad_vertexbuffer);
  glDeleteVertexArrays(1, &VertexArrayID);

  // Close OpenGL window and terminate GLFW
  glfwTerminate();

  return true;
}
