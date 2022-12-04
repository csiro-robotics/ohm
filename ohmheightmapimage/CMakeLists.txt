
# ohmheightmapimage provides additional functionality for extracting data from an ohm heightmap.
include(GenerateExportHeader)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLM REQUIRED)
find_package(glfw3 REQUIRED)
if(OHM_FEATURE_THREADS)
  find_package(TBB)
endif(OHM_FEATURE_THREADS)

configure_file(OhmHeightmapImageConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmapimage/OhmHeightmapImageConfig.h")

set(SOURCES
  HeightmapImage.cpp
  HeightmapImage.h
  OhmHeightmapImageConfig.in.h
)

set(PUBLIC_HEADERS
  HeightmapImage.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmapimage/OhmHeightmapImageConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmapimage/OhmHeightmapUtilExport.h"
)
add_library(ohmheightmapimage ${SOURCES})
clang_tidy_target(ohmheightmapimage)

generate_export_header(ohmheightmapimage
      EXPORT_MACRO_NAME ohmheightmapimage_API
      EXPORT_FILE_NAME ohmheightmapimage/OhmHeightmapUtilExport.h
      STATIC_DEFINE OHMTOOLS_STATIC)

target_link_libraries(ohmheightmapimage PUBLIC ohmheightmap ohm ohmutil)
target_link_libraries(ohmheightmapimage
  PUBLIC
    glm::glm
    OpenGL::GL
    OpenGL::GLU
    $<$<BOOL:${OHM_FEATURE_THREADS}>:TBB::tbb>
)
if(BUILD_SHARED)
  # Link additional dependencies when building shared ohm.
  # Because ohm is shared and these are private, static depenencies, we do not need to propagate
  # the dependencies and limit this with $<BUILD_INTERFACE:>
  target_link_libraries(ohmheightmapimage
    PRIVATE
      # Link 3es if enabled.
      $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
      $<BUILD_INTERFACE:GLEW::GLEW>
      $<BUILD_INTERFACE:glfw>
  )
else(BUILD_SHARED)
  # With ohm static, we link depdencencies in such as way that the will propagate and need to be
  # found when ohm is linked.
  target_link_libraries(ohmheightmapimage
    PRIVATE
      # Link 3es if enabled.
      $<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>
      GLEW::GLEW
      glfw
  )
endif(BUILD_SHARED)

target_include_directories(ohmheightmapimage
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmheightmapimage>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmheightmapimage
  SYSTEM PRIVATE
    $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
)

install(TARGETS ohmheightmapimage EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmheightmapimage
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmheightmapimage)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
