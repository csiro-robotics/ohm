
# ohmheightmaputil provides additional functionality for extracting data from an ohm heightmap.
include(GenerateExportHeader)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLM REQUIRED)

find_package(glfw3 REQUIRED)

configure_file(OhmHeightmapUtilConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmaputil/OhmHeightmapUtilConfig.h")

set(SOURCES
  HeightmapImage.cpp
  HeightmapImage.h
  OhmHeightmapUtilConfig.in.h
)

set(PUBLIC_HEADERS
  HeightmapImage.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmaputil/OhmHeightmapUtilConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmaputil/OhmHeightmapUtilExport.h"
)
add_library(ohmheightmaputil ${SOURCES})

generate_export_header(ohmheightmaputil
      EXPORT_MACRO_NAME ohmheightmaputil_API
      EXPORT_FILE_NAME ohmheightmaputil/OhmHeightmapUtilExport.h
      STATIC_DEFINE OHMTOOLS_STATIC)

target_link_libraries(ohmheightmaputil PUBLIC ohm ohmutil)
target_link_libraries(ohmheightmaputil PUBLIC
  OpenGL::GL
  OpenGL::GLU
  ${GLEW_LIBRARIES}
  glfw
  )# OpenGL::GLX)

target_include_directories(ohmheightmaputil
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmheightmaputil>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmheightmaputil SYSTEM
  PUBLIC
    "${GLM_INCLUDE_DIR}"
    "${GLEW_INCLUDE_DIRS}"
)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmheightmaputil PUBLIC 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmheightmaputil PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

install(TARGETS ohmheightmaputil EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmheightmaputil
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmheightmaputil)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
