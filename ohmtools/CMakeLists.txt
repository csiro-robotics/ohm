
include(GenerateExportHeader)

find_package(GLM)

configure_file(ohmtoolsconfig.in.h ohmtoolsconfig.h)
set(SOURCES "${CMAKE_CURRENT_BINARY_DIR}/ohmtoolsconfig.h")

set(SOURCES
  ohmcloud.cpp
  ohmcloud.h
  ohmgen.cpp
  ohmgen.h
  ohmtoolsconfig.in.h
)

set(PUBLIC_HEADERS
  ohmcloud.h
  ohmgen.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmtoolsconfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmtoolsexport.h"
)

add_library(ohmtools ${SOURCES})

generate_export_header(ohmtools
      EXPORT_MACRO_NAME ohmtools_API
      EXPORT_FILE_NAME ohmtoolsexport.h
      STATIC_DEFINE OHMTOOLS_STATIC)

target_link_libraries(ohmtools ohm ohmutil)

target_include_directories(ohmtools
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_include_directories(ohmtools SYSTEM
  PUBLIC
    "${GLM_INCLUDE_DIR}"
)

install(TARGETS ohmtools EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(FILES ${PUBLIC_HEADERS} DESTINATION include/ohmtools)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
