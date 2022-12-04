
include(GenerateExportHeader)

find_package(GLM)
find_package(Threads)

set(SOURCES
  Colour.cpp
  Colour.h
  p2p.h
  PlyMesh.cpp
  PlyMesh.h
  PlyPointStream.cpp
  PlyPointStream.h
  ProgressMonitor.cpp
  ProgressMonitor.h
  GlmStream.h
  Options.h
  Profile.cpp
  Profile.h
  ProfileMarker.cpp
  ProfileMarker.h
  SafeIO.cpp
  SafeIO.h
  ScopedTimeDisplay.cpp
  ScopedTimeDisplay.h
  VectorHash.h
)

# Split 3rd-party headers so we can include them as system headers and don't need to address clang-tidy issues.
set(PUBLIC_SKA_HEADERS
  3rdparty/ska/bytell_hash_map.hpp
  3rdparty/ska/flat_hash_map.hpp
  3rdparty/ska/ska_sort.hpp
  3rdparty/ska/unordered_map.hpp
)

set(PUBLIC_CXXOPT_HEADERS
  3rdparty/cxxopts/cxxopts.hpp
)

set(PUBLIC_HEADERS
  Colour.h
  GlmStream.h
  Options.h
  p2p.h
  PlyMesh.h
  PlyPointStream.h
  Profile.h
  ProfileMarker.h
  ProgressMonitor.h
  SafeIO.h
  ScopedTimeDisplay.h
  VectorHash.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmutil/OhmUtilExport.h"
)

add_library(ohmutil ${SOURCES})
clang_tidy_target(ohmutil EXCLUDE_MATCHES ".*ska/\\*\\.hpp")

generate_export_header(ohmutil
      EXPORT_MACRO_NAME ohmutil_API
      EXPORT_FILE_NAME ohmutil/OhmUtilExport.h
      STATIC_DEFINE ohmutil_STATIC)

target_link_libraries(ohmutil PUBLIC glm::glm logutil)

target_include_directories(ohmutil
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmutil>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmutil SYSTEM
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/3rdparty>
)

install(TARGETS ohmutil EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmutil
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmutil)
install(FILES ${PUBLIC_SKA_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmutil/ska)
install(FILES ${PUBLIC_CXXOPT_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmutil/cxxopts)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
