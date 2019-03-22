
include(GenerateExportHeader)

find_package(GLM)
find_package(Threads)

set(SOURCES
  ska/bytell_hash_map.hpp
  ska/flat_hash_map.hpp
  ska/ska_sort.hpp
  ska/unordered_map.hpp
  Colour.cpp
  Colour.h
  cxxopts.hpp
  p2p.h
  # OhmUtil.cpp
  OhmUtil.h
  PlyMesh.cpp
  PlyMesh.h
  PointsDoubleBuffer.h
  PointsDoubleBuffer.inl
  PointsRingBuffer.h
  PointsRingBuffer.inl
  ProgressMonitor.cpp
  ProgressMonitor.h
  GlmStream.h
  LineWalk.h
  Options.h
  Profile.cpp
  Profile.h
  ProfileMarker.cpp
  ProfileMarker.h
  SafeIO.cpp
  SafeIO.h
  ScopedTimeDisplay.cpp
  ScopedTimeDisplay.h
  SpinLock.cpp
  SpinLock.h
  VectorHash.h
)

set(PUBLIC_SKA_HEADERS
  ska/bytell_hash_map.hpp
  ska/flat_hash_map.hpp
  ska/ska_sort.hpp
  ska/unordered_map.hpp
)

set(PUBLIC_HEADERS
  Colour.h
  cxxopts.hpp
  GlmStream.h
  LineWalk.h
  OhmUtil.h
  Options.h
  p2p.h
  PlyMesh.h
  PointsDoubleBuffer.h
  PointsDoubleBuffer.inl
  PointsRingBuffer.h
  PointsRingBuffer.inl
  Profile.h
  ProfileMarker.h
  ProgressMonitor.h
  SafeIO.h
  ScopedTimeDisplay.h
  SpinLock.h
  VectorHash.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmutil/OhmUtilExport.h"
)

add_library(ohmutil ${SOURCES})

generate_export_header(ohmutil
      EXPORT_MACRO_NAME ohmutil_API
      EXPORT_FILE_NAME ohmutil/OhmUtilExport.h
      STATIC_DEFINE ohmutil_STATIC)

target_include_directories(ohmutil
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmutil>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmutil SYSTEM
  PUBLIC "${GLM_INCLUDE_DIR}"
)

# Kazys: Why did this suddenly become necessary on my Linux desktop?
if(TARGET Threads::Threads)
  target_link_libraries(ohmutil Threads::Threads)
endif()

install(TARGETS ohmutil EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmutil
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmutil)
install(FILES ${PUBLIC_SKA_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmutil/ska)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
