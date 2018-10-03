
include(GenerateExportHeader)

find_package(GLM)

find_package(Threads)

set(SOURCES
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

set(PUBLIC_HEADERS
  Colour.h
  cxxopts.hpp
  p2p.h
  OhmUtil.h
  Plymesh.h
  PointsDoubleBuffer.h
  PointsDoubleBuffer.inl
  PointsRingBuffer.h
  PointsRingBuffer.inl
  ProgressMonitor.h
  GlmStream.h
  LineWalk.h
  Options.h
  Profile.h
  ProfileMarker.h
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

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
