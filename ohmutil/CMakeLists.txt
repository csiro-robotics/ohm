
include(GenerateExportHeader)

find_package(GLM)

find_package(Threads)

set(SOURCES
  colour.cpp
  colour.h
  cxxopts.hpp
  p2p.h
  # ohmutil.cpp
  ohmutil.h
  plymesh.cpp
  plymesh.h
  pointsdoublebuffer.h
  pointsdoublebuffer.inl
  pointsringbuffer.h
  pointsringbuffer.inl
  progressmonitor.cpp
  progressmonitor.h
  glmstream.h
  linewalk.h
  options.h
  profile.cpp
  profile.h
  profilemarker.cpp
  profilemarker.h
  safeio.cpp
  safeio.h
  scopedtimedisplay.cpp
  scopedtimedisplay.h
  spinlock.cpp
  spinlock.h
  vectorhash.h
)

set(PUBLIC_HEADERS
  colour.h
  cxxopts.hpp
  p2p.h
  ohmutil.h
  plymesh.h
  pointsdoublebuffer.h
  pointsdoublebuffer.inl
  pointsringbuffer.h
  pointsringbuffer.inl
  progressmonitor.h
  glmstream.h
  linewalk.h
  options.h
  profile.h
  profilemarker.h
  safeio.h
  scopedtimedisplay.h
  spinlock.h
  vectorhash.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmutil/ohmutilexport.h"
)

add_library(ohmutil ${SOURCES})

generate_export_header(ohmutil
      EXPORT_MACRO_NAME ohmutil_API
      EXPORT_FILE_NAME ohmutil/ohmutilexport.h
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
