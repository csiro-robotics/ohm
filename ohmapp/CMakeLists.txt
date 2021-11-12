
# Helper for building an application around ohm map generation.

set(SOURCES
  DataSource.cpp
  DataSource.h
  MapHarness.cpp
  MapHarness.h
  ohmpopmain.inl
  SlamIOSource.cpp
  SlamIOSource.h
)

set(PUBLIC_HEADERS
  DataSource.h
  MapHarness.h
  ohmpopmain.inl
  SlamIOSource.h
)

add_library(ohmapp STATIC ${SOURCES})
clang_tidy_target(ohmapp)

target_link_libraries(ohmapp PUBLIC ohm ohmutil slamio)

target_include_directories(ohmapp
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmapp>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

install(TARGETS ohmapp EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmapp
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmapp)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
