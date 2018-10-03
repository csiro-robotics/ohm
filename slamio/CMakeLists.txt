
include(GenerateExportHeader)

find_package(GLM)

find_package(LIBLAS)
find_package(LASZIP QUIET)

if(NOT LIBLAS_FOUND)
  message(STATUS "Skipping slamio. libLAS not found.")
endif(NOT LIBLAS_FOUND)

# Need to add boost link directories because of LIBLAS.
find_package(Boost COMPONENTS "thread")

set(SOURCES
  SlamCloudLoader.cpp
  SlamCloudLoader.h
)

set(PUBLIC_HEADERS
  SlamCloudLoader.h
  "${CMAKE_CURRENT_BINARY_DIR}/slamio/SlamIOExport.h"
)

add_library(slamio ${SOURCES})

generate_export_header(slamio
      EXPORT_MACRO_NAME slamio_API
      EXPORT_FILE_NAME slamio/SlamIOExport.h
      STATIC_DEFINE slamio_STATIC)

target_include_directories(slamio
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/slamio>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(slamio SYSTEM
  PUBLIC "${GLM_INCLUDE_DIR}"
  PRIVATE "${LIBLAS_INCLUDE_DIR}"
)

# Not the best CMake use, but it avoids some Boost link issues.
link_directories("${Boost_LIBRARY_DIRS}")
target_link_libraries(slamio ohmutil
  ${LIBLAS_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(LASZIP_FOUND)
  target_include_directories(slamio SYSTEM PRIVATE "${LASZIP_INCLUDE_DIR}")
  target_link_libraries(slamio ${LASZIP_LIBRARIES})
endif(LASZIP_FOUND)

install(TARGETS slamio EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/slamio
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/slamio)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
