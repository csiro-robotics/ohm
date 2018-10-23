
include(GenerateExportHeader)

find_package(PDAL QUIET)
find_package(LIBLAS QUIET)
find_package(LASZIP QUIET)

set(OHM_CLOUD_IO_LIBRARY auto CACHE STRING "Select point cloud IO library to use:  auto (prefer pdal), liblas, pdal or none")
set_property(CACHE OHM_CLOUD_IO_LIBRARY PROPERTY STRINGS auto liblas pdal none)

message(STATUS "Cloud IO Library: ${OHM_CLOUD_IO_LIBRARY}")

if(OHM_CLOUD_IO_LIBRARY STREQUAL "none")
  return()
endif(OHM_CLOUD_IO_LIBRARY STREQUAL "none")

# Select the cloud io library to use.
set(CLOUD_IO_LIB ${OHM_CLOUD_IO_LIBRARY})
if(CLOUD_IO_LIB STREQUAL "auto")
  if(PDAL_FOUND)
    set(CLOUD_IO_LIB "pdal")
  elseif(LIBLAS_FOUND)
    set(CLOUD_IO_LIB "liblas")
  else()
    message(SEND_ERROR "Unable to local a point cloud IO library [pdal, liblas].")
    return()
  endif()
  message(STATUS "Cloud IO auto selected: ${CLOUD_IO_LIB}")
endif(CLOUD_IO_LIB STREQUAL "auto")

find_package(GLM)

set(SOURCES
  SlamCloudLoader.h
)

set(PUBLIC_HEADERS
  SlamCloudLoader.h
  "${CMAKE_CURRENT_BINARY_DIR}/slamio/SlamIOExport.h"
)

# Need to add boost link directories because of LIBLAS.
if(CLOUD_IO_LIB STREQUAL "liblas")
  find_package(Boost COMPONENTS "thread")
  list(APPEND SOURCES
    SlamCloudLoader_las.cpp
  )
endif(CLOUD_IO_LIB STREQUAL "liblas")

if(CLOUD_IO_LIB STREQUAL "pdal")
  list(APPEND SOURCES
    SlamCloudLoader_pdal.cpp
  )
endif(CLOUD_IO_LIB STREQUAL "pdal")

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
)

target_link_libraries(slamio ohmutil)

if(CLOUD_IO_LIB STREQUAL "pdal")
  target_link_libraries(slamio ${PDAL_LIBRARIES})
endif(CLOUD_IO_LIB STREQUAL "pdal")

if(CLOUD_IO_LIB STREQUAL "liblas")
  target_include_directories(slamio SYSTEM
    PRIVATE "${LIBLAS_INCLUDE_DIR}"
  )
  # Not the best CMake use, but it avoids some Boost link issues.
  link_directories("${Boost_LIBRARY_DIRS}")
  target_link_libraries(slamio
    ${LIBLAS_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  if(LASZIP_FOUND)
    target_include_directories(slamio SYSTEM PRIVATE "${LASZIP_INCLUDE_DIR}")
    target_link_libraries(slamio ${LASZIP_LIBRARIES})
  endif(LASZIP_FOUND)
endif(CLOUD_IO_LIB STREQUAL "liblas")

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
