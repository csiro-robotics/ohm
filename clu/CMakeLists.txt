

# Convert version string to a list of versions.
string(REPLACE "." ";" OPENCL_VERSION_LIST ${OHM_OPENCL_VER})

set(OPENCL_VERSION_MINOR 0)
set(OPENCL_VERSION_PATCH 0)
list(LENGTH OPENCL_VERSION_LIST VERSION_COMPONENTS)
list(GET OPENCL_VERSION_LIST 0 OPENCL_VERSION_MAJOR)
if(VERSION_COMPONENTS GREATER 1)
  list(GET OPENCL_VERSION_LIST 1 OPENCL_VERSION_MINOR)
  if(VERSION_COMPONENTS GREATER 2)
    list(GET OPENCL_VERSION_LIST 2 OPENCL_VERSION_PATCH)
  endif(VERSION_COMPONENTS GREATER 2)
endif(VERSION_COMPONENTS GREATER 1)

# Convert version into an integer for use in code. Each version component is a digit.
set(CLU_TARGET_OPENCL_VERSION ${OPENCL_VERSION_MAJOR}${OPENCL_VERSION_MINOR}${OPENCL_VERSION_PATCH})

find_package(OpenCL ${OHM_OPENCL_VER})

configure_file(cluConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/clu/cluConfig.h")

set(SOURCES
  cl2.hpp
  clu.cpp
  clu.h
  cluBuffer.h
  cluConstraint.cpp
  cluConstraint.h
  cluKernel.cpp
  cluKernel.h
  cluProgram.cpp
  cluProgram.h
)

set(PUBLIC_HEADERS
  cl2.hpp
  clu.h
  cluBuffer.h
  cluConstraint.h
  cluKernel.h
  cluProgram.h
  "${CMAKE_CURRENT_BINARY_DIR}/clu/cluConfig.h"
)

add_library(clu STATIC ${SOURCES} ${PUBLIC_HEADERS})
target_include_directories(clu
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/clu>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(clu SYSTEM
  PUBLIC
    "${OpenCL_INCLUDE_DIRS}"
)
# set_property(TARGET clu PROPERTY DEBUG_POSTFIX "d")

install(TARGETS clu EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/clu
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/clu)

source_group("source" REGULAR_EXPRESSION ".*$")
