if(NOT LIBLAS_FOUND)
  message(STATUS "Skipping ohmpop. Missing libLAS")
  return()
endif()

# Need to add boost link directories because of LIBLAS.
link_directories("${Boost_LIBRARY_DIRS}")

include_directories(
  "${LIBLAS_INCLUDE_DIR}"
  "${GLM_INCLUDE_DIR}"
  )

configure_file(OhmPopConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OhmPopConfig.h")

set(SOURCES
  ohmpop.cpp
  OhmPopConfig.in.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OhmPopConfig.h"
)

add_executable(ohmpop ${SOURCES})

set_target_properties(ohmpop PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmpop PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmpop
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmpop>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmpop SYSTEM
  PRIVATE
    "${LIBLAS_INCLUDE_DIR}"
    "${GLM_INCLUDE_DIR}"
)

target_link_libraries(ohmpop ohm slamio ohmutil ${LIBLAS_LIBRARIES})

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmpop DESTINATION bin)
