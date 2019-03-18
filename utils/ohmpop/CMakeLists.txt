if(NOT TARGET slamio)
  message(STATUS "Skipping ohmpop. slamio not built")
  return()
endif()

if (Boost_FOUND)
  # Need to add boost link directories because of LIBLAS.
  link_directories("${Boost_LIBRARY_DIRS}")
endif(Boost_FOUND)

# include_directories(
#   "${LIBLAS_INCLUDE_DIR}"
#   "${GLM_INCLUDE_DIR}"
#   )

configure_file(OhmPopConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OhmPopConfig.h")

set(SOURCES
  ohmpop.cpp
  OhmPopConfig.in.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OhmPopConfig.h"
)

function(_ohmpop_setup GPU_MODE)
  set(TARGET_NAME ohmpop${GPU_MODE})
  add_executable(${TARGET_NAME} ${SOURCES})

  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER utils)
  if(MSVC)
    set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "d")
  endif(MSVC)

  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmpop>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  )

  target_include_directories(${TARGET_NAME} SYSTEM
    PRIVATE
      "${GLM_INCLUDE_DIR}"
  )

  target_link_libraries(${TARGET_NAME} ohm${GPU_MODE} slamio ohmutil)

  install(TARGETS ${TARGET_NAME} DESTINATION bin)
endfunction(_ohmpop_setup)

if(OHM_BUILD_OPENCL)
  _ohmpop_setup(ocl)
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA)
  _ohmpop_setup(cuda)
endif(OHM_BUILD_CUDA)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
