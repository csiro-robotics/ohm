if(NOT TARGET slamio)
  message(STATUS "Skipping ohmpop. slamio not built")
  return()
endif()

if(OHM_BUILD_CUDA AND OHM_USE_DEPRECATED_CMAKE_CUDA)
  # Need to find CUDA for deprecated project configuration
  find_package(CUDA)
endif(OHM_BUILD_CUDA AND OHM_USE_DEPRECATED_CMAKE_CUDA)

configure_file(OhmPopConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OhmPopConfig.h")

set(SOURCES
  ohmpop.cpp
  OhmPopConfig.in.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OhmPopConfig.h"
)

function(_ohmpop_setup GPU_MODE)
  set(TARGET_NAME ohmpop${GPU_MODE})
  if(NOT GPU_MODE STREQUAL "cpu")
    set(OHMLIB_NAME ohm${GPU_MODE})
    set(GPUTILLIB_NAME gputil${GPU_MODE})
  else(NOT GPU_MODE STREQUAL "cpu")
    set(OHMLIB_NAME ohm)
    set(GPUTILLIB_NAME)
  endif(NOT GPU_MODE STREQUAL "cpu")

  set(TARGET_NAME ohmpop${GPU_MODE})

  if(GPU_MODE STREQUAL "cuda" AND OHM_USE_DEPRECATED_CMAKE_CUDA)
    cuda_add_executable(${TARGET_NAME} ${SOURCES})
  else()
    add_executable(${TARGET_NAME} ${SOURCES})
  endif()
  leak_track_target_enable(${TARGET_NAME} CONDITION OHM_LEAK_TRACK)

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

  target_link_libraries(${TARGET_NAME} PRIVATE ${OHMLIB_NAME} slamio ohmtools ohmutil ${GPUTILLIB_NAME})

  if(GPU_MODE STREQUAL "cpu")
    target_compile_definitions(${TARGET_NAME} PUBLIC "-DOHMPOP_CPU")
  else()
    target_compile_definitions(${TARGET_NAME} PUBLIC "-DOHMPOP_GPU")
  endif(GPU_MODE STREQUAL "cpu")

  install(TARGETS ${TARGET_NAME} DESTINATION bin)
endfunction(_ohmpop_setup)

if(OHM_BUILD_OPENCL)
  _ohmpop_setup(ocl)
  clang_tidy_target(ohmpopocl)
  # Required to run NVIDIA OpenCL
  leak_track_default_options(ohmpopocl CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmpopocl CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_OCL}
    "libpdal_base"
  )
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA)
  _ohmpop_setup(cuda)
  clang_tidy_target(ohmpopcuda)
  leak_track_default_options(ohmpopcuda CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmpopcuda CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_CUDA}
    "libpdal_base"
  )
endif(OHM_BUILD_CUDA)
_ohmpop_setup(cpu)
clang_tidy_target(ohmpopcpu)
leak_track_suppress(ohmpopcpu CONDITION OHM_LEAK_TRACK
  "libpdal_base"
)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
