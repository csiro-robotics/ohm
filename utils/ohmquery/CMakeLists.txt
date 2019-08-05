
set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(OhmQueryConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmquery/OhmQueryConfig.h")
set(SOURCES "${CMAKE_CURRENT_BINARY_DIR}/ohmquery/OhmQueryConfig.h")

set(SOURCES
  ohmquery.cpp
  OhmQueryConfig.in.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmquery/OhmQueryConfig.h"
)

function(_ohmquery_setup GPU_MODE)
  set(TARGET_NAME ohmquery${GPU_MODE})
  add_executable(${TARGET_NAME} ${SOURCES})
  leak_track_target_enable(${TARGET_NAME} CONDITION OHM_LEAK_TRACK)

  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER utils)
  if(MSVC)
    set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "d")
  endif(MSVC)

  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmquery>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  )

  target_include_directories(${TARGET_NAME} SYSTEM
    PRIVATE
      "${GLM_INCLUDE_DIR}"
  )

  target_link_libraries(${TARGET_NAME} PUBLIC ohm${GPU_MODE} ohmutil)

  if(OHM_TES_DEBUG)
    target_link_libraries(${TARGET_NAME} PUBLIC 3es::3es-core)
  else(OHM_TES_DEBUG)
    target_include_directories(${TARGET_NAME} PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
  endif(OHM_TES_DEBUG)

  install(TARGETS ${TARGET_NAME} DESTINATION bin)
endfunction(_ohmquery_setup)

if(OHM_BUILD_OPENCL)
  _ohmquery_setup(ocl)
  clang_tidy_target(ohmqueryocl)
  # Required to run NVIDIA OpenCL
  leak_track_default_options(ohmqueryocl CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmqueryocl CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_OCL}
    ${OHM_LEAK_SUPPRESS_TBB}
  )
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA)
  _ohmquery_setup(cuda)
  clang_tidy_target(ohmquerycuda)
  leak_track_default_options(ohmquerycuda CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmquerycuda CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_CUDA}
    ${OHM_LEAK_SUPPRESS_TBB}
  )
endif(OHM_BUILD_CUDA)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
