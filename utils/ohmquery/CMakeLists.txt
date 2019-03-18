
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

  target_link_libraries(${TARGET_NAME} ohm${GPU_MODE} ohmutil)

  if(OHM_TES_DEBUG)
    target_link_libraries(${TARGET_NAME} 3es::3es-core)
  else(OHM_TES_DEBUG)
    target_include_directories(${TARGET_NAME} PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
  endif(OHM_TES_DEBUG)

  install(TARGETS ${TARGET_NAME} DESTINATION bin)
endfunction(_ohmquery_setup)

if(OHM_BUILD_OPENCL)
  _ohmquery_setup(ocl)
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA)
  _ohmquery_setup(cuda)
endif(OHM_BUILD_CUDA)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
