
# Helper for building an application around ohm map generation.

include(GenerateExportHeader)

configure_file(OhmAppConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmapp/OhmAppConfig.h")
configure_file(OhmAppGpuConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmapp/OhmAppGpuConfig.h")

set(SOURCES
  DataSource.cpp
  DataSource.h
  MapHarness.cpp
  MapHarness.h
  OhmAppConfig.in.h
  OhmAppCpu.cpp
  OhmAppCpu.h
  ohmappmain.inl
  SlamIOSource.cpp
  SlamIOSource.h
)

set(SOURCES_GPU
  OhmAppGpu.cpp
  OhmAppGpu.h
)

set(PUBLIC_HEADERS
  DataSource.h
  MapHarness.h
  OhmAppCpu.h
  ohmappmain.inl
  SlamIOSource.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmapp/OhmAppConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmapp/OhmAppExport.h"
)

set(PUBLIC_HEADERS_GPU
  OhmAppGpu.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmapp/OhmAppGpuConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmapp/OhmAppGpuExport.h"
)

function(_ohmapp_setup_target TARGET_NAME OHM_GPU)
  # Because of the way we compile our GPU library twice with different names, we must explicitly define the export
  # macro. Curiously, there's a way to overide all the macros except the one used to control whether to export the
  # symbols or not. This puts us in a position where it could either be ohmcuda_EXPORTS or ohmocl_EXPORTS depending
  # on which targets are enabled. We build both the same way though, so define both symbols for all builds.
  if(OHM_GPU STREQUAL "cpu")
    target_compile_definitions(${TARGET_NAME} PRIVATE "-Dohmapp_EXPORTS")
    set(OHMGPU_LIBRARY)
    set(EXPORT_HEADER "OhmAppExport.h")
    set(EXPORT_PREFIX "ohmapp")
    set(EXPORT_STATIC "OHMAPP")
  else(OHM_GPU STREQUAL "cpu")
    # target_compile_definitions(${TARGET_NAME} PUBLIC "-DOHM_GPU=${OHM_GPU}")
    target_compile_definitions(${TARGET_NAME} PRIVATE "-Dohmapp_EXPORTS" "-Dohmcuda_EXPORTS" "-Dohmocl_EXPORTS")
    set(OHMGPU_LIBRARY ohm${OHM_GPU})
    set(EXPORT_HEADER "OhmAppGpuExport.h")
    set(EXPORT_PREFIX "ohmappgpu")
    set(EXPORT_STATIC "OHMAPPGPU")
  endif(OHM_GPU STREQUAL "cpu")
  target_link_libraries(${TARGET_NAME} PUBLIC ${OHMGPU_LIBRARY})

  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmapp>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  )

  clang_tidy_target(${TARGET_NAME})

  generate_export_header(${TARGET_NAME}
    EXPORT_MACRO_NAME "${EXPORT_PREFIX}_API"
    DEPRECATED_MACRO_NAME "${EXPORT_PREFIX}_DEPRECATED"
    NO_EXPORT_MACRO_NAME "${EXPORT_PREFIX}_NO_EXPORT"
    EXPORT_FILE_NAME "ohmapp/${EXPORT_HEADER}"
    NO_DEPRECATED_MACRO_NAME  "${EXPORT_PREFIX}_NO_DEPRECATED"
    STATIC_DEFINE "${EXPORT_STATIC}_STATIC")

  install(TARGETS ${TARGET_NAME} EXPORT ${CMAKE_PROJECT_NAME}-config-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmapp
  )
endfunction(_ohmapp_setup_target)

add_library(ohmapp ${SOURCES})
_ohmapp_setup_target(ohmapp "cpu")
target_link_libraries(ohmapp PUBLIC ohm ohmutil ohmtools slamio)

# Add GPU sources and libarries
list(APPEND SOURCE OhmAppGpu.cpp OhmAppGpu.h)
if(OHM_BUILD_CUDA)
  add_library(ohmappcuda ${SOURCES_GPU})
  _ohmapp_setup_target(ohmappcuda "cuda")
  target_link_libraries(ohmappcuda PUBLIC ohmapp)
endif(OHM_BUILD_CUDA)
if(OHM_BUILD_OPENCL)
  add_library(ohmappocl ${SOURCES_GPU})
  _ohmapp_setup_target(ohmappocl "ocl")
  target_link_libraries(ohmappocl PUBLIC ohmapp)
endif(OHM_BUILD_OPENCL)

install(FILES ${PUBLIC_HEADERS} ${PUBLIC_HEADERS_GPU} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmapp)

source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
