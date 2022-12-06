
include(GenerateExportHeader)
include(TextFileResource)


find_package(ZLIB)
if(OHM_FEATURE_THREADS)
  find_package(TBB)
endif(OHM_FEATURE_THREADS)

if(OHM_FEATURE_CUDA AND OHM_USE_DEPRECATED_CMAKE_CUDA)
  # Need to find CUDA for deprecated project configuration
  find_package(CUDA)
endif(OHM_FEATURE_CUDA AND OHM_USE_DEPRECATED_CMAKE_CUDA)

set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(OhmGpuConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmgpu/OhmGpuConfig.h")

set(SOURCES
  private/ClearanceProcessDetail.cpp
  private/ClearanceProcessDetail.h
  private/GpuMapDetail.cpp
  private/GpuMapDetail.h
  private/GpuProgramRef.cpp
  private/GpuProgramRef.h
  private/GpuTransformSamplesDetail.h
  private/GpuTsdfMapDetail.h
  private/LineKeysQueryDetailGpu.h
  private/LineQueryDetailGpu.h
  private/RaysQueryDetailGpu.cpp
  private/RaysQueryDetailGpu.h
  private/RoiRangeFill.cpp
  private/RoiRangeFill.h
  ClearanceProcess.cpp
  ClearanceProcess.h
  GpuCache.cpp
  GpuCache.h
  GpuCachePostSyncHandler.h
  GpuCacheStats.h
  GpuKey.h
  GpuLayerCache.cpp
  GpuLayerCache.h
  GpuLayerCacheParams.cpp
  GpuLayerCacheParams.h
  GpuMap.cpp
  GpuMap.h
  GpuNdtMap.cpp
  GpuNdtMap.h
  GpuTransformSamples.cpp
  GpuTransformSamples.h
  GpuTsdfMap.cpp
  GpuTsdfMap.h
  LineKeysQueryGpu.cpp
  LineKeysQueryGpu.h
  LineQueryGpu.cpp
  LineQueryGpu.h
  OhmGpu.cpp
  OhmGpu.h
  RaysQueryGpu.cpp
  RaysQueryGpu.h
)

set(GPU_SOURCES
  gpu/CovarianceHitNdt.cl
  gpu/LineKeys.cl
  gpu/RaysQuery.cl
  gpu/RegionUpdate.cl
  gpu/RoiRangeFill.cl
  gpu/TransformSamples.cl
  gpu/TsdfUpdate.cl
)

get_target_property(OHM_SOURCE_DIR ohm SOURCE_DIR)
set(GPU_HEADERS
  gpu/AdjustNdt.cl
  gpu/AdjustOccupancy.cl
  gpu/CovarianceHitNdt_h.cl
  gpu/Traversal.cl
  gpu/LineWalk.cl
  gpu/LineWalkMarkers.cl
  gpu/VoxelIncident.cl
  gpu/VoxelMean.cl
  gpu/RaysQueryResult.h
  GpuKey.h
  # Need some headers from the OHM core project.
  ${OHM_SOURCE_DIR}/CovarianceVoxelCompute.h
  ${OHM_SOURCE_DIR}/LineWalkCompute.h
  ${OHM_SOURCE_DIR}/MapCoord.h
  ${OHM_SOURCE_DIR}/RayFlag.h
  ${OHM_SOURCE_DIR}/VoxelIncidentCompute.h
  ${OHM_SOURCE_DIR}/VoxelMeanCompute.h
  ${OHM_SOURCE_DIR}/VoxelTouchTimeCompute.h
  ${OHM_SOURCE_DIR}/VoxelTsdfCompute.h
)

set(PUBLIC_HEADERS
  ClearanceProcess.h
  GpuCache.h
  GpuLayerCacheParams.h
  GpuCachePostSyncHandler.h
  GpuCacheStats.h
  GpuKey.h
  GpuLayerCache.h
  GpuMap.h
  GpuNdtMap.h
  GpuTransformSamples.h
  LineKeysQueryGpu.h
  LineQueryGpu.h
  OhmGpu.h
  RaysQueryGpu.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmgpu/OhmGpuConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmgpu/OhmGpuExport.h"
  )

function(_ohmgpu_setup_target TARGET_NAME GPUTIL_LIBRARY OHM_GPU)
  target_compile_definitions(${TARGET_NAME} PUBLIC "-DOHM_GPU=${OHM_GPU}")
  # Because of the way we compile our GPU library twice with different names, we must explicitly define the export
  # macro. Curiously, there's a way to overide all the macros except the one used to control whether to export the
  # symbols or not. This puts us in a position where it could either be ohmcuda_EXPORTS or ohmocl_EXPORTS depending
  # on which targets are enabled. We build both the same way though, so define both symbols for all builds.
  target_compile_definitions(${TARGET_NAME} PRIVATE "-Dohmcuda_EXPORTS" "-Dohmocl_EXPORTS")
  target_link_libraries(${TARGET_NAME}
    PUBLIC
      ohm
      ohmutil
      $<$<BOOL:${OHM_FEATURE_THREADS}>:TBB::tbb>
    PRIVATE
      ${GPUTIL_LIBRARY}
  )

    if(BUILD_SHARED)
      # Link additional dependencies when building shared ohm.
      # Because ohm is shared and these are private, static depenencies, we do not need to propagate
      # the dependencies and limit this with $<BUILD_INTERFACE:>
      target_link_libraries(${TARGET_NAME}
        PRIVATE
          # Link 3es if enabled.
          $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
      )
    else(BUILD_SHARED)
      # With ohm static, we link depdencencies in such as way that the will propagate and need to be
      # found when ohm is linked.
      target_link_libraries(${TARGET_NAME}
        PRIVATE
          # Link 3es if enabled.
          $<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>
      )
    endif(BUILD_SHARED)
  
  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
      # Access private ohm interfaces
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmgpu>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
      # The following include directories are required to get the GPU code building in all different context.
      # Otherwise we have a lot of book keeping in managing include statements.
      $<BUILD_INTERFACE:${OHM_SOURCE_DIR}>
      $<BUILD_INTERFACE:${OHM_SOURCE_DIR}/private>
    )

  target_include_directories(${TARGET_NAME}
    SYSTEM PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/3rdparty"
      $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
    )

  clang_tidy_target(${TARGET_NAME} EXCLUDE_MATCHES ".*\.cl")

  generate_export_header(${TARGET_NAME}
    EXPORT_MACRO_NAME ohmgpu_API
    DEPRECATED_MACRO_NAME ohmgpu_DEPRECATED
    NO_EXPORT_MACRO_NAME ohmgpu_NO_EXPORT
    EXPORT_FILE_NAME ohmgpu/OhmGpuExport.h
    NO_DEPRECATED_MACRO_NAME ohmgpu_NO_DEPRECATED
    STATIC_DEFINE OHMGPU_STATIC)

  install(TARGETS ${TARGET_NAME} EXPORT ${CMAKE_PROJECT_NAME}-config-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmgpu
  )
endfunction(_ohmgpu_setup_target)


if(OHM_FEATURE_OPENCL)
  get_target_property(OHM_INCLUDE_DIRS ohm SOURCE_DIR)

  # Add gputil:gpu_ext.h to this project for correct marshalling.
  get_target_property(GPUTIL_INCLUDE_DIRS gputilocl SOURCE_DIR)
  get_target_property(GPUTIL_SOURCE_DIR gputilocl SOURCE_DIR)
  list(APPEND GPU_HEADERS ${GPUTIL_SOURCE_DIR}/gpu_ext.h)

  if(OHM_EMBED_GPU_CODE)
    # Embedding GPU code. Generate source file resources.
    # Only need to embed for OpenCL.
    foreach(CLSOURCE ${GPU_SOURCES})
      get_filename_component(CLSOURCENAME "${CLSOURCE}" NAME)
      get_filename_component(CLSOURCENAME_WE "${CLSOURCE}" NAME_WE)
      text_file_resource("${CLSOURCE}" "${CLSOURCENAME_WE}Code"
        TYPE opencl
        # ECHO
        PATHS "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/gpu" "${OHM_INCLUDE_DIRS}" "${PROJECT_SOURCE_DIR}" "${GPUTIL_INCLUDE_DIRS}"
        # Append to the SOURCES list.
        FILELIST SOURCES
      )
    endforeach(CLSOURCE)
  else(OHM_EMBED_GPU_CODE)
    list(APPEND GPU_SOURCES gpu/LineWalk.cl)
    list(APPEND GPU_SOURCES gpu/LineWalkMarkers.cl)
    list(APPEND GPU_SOURCES gpu/Regions.cl)
  endif(OHM_EMBED_GPU_CODE)

  add_library(ohmocl ${SOURCES} ${GPU_SOURCES} ${GPU_HEADERS})
  _ohmgpu_setup_target(ohmocl gputilocl ${OHM_GPU_OPENCL})

  if(NOT OHM_EMBED_GPU_CODE)
    # Not embedding GPU source code. Make sure we marshal GPU headers as well as code.
    list(APPEND GPU_SOURCES "${GPU_HEADERS}")
    foreach(CLSOURCE ${GPU_SOURCES})
      get_filename_component(CLSOURCENAME "${CLSOURCE}" NAME)
      get_filename_component(CLSOURCEPATH "${CLSOURCE}" ABSOLUTE)
      # Not embedding GPU code. Marshal GPU source files.
      add_custom_command(
        OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CLSOURCEPATH}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
        MAIN_DEPENDENCY "${CLSOURCE}"
        COMMENT "Copying ${CLSOURCE}"
      )
    endforeach(CLSOURCE)
  endif(NOT OHM_EMBED_GPU_CODE)
endif(OHM_FEATURE_OPENCL)

if(OHM_FEATURE_CUDA)
  list(APPEND GPU_SOURCES
    gpu/CovarianceHitNdt.cu
    gpu/LineKeys.cu
    gpu/RaysQuery.cu
    gpu/RegionUpdate.cu
    gpu/RegionUpdateNdt.cu
    gpu/RoiRangeFill.cu
    gpu/TransformSamples.cu
    gpu/TsdfUpdate.cu
  )

  if(OHM_USE_DEPRECATED_CMAKE_CUDA)
    cuda_add_library(ohmcuda ${SOURCES} ${GPU_SOURCES} ${GPU_HEADERS})
  else(OHM_USE_DEPRECATED_CMAKE_CUDA)
    add_library(ohmcuda ${SOURCES} ${GPU_SOURCES} ${GPU_HEADERS})
    target_link_libraries(ohmcuda PUBLIC "${OHM_CUDA_LIBRARY}")
  endif(OHM_USE_DEPRECATED_CMAKE_CUDA)

  _ohmgpu_setup_target(ohmcuda gputilcuda ${OHM_GPU_CUDA})
endif(OHM_FEATURE_CUDA)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmgpu)

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\cl" REGULAR_EXPRESSION "/cl/.*$")
source_group("source\\gpu" REGULAR_EXPRESSION "/gpu/.*$")
source_group("source\\private" REGULAR_EXPRESSION "/private/.*$")
