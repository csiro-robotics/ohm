
include(GenerateExportHeader)
include(TextFileResource)

find_package(GLM)
find_package(ZLIB)
if(OHM_THREADS)
  find_tbb()
endif(OHM_THREADS)

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
  private/LineKeysQueryDetailGpu.h
  private/LineQueryDetailGpu.h
  private/RoiRangeFill.cpp
  private/RoiRangeFill.h
  ClearanceProcess.cpp
  ClearanceProcess.h
  GpuCache.cpp
  GpuCache.h
  GpuCacheParams.h
  GpuCachePostSyncHandler.h
  GpuCacheStats.h
  GpuKey.h
  GpuLayerCache.cpp
  GpuLayerCache.h
  GpuMap.cpp
  GpuMap.h
  GpuNdtMap.cpp
  GpuNdtMap.h
  GpuTransformSamples.cpp
  GpuTransformSamples.h
  LineKeysQueryGpu.cpp
  LineKeysQueryGpu.h
  LineQueryGpu.cpp
  LineQueryGpu.h
  OhmGpu.cpp
  OhmGpu.h
)

set(GPU_SOURCES
  gpu/CovarianceHit.cl
  gpu/LineKeys.cl
  gpu/RegionUpdate.cl
  gpu/RoiRangeFill.cl
  gpu/TransformSamples.cl
)

get_target_property(OHM_SOURCE_DIR ohm SOURCE_DIR)
set(GPU_HEADERS
  gpu/AdjustNdt.cl
  gpu/AdjustOccupancy.cl
  gpu/LineWalk.cl
  gpu/VoxelMean.cl
  GpuKey.h
  # Need some headers from the OHM core project.
  ${OHM_SOURCE_DIR}/CovarianceVoxelCompute.h
  ${OHM_SOURCE_DIR}/MapCoord.h
  ${OHM_SOURCE_DIR}/RayFlag.h
  ${OHM_SOURCE_DIR}/VoxelMeanCompute.h
)

set(PUBLIC_HEADERS
  ClearanceProcess.h
  GpuCache.h
  GpuCacheParams.h
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
  "${CMAKE_CURRENT_BINARY_DIR}/ohmgpu/OhmGpuConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmgpu/OhmGpuExport.h"
  )

function(_ohmgpu_setup_target TARGET_NAME GPUTIL_LIBRARY OHM_GPU)
  target_compile_definitions(${TARGET_NAME} PUBLIC "-DOHM_GPU=${OHM_GPU}")
  target_link_libraries(${TARGET_NAME} PUBLIC ohm ohmutil ${GPUTIL_LIBRARY})

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

  if(OHM_THREADS)
    target_include_directories(${TARGET_NAME} PRIVATE SYSTEM ${TBB_INCLUDE_DIRS})
    target_link_libraries(${TARGET_NAME} PUBLIC ${TBB_LIBRARIES})
  endif(OHM_THREADS)
  if(OHM_TES_DEBUG)
    target_link_libraries(${TARGET_NAME} PUBLIC 3es::3es-core)
  else(OHM_TES_DEBUG)
    target_include_directories(${TARGET_NAME} PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
  endif(OHM_TES_DEBUG)

  clang_tidy_target(${TARGET_NAME} EXCLUDE_MATCHES ".*\.cl")

  generate_export_header(${TARGET_NAME}
    EXPORT_MACRO_NAME ohmgpu_API
    EXPORT_FILE_NAME ohmgpu/OhmGpuExport.h
    STATIC_DEFINE OHM_STATIC)

  install(TARGETS ${TARGET_NAME} EXPORT ${CMAKE_PROJECT_NAME}-config-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmgpu
  )
endfunction(_ohmgpu_setup_target)


if(OHM_BUILD_OPENCL)
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
    list(APPEND GPU_SOURCES gpu/Regions.cl)
  endif(OHM_EMBED_GPU_CODE)

  add_library(ohmocl ${SOURCES} ${GPU_SOURCES} ${GPU_HEADERS})
  _ohmgpu_setup_target(ohmocl gputilocl ${OHM_GPU_OPENCL})
  target_link_libraries(ohmocl PUBLIC clu)

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
endif(OHM_BUILD_OPENCL)

if(OHM_BUILD_CUDA)
  list(APPEND GPU_SOURCES
    gpu/CovarianceHit.cu
    gpu/LineKeys.cu
    gpu/RegionUpdate.cu
    gpu/RegionUpdateNdt.cu
    gpu/RegionUpdateWithMean.cu
    gpu/RoiRangeFill.cu
    gpu/TransformSamples.cu
  )

  cuda_add_library(ohmcuda ${SOURCES} ${GPU_SOURCES} ${GPU_HEADERS})
  _ohmgpu_setup_target(ohmcuda gputilcuda ${OHM_GPU_CUDA})
endif(OHM_BUILD_CUDA)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmgpu)

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\cl" REGULAR_EXPRESSION "/cl/.*$")
source_group("source\\gpu" REGULAR_EXPRESSION "/gpu/.*$")
source_group("source\\private" REGULAR_EXPRESSION "/private/.*$")
