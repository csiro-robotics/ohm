
include(GenerateExportHeader)
include(TextFileResource)

find_package(GLM)
find_package(ZLIB)
if(OHM_THREADS)
  find_package(TBB)
endif(OHM_THREADS)

set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(OhmConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmConfig.h")

set(SOURCES
  private/ClearanceProcessDetail.cpp
  private/ClearanceProcessDetail.h
  private/HeightmapDetail.cpp
  private/HeightmapDetail.h
  private/LineQueryDetail.h
  private/MapLayerDetail.h
  private/MapLayoutDetail.h
  private/NearestNeighboursDetail.h
  private/VoxelAlgorithms.cpp
  private/VoxelAlgorithms.h
  private/OccupancyMapDetail.cpp
  private/OccupancyMapDetail.h
  private/QueryDetail.h
  private/RoiRangeFill.cpp
  private/RoiRangeFill.h
  private/VoxelLayoutDetail.h
  Aabb.h
  ClearanceProcess.cpp
  ClearanceProcess.h
  DataType.cpp
  DataType.h
  DefaultLayer.cpp
  DefaultLayer.h
  Heightmap.cpp
  Heightmap.h
  HeightmapVoxel.cpp
  HeightmapVoxel.h
  Key.cpp
  Key.h
  KeyList.cpp
  KeyList.h
  LineKeysQuery.cpp
  LineKeysQuery.h
  LineQuery.cpp
  LineQuery.h
  MapCache.cpp
  MapCache.h
  MapChunk.cpp
  MapChunk.h
  MapCoord.h
  MapInfo.cpp
  MapInfo.h
  MapLayer.cpp
  MapLayer.h
  MapLayout.cpp
  MapLayout.h
  Mapper.cpp
  Mapper.h
  MappingProcess.cpp
  MappingProcess.h
  MapProbability.h
  MapRegion.cpp
  MapRegion.h
  MapSerialise.cpp
  MapSerialise.h
  NearestNeighbours.cpp
  NearestNeighbours.h
  OccupancyMap.cpp
  OccupancyMap.h
  OccupancyType.cpp
  OccupancyType.h
  OhmGpu.cpp
  OhmGpu.h
  Query.cpp
  Query.h
  QueryFlag.h
  RayFilter.cpp
  RayFilter.h
  Stream.cpp
  Stream.h
  Voxel.cpp
  Voxel.h
  VoxelLayout.cpp
  VoxelLayout.h
)

if(TES_ENABLE)
  # Only required to define the 3es server symbol.
  list(APPEND SOURCES OccupancyUtil.cpp)
endif(TES_ENABLE)

set(PUBLIC_HEADERS
  Aabb.h
  ClearanceProcess.h
  DataType.h
  DefaultLayer.h
  Heightmap.h
  HeightmapVoxel.h
  Key.h
  KeyList.h
  LineKeysQuery.h
  LineQuery.h
  MapCache.h
  MapChunk.h
  MapChunkFlag.h
  MapCoord.h
  MapInfo.h
  MapLayer.h
  MapLayout.h
  Mapper.h
  MappingProcess.h
  MapProbability.h
  MapRegion.h
  MapSerialise.h
  NearestNeighbours.h
  OccupancyMap.h
  OccupancyType.h
  OccupancyUtil.h
  OhmGpu.h
  Query.h
  QueryFlag.h
  RayFilter.h
  Stream.h
  Voxel.h
  VoxelLayout.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmExport.h"
  )

if(OHM_GPU)
  # Add gputil:gpu_ext.h to this project for correct marshalling.
  # Always re-resolve gpu_ext.h in case it moves.
  unset(GPU_EXT_H CACHE)
  find_file(GPU_EXT_H gpu_ext.h PATHS ${GPUTIL_DEVICE_INCLUDE_DIRS})

  set(GPU_SOURCES
    gpu/LineKeys.cl
    gpu/NearestNeighboursQuery.cl
    gpu/RegionUpdate.cl
    gpu/RoiRangeFill.cl
    gpu/TransformSamples.cl
  )
  if(NOT OHM_EMBED_GPU_CODE)
    list(APPEND GPU_SOURCES gpu/LineWalk.cl)
    list(APPEND GPU_SOURCES gpu/Regions.cl)
  endif(NOT OHM_EMBED_GPU_CODE)

  set(GPU_HEADERS
    GpuKey.h
    MapCoord.h
  )

  list(APPEND SOURCES
    private/GpuMapDetail.cpp
    private/GpuMapDetail.h
    private/GpuTransformSamplesDetail.h
    GpuCache.cpp
    GpuCache.h
    GpuCacheParams.h
    GpuLayerCache.cpp
    GpuLayerCache.h
    GpuMap.cpp
    GpuMap.h
    GpuTransformSamples.cpp
    GpuTransformSamples.h
  )

  list(APPEND PUBLIC_HEADERS
    GpuCache.h
    GpuCacheParams.h
    GpuKey.h
    GpuLayerCache.h
    GpuMap.h
    GpuTransformSamples.h
  )
endif(OHM_GPU)

set(USE_CUDA FALSE)
if(OHM_GPU)
  list(APPEND SOURCES ${GPU_SOURCES} ${GPU_HEADERS} "${GPU_EXT_H}")
  if(OHM_GPU EQUAL OHM_GPU_OPENCL)
    get_target_property(CLU_INCLUDE_DIRS clu INCLUDE_DIRECTORIES)
    list(APPEND SOURCES
      cl/clProgram.cpp
      cl/clProgram.h
      cl/NearestNeighboursQuery.cpp
      cl/RoiRangeFill.cpp
    )

    if(OHM_EMBED_GPU_CODE)
      # Embedding GPU code. Generate source file resources.
      # Only need to embed for OpenCL.
      foreach(CLSOURCE ${GPU_SOURCES})
        get_filename_component(CLSOURCENAME "${CLSOURCE}" NAME)
        get_filename_component(CLSOURCENAME_WE "${CLSOURCE}" NAME_WE)
        text_file_resource("${CLSOURCE}" "${CLSOURCENAME_WE}Code"
          TYPE opencl
          PATHS "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/gpu" "${GPUTIL_DEVICE_INCLUDE_DIRS}"
          # Append to the SOURCES list.
          FILELIST SOURCES
        )
      endforeach(CLSOURCE)
    endif(OHM_EMBED_GPU_CODE)
  else()
    # CUDA
    set(USE_CUDA TRUE)
    find_package(CUDA)
    list(APPEND SOURCES
      cuda/NearestNeighboursQuery.cu
      cuda/RoiRangeFill.cu
    )
  endif(OHM_GPU EQUAL OHM_GPU_OPENCL)
endif(OHM_GPU)

if(USE_CUDA)
  cuda_add_library(ohm ${SOURCES})
else(USE_CUDA)
  add_library(ohm ${SOURCES})
endif(USE_CUDA)

target_link_libraries(ohm ${ZLIB_LIBRARIES} ohmutil gputil)

if(OHM_TES_DEBUG)
  target_link_libraries(ohm 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohm PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)


if(OHM_GPU EQUAL OHM_GPU_OPENCL)
  target_link_libraries(ohm clu)

  if(NOT OHM_EMBED_GPU_CODE)
    # Not embedding GPU source code. Make sure we marshal GPU headers as well as code.
    get_filename_component(CLSOURCENAME "${GPU_EXT_H}" NAME)
    add_custom_command(
      OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${GPU_EXT_H}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
      MAIN_DEPENDENCY "${GPU_EXT_H}"
      COMMENT "Copying ${CLSOURCENAME}"
    )

    list(APPEND GPU_SOURCES "${GPU_HEADERS}")
    foreach(CLSOURCE ${GPU_SOURCES})
      get_filename_component(CLSOURCENAME "${CLSOURCE}" NAME)
      # Not embedding GPU code. Marshal GPU source files.
      add_custom_command(
        OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_LIST_DIR}/${CLSOURCE}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
        MAIN_DEPENDENCY "${CMAKE_CURRENT_LIST_DIR}/${CLSOURCE}"
        COMMENT "Copying ${CLSOURCE}"
      )
    endforeach(CLSOURCE)
  endif(NOT OHM_EMBED_GPU_CODE)
endif(OHM_GPU EQUAL OHM_GPU_OPENCL)

generate_export_header(ohm
      EXPORT_MACRO_NAME ohm_API
      EXPORT_FILE_NAME ohm/OhmExport.h
      STATIC_DEFINE OHM_STATIC)

target_include_directories(ohm
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohm>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_include_directories(ohm SYSTEM
  PUBLIC "${GLM_INCLUDE_DIR}"
  PRIVATE "${ZLIB_INCLUDE_DIR}"
)

if(OHM_THREADS)
  target_link_libraries(ohm ${TBB_LIBRARIES})
endif(OHM_THREADS)


install(TARGETS ohm EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohm
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohm)

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\cl" REGULAR_EXPRESSION "/cl/.*$")
source_group("source\\gpu" REGULAR_EXPRESSION "/gpu/.*$")
source_group("source\\private" REGULAR_EXPRESSION "/private/.*$")
