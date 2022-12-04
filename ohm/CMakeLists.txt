
include(GenerateExportHeader)
include(TextFileResource)

find_package(GLM)
find_package(ZLIB)
if(OHM_FEATURE_THREADS)
  find_package(TBB)
endif(OHM_FEATURE_THREADS)

set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(OhmConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmConfig.h")

set(SOURCES
  private/ClearingPatternDetail.h
  private/LineQueryDetail.h
  private/MapLayerDetail.h
  private/MapLayoutDetail.h
  private/NdtMapDetail.h
  private/NearestNeighboursDetail.h
  private/OccupancyMapDetail.cpp
  private/OccupancyMapDetail.h
  private/QueryDetail.h
  private/RaysQueryDetail.h
  private/SerialiseUtil.h
  private/VoxelAlgorithms.cpp
  private/VoxelAlgorithms.h
  private/VoxelBlockCompressionQueueDetail.h
  private/VoxelLayoutDetail.h
  serialise/MapSerialiseV0.1.cpp
  serialise/MapSerialiseV0.1.h
  serialise/MapSerialiseV0.2.cpp
  serialise/MapSerialiseV0.2.h
  # Note: there is no ongong support for 0.3.x. That added "sub-voxel" support, but in a way which mutated the occupancy
  # layer and did not generate good results. Version 0.4.0 introduced an explicit VoxelMean layer for much better
  # results and obviated the need for map header changes.
  serialise/MapSerialiseV0.4.cpp
  serialise/MapSerialiseV0.4.h
  serialise/MapSerialiseV0.cpp
  serialise/MapSerialiseV0.h
  Aabb.h
  CopyUtil.cpp
  CopyUtil.h
  CalculateSegmentKeys.cpp
  CalculateSegmentKeys.h
  ClearingPattern.cpp
  ClearingPattern.h
  CompareMaps.cpp
  CompareMaps.h
  CovarianceVoxel.cpp
  CovarianceVoxel.h
  CovarianceVoxelCompute.h
  DataType.cpp
  DataType.h
  DebugDraw.cpp
  DebugDraw.h
  Density.cpp
  Density.h
  DefaultLayer.cpp
  DefaultLayer.h
  Key.cpp
  Key.h
  KeyStream.h
  KeyHash.h
  KeyList.cpp
  KeyList.h
  KeyRange.cpp
  KeyRange.h
  LineKeysQuery.cpp
  LineKeysQuery.h
  LineQuery.cpp
  LineQuery.h
  LineWalk.h
  LineWalkCompute.h
  MapChunk.cpp
  MapChunk.h
  MapCoord.h
  MapFlag.cpp
  MapFlag.h
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
  MapRegionCache.cpp
  MapRegionCache.h
  MapSerialise.cpp
  MapSerialise.h
  Mutex.cpp
  Mutex.h
  NdtMap.cpp
  NdtMap.h
  NdtMode.cpp
  NdtMode.h
  NearestNeighbours.cpp
  NearestNeighbours.h
  OccupancyMap.cpp
  OccupancyMap.h
  OccupancyType.cpp
  OccupancyType.h
  Query.cpp
  Query.h
  QueryFlag.h
  RayFilter.cpp
  RayFilter.h
  RayFlag.h
  RayMapper.cpp
  RayMapper.h
  RayMapperNdt.cpp
  RayMapperNdt.h
  RayMapperOccupancy.cpp
  RayMapperOccupancy.h
  RayMapperSecondarySample.cpp
  RayMapperSecondarySample.h
  RayMapperTrace.cpp
  RayMapperTrace.h
  RayMapperTsdf.cpp
  RayMapperTsdf.h
  RayPattern.cpp
  RayPattern.h
  RayPatternConical.cpp
  RayPatternConical.h
  RaysQuery.cpp
  RaysQuery.h
  Stream.cpp
  Stream.h
  Trace.cpp
  Trace.h
  Voxel.cpp
  Voxel.h
  VoxelBlock.cpp
  VoxelBlock.h
  VoxelBlockCompressionQueue.cpp
  VoxelBlockCompressionQueue.h
  VoxelBuffer.cpp
  VoxelBuffer.h
  VoxelData.h
  VoxelIncident.h
  VoxelIncidentCompute.h
  VoxelLayout.cpp
  VoxelLayout.h
  VoxelMean.h
  VoxelMeanCompute.h
  VoxelOccupancy.h
  VoxelOccupancyCompute.h
  VoxelSecondarySample.h
  VoxelTouchTime.h
  VoxelTouchTimeCompute.h
  VoxelTsdf.cpp
  VoxelTsdf.h
  VoxelTsdfCompute.h
)

if(TES_ENABLE)
  # Only required to define the 3es server symbol.
  list(APPEND SOURCES OccupancyUtil.cpp)
endif(TES_ENABLE)

set(PUBLIC_HEADERS
  Aabb.h
  CalculateSegmentKeys.h
  ClearingPattern.h
  CompareMaps.h
  CopyUtil.h
  CovarianceVoxel.h
  CovarianceVoxelCompute.h
  DataType.h
  Density.h
  DefaultLayer.h
  Key.h
  KeyStream.h
  KeyHash.h
  KeyList.h
  KeyRange.h
  LineKeysQuery.h
  LineQuery.h
  LineWalk.h
  LineWalkCompute.h
  MapChunkFlag.h
  MapChunk.h
  MapCoord.h
  MapFlag.h
  MapInfo.h
  MapLayer.h
  MapLayout.h
  MapLayoutMatch.h
  Mapper.h
  MappingProcess.h
  MapProbability.h
  MapRegionCache.h
  MapRegion.h
  MapSerialise.h
  Mutex.h
  NdtMap.h
  NdtMode.h
  NearestNeighbours.h
  OccupancyMap.h
  OccupancyType.h
  OccupancyUtil.h
  QueryFlag.h
  Query.h
  RayFilter.h
  RayFlag.h
  RayMapper.h
  RayMapperNdt.h
  RayMapperOccupancy.h
  RayMapperSecondarySample.h
  RayMapperTrace.h
  RayPatternConical.h
  RayPattern.h
  RaysQuery.h
  Stream.h
  Trace.h
  Voxel.h
  VoxelBlock.h
  VoxelBlockCompressionQueue.h
  VoxelBuffer.h
  VoxelData.h
  VoxelIncident.h
  VoxelLayout.h
  VoxelMean.h
  VoxelMeanCompute.h
  VoxelOccupancy.h
  VoxelOccupancyCompute.h
  VoxelSecondarySample.h
  VoxelTouchTime.h
  VoxelTouchTimeCompute.h
  VoxelTsdf.h
  VoxelTsdfCompute.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmExport.h"
  )

add_library(ohm ${SOURCES})
clang_tidy_target(ohm)

target_link_libraries(ohm
  PUBLIC
    logutil
    ohmutil
    glm::glm
    $<$<BOOL:${OHM_FEATURE_THREADS}>:TBB::tbb>
  PRIVATE
    # Link Eigen as a private depedency. This will only apply additional include dirs as eigen is header only.
    # Usage is entirely private.
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_FEATURE_EIGEN}>:Eigen3::Eigen>>
)
if(BUILD_SHARED)
  # Link additional dependencies when building shared ohm.
  # Because ohm is shared and these are private, static depenencies, we do not need to propagate
  # the dependencies and limit this with $<BUILD_INTERFACE:>
  target_link_libraries(ohm
    PRIVATE
      # Link 3es if enabled.
      $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
      $<BUILD_INTERFACE:ZLIB::ZLIB>
  )
else(BUILD_SHARED)
  # With ohm static, we link depdencencies in such as way that the will propagate and need to be
  # found when ohm is linked.
  target_link_libraries(ohm
    PRIVATE
      # Link 3es if enabled.
      $<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>
      ZLIB::ZLIB
  )
endif(BUILD_SHARED)

if(TARGET Threads::Threads)
  target_link_libraries(ohm PUBLIC Threads::Threads)
endif(TARGET Threads::Threads)

target_include_directories(ohm
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohm>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_include_directories(ohm
  SYSTEM PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/3rdparty"
    # When OHM_TES_DEBUG is disabled, we don't link the 3es library, which would resolve our include dirs.
    # However, we need to include a dummy 3esservermacros.h file, which is in this project and will be
    # found under ${3ES_INCLUDE_DIRS}.
    $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
)

generate_export_header(ohm
  EXPORT_MACRO_NAME ohm_API
  EXPORT_FILE_NAME ohm/OhmExport.h
  STATIC_DEFINE OHM_STATIC)

install(TARGETS ohm EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohm
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohm)

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\private" REGULAR_EXPRESSION "/private/.*$")
source_group("source\\serialise" REGULAR_EXPRESSION "/serialise/.*$")
