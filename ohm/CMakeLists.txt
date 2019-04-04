
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
  3rdparty/delaunator.hpp
  private/ClearingPatternDetail.h
  private/HeightmapDetail.cpp
  private/HeightmapDetail.h
  private/LineQueryDetail.h
  private/MapLayerDetail.h
  private/MapLayoutDetail.h
  private/NearestNeighboursDetail.h
  private/OccupancyMapDetail.cpp
  private/OccupancyMapDetail.h
  private/QueryDetail.h
  private/SerialiseUtil.h
  private/VoxelAlgorithms.cpp
  private/VoxelAlgorithms.h
  private/VoxelLayoutDetail.h
  serialise/MapSerialiseV0.1.cpp
  serialise/MapSerialiseV0.1.h
  serialise/MapSerialiseV0.2.cpp
  serialise/MapSerialiseV0.2.h
  serialise/MapSerialiseV0.3.cpp
  serialise/MapSerialiseV0.3.h
  serialise/MapSerialiseV0.cpp
  serialise/MapSerialiseV0.h
  Aabb.h
  ClearingPattern.cpp
  ClearingPattern.h
  DataType.cpp
  DataType.h
  DefaultLayer.cpp
  DefaultLayer.h
  Heightmap.cpp
  Heightmap.h
  HeightmapMesh.cpp
  HeightmapMesh.h
  HeightmapVoxel.cpp
  HeightmapVoxel.h
  Key.cpp
  Key.h
  KeyHash.h
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
  RayPattern.cpp
  RayPattern.h
  RayPatternConical.cpp
  RayPatternConical.h
  Stream.cpp
  Stream.h
  SubVoxel.h
  TriangleEdge.h
  TriangleNeighbours.h
  UpAxis.h
  Voxel.cpp
  Voxel.h
  VoxelLayout.cpp
  VoxelLayout.h
  VoxelUtil.cpp
  VoxelUtil.h
)

if(TES_ENABLE)
  # Only required to define the 3es server symbol.
  list(APPEND SOURCES OccupancyUtil.cpp)
endif(TES_ENABLE)

set(PUBLIC_HEADERS
  Aabb.h
  ClearingPattern.h
  DataType.h
  DefaultLayer.h
  Heightmap.h
  HeightmapMesh.h
  HeightmapVoxel.h
  Key.h
  KeyHash.h
  KeyList.h
  LineKeysQuery.h
  LineQuery.h
  MapCache.h
  MapChunk.h
  MapChunkFlag.h
  MapCoord.h
  MapFlag.h
  MapInfo.h
  MapLayer.h
  MapLayout.h
  Mapper.h
  MappingProcess.h
  MapProbability.h
  MapRegion.h
  MapRegionCache.h
  MapSerialise.h
  NearestNeighbours.h
  OccupancyMap.h
  OccupancyType.h
  OccupancyUtil.h
  Query.h
  QueryFlag.h
  RayFilter.h
  RayFlag.h
  RayPattern.h
  RayPatternConical.h
  Stream.h
  SubVoxel.h
  TriangleEdge.h
  TriangleNeighbours.h
  UpAxis.h
  Voxel.h
  VoxelLayout.h
  VoxelUtil.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohm/OhmExport.h"
  )

add_library(ohm ${SOURCES})

target_link_libraries(ohm ${ZLIB_LIBRARIES} ohmutil ${GPUTIL_LIBRARY})

target_include_directories(ohm
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohm>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

if(OHM_THREADS)
  target_link_libraries(ohm ${TBB_LIBRARIES})
endif(OHM_THREADS)

target_include_directories(ohm SYSTEM
  PUBLIC "${GLM_INCLUDE_DIR}"
  PRIVATE "${ZLIB_INCLUDE_DIR}"
)

if(OHM_TES_DEBUG)
  target_link_libraries(ohm 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohm PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

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
