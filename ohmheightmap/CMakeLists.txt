
include(GenerateExportHeader)
include(TextFileResource)

find_package(GLM)
find_package(ZLIB)
if(OHM_FEATURE_THREADS)
  find_package(TBB)
endif(OHM_FEATURE_THREADS)

set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(OhmHeightmapConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmap/OhmHeightmapConfig.h")

set(SOURCES
  private/HeightmapDetail.cpp
  private/HeightmapDetail.h
  private/HeightmapOperations.cpp
  private/HeightmapOperations.h
  Heightmap.cpp
  Heightmap.h
  HeightmapMesh.cpp
  HeightmapMesh.h
  HeightmapMode.cpp
  HeightmapMode.h
  HeightmapSerialise.cpp
  HeightmapSerialise.h
  HeightmapUtil.cpp
  HeightmapUtil.h
  HeightmapVoxel.cpp
  HeightmapVoxel.h
  HeightmapVoxelType.h
  PlaneFillLayeredWalker.cpp
  PlaneFillLayeredWalker.h
  PlaneFillWalker.cpp
  PlaneFillWalker.h
  PlaneWalker.cpp
  PlaneWalker.h
  PlaneWalkVisitMode.h
  TriangleEdge.h
  TriangleNeighbours.h
  UpAxis.h
)

set(PUBLIC_HEADERS
  Heightmap.h
  HeightmapMesh.h
  HeightmapMode.h
  HeightmapSerialise.h
  HeightmapUtil.h
  HeightmapVoxel.h
  HeightmapVoxelType.h
  PlaneFillLayeredWalker.h
  PlaneFillWalker.h
  PlaneWalker.h
  PlaneWalkVisitMode.h
  TriangleEdge.h
  TriangleNeighbours.h
  UpAxis.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmap/OhmHeightmapConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmheightmap/OhmHeightmapExport.h"
  )

add_library(ohmheightmap ${SOURCES})
clang_tidy_target(ohmheightmap)

target_link_libraries(ohmheightmap PUBLIC ohm ohmutil)

target_link_libraries(ohmheightmap
  PUBLIC
    glm::glm
  PRIVATE
    # Link static and header only dependencies as a private link target under the BUILD_INTERFACE. This imports inlcude
    # directories and links the libraries, but does not chain the dependency downstream.
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_FEATURE_EIGEN}>:Eigen3::Eigen>>
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_FEATURE_THREADS}>:TBB::tbb>>
    $<BUILD_INTERFACE:ZLIB::ZLIB>
)

target_include_directories(ohmheightmap
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmheightmap>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_include_directories(ohmheightmap SYSTEM
  PRIVATE
    # Link static and header only dependencies as a private link target under the BUILD_INTERFACE. This imports inlcude
    # directories and links the libraries, but does not chain the dependency downstream.
    "${CMAKE_CURRENT_LIST_DIR}/3rdparty"
    $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
)

generate_export_header(ohmheightmap
  EXPORT_MACRO_NAME ohmheightmap_API
  EXPORT_FILE_NAME ohmheightmap/OhmHeightmapExport.h
  STATIC_DEFINE OHMHEIGHTMAP_STATIC)

install(TARGETS ohmheightmap EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/ohmheightmap
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/ohmheightmap)

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\private" REGULAR_EXPRESSION "/private/.*$")
source_group("source\\serialise" REGULAR_EXPRESSION "/serialise/.*$")
