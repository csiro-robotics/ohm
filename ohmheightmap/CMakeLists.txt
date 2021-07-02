
include(GenerateExportHeader)
include(TextFileResource)

find_package(GLM)
find_package(ZLIB)
if(OHM_THREADS)
  find_tbb()
endif(OHM_THREADS)

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

if(TES_ENABLE)
  # Only required to define the 3es server symbol.
  list(APPEND SOURCES OccupancyUtil.cpp)
endif(TES_ENABLE)

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

target_include_directories(ohmheightmap
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmheightmap>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

if(OHM_THREADS)
  target_include_directories(ohmheightmap SYSTEM PRIVATE ${TBB_INCLUDE_DIRS})
  target_link_libraries(ohmheightmap PUBLIC ${TBB_LIBRARIES})
endif(OHM_THREADS)

target_include_directories(ohmheightmap SYSTEM
  PUBLIC "${GLM_INCLUDE_DIR}"
  PRIVATE "${ZLIB_INCLUDE_DIR}"
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/3rdparty>
)

if(OHM_WITH_EIGEN)
  # Link Eigen as a private link target under the BUILD_INTERFACE. This allows Eigen definitions to be set,
  # imports inlcude directories, but does not chain the dependency downstream, which is fine for static, or header only
  # libraries (Eigen being header only).
  target_link_libraries(ohmheightmap PRIVATE $<BUILD_INTERFACE:Eigen3::Eigen>)
endif(OHM_WITH_EIGEN)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmheightmap PUBLIC 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmheightmap SYSTEM PRIVATE "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

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
