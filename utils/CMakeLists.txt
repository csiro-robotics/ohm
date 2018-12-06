find_package(Boost COMPONENTS "thread")
find_package(GLM)
find_package(LIBLAS QUIET)
find_package(LASZIP QUIET)
find_package(ZLIB)

option(OHM_WITH_HEIGHTMAP_IMAGE "Build the heightmap to image utility?" On)

add_subdirectory(ohm2ply)
add_subdirectory(ohmheightmap)
add_subdirectory(ohminfo)
add_subdirectory(ohmpop)
add_subdirectory(ohmquery)

if(OHM_WITH_HEIGHTMAP_IMAGE)
  add_subdirectory(ohmhm2img)
endif(OHM_WITH_HEIGHTMAP_IMAGE)

if(CLOUD_IO_LIB STREQUAL "liblas")
  add_subdirectory(ohmcloudfilter)
endif(CLOUD_IO_LIB STREQUAL "liblas")
