find_package(Boost COMPONENTS "thread")
find_package(GLM)
find_package(LIBLAS QUIET)
find_package(LASZIP QUIET)
find_package(ZLIB)

add_subdirectory(ohm2ply)
add_subdirectory(ohmquery)
add_subdirectory(ohmpop)

if(CLOUD_IO_LIB STREQUAL "liblas")
  add_subdirectory(ohmcloudfilter)
endif(CLOUD_IO_LIB STREQUAL "liblas")
