
find_package(ZLIB)
find_package(TBB)

add_subdirectory(ohm2ply)
add_subdirectory(ohmcmp)
add_subdirectory(ohmfilter)
add_subdirectory(ohmheightmap)
add_subdirectory(ohminfo)
add_subdirectory(ohmpop)
add_subdirectory(ohmprob)
add_subdirectory(ohmquery)
add_subdirectory(ohmsubmap)

if(OHM_FEATURE_HEIGHTMAP_IMAGE)
  add_subdirectory(ohmhm2img)
endif(OHM_FEATURE_HEIGHTMAP_IMAGE)
