find_package(GLM)
find_package(ZLIB)

set(SOURCES
  ohmheightmap.cpp
)

# Can't add as ohmheightmap as the library target is called that. Use ohmheightmaputil then change the output name.
add_executable(ohmheightmaputil ${SOURCES})
leak_track_target_enable(ohmheightmaputil CONDITION OHM_LEAK_TRACK)

set_target_properties(ohmheightmaputil PROPERTIES OUTPUT_NAME ohmheightmap)

set_target_properties(ohmheightmaputil PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmheightmaputil PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmheightmaputil SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmheightmaputil PUBLIC ohmheightmap ohmtools ohmutil ${ZLIB_LIBRARIES})

if(OHM_TES_DEBUG)
  # target_link_libraries(ohmheightmaputil PUBLIC 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmheightmaputil PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)
clang_tidy_target(ohmheightmaputil)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmheightmaputil DESTINATION bin)
