find_package(GLM)
find_package(ZLIB)

set(SOURCES
  ohmheightmap.cpp
)

add_executable(ohmheightmap ${SOURCES})
leak_track_target_enable(ohmheightmap CONDITION OHM_LEAK_TRACK)

set_target_properties(ohmheightmap PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmheightmap PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmheightmap SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmheightmap PUBLIC ohm ohmtools ohmutil ${ZLIB_LIBRARIES} ohmheightmaputil)

if(OHM_TES_DEBUG)
  # target_link_libraries(ohmheightmap PUBLIC 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmheightmap PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)
clang_tidy_target(ohmheightmap)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmheightmap DESTINATION bin)
