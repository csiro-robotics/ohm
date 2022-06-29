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

target_link_libraries(ohmheightmaputil
  PUBLIC
    ohmheightmap
    ohmtools
    ohmutil
  PRIVATE
    glm::glm
    $<BUILD_INTERFACE:ZLIB::ZLIB>
)

target_include_directories(ohmheightmaputil
  SYSTEM PRIVATE
    $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
)

clang_tidy_target(ohmheightmaputil)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmheightmaputil DESTINATION bin)
