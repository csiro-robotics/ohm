find_package(GLM REQUIRED)
find_package(PNG REQUIRED)

set(SOURCES
  ohmhm2img.cpp
)

add_executable(ohmhm2img ${SOURCES})
leak_track_target_enable(ohmhm2img CONDITION OHM_LEAK_TRACK)

set_target_properties(ohmhm2img PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmhm2img PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_link_libraries(ohmhm2img PUBLIC ohmheightmap ohmheightmapimage)

target_link_libraries(ohmhm2img
  PRIVATE
    glm::glm
    $<BUILD_INTERFACE:PNG::PNG>
    $<BUILD_INTERFACE:ZLIB::ZLIB>
)

clang_tidy_target(ohmhm2img)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmhm2img DESTINATION bin)
