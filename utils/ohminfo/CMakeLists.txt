find_package(GLM)
find_package(ZLIB)

set(SOURCES
  ohminfo.cpp
)

add_executable(ohminfo ${SOURCES})
leak_track_target_enable(ohminfo CONDITION OHM_LEAK_TRACK)

set_target_properties(ohminfo PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohminfo PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_link_libraries(ohminfo PUBLIC ohm ohmutil)

target_link_libraries(ohminfo
  PRIVATE
    glm::glm
    $<BUILD_INTERFACE:ZLIB::ZLIB>
)

clang_tidy_target(ohminfo)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohminfo DESTINATION bin)
