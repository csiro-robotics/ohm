find_package(GLM)
find_package(ZLIB)

set(SOURCES
ohmprob.cpp
)

add_executable(ohmprob ${SOURCES})
leak_track_target_enable(ohmprob CONDITION OHM_LEAK_TRACK)

set_target_properties(ohmprob PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmprob PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_link_libraries(ohmprob PUBLIC ohm ohmutil)

target_link_libraries(ohmprob
  PRIVATE
    glm::glm
    $<BUILD_INTERFACE:ZLIB::ZLIB>
)

clang_tidy_target(ohmprob)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmprob DESTINATION bin)
