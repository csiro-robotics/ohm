find_package(GLM)
find_package(ZLIB)

set(SOURCES
ohmfilter.cpp
)

add_executable(ohmfilter ${SOURCES})
leak_track_target_enable(ohmfilter CONDITION OHM_LEAK_TRACK)

set_target_properties(ohmfilter PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmfilter PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmfilter SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmfilter PUBLIC ohm ohmutil slamio ${ZLIB_LIBRARIES})

clang_tidy_target(ohmfilter)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmfilter DESTINATION bin)
