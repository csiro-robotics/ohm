find_package(GLM)
find_package(ZLIB)

set(SOURCES
  ohmcmp.cpp
)

add_executable(ohmcmp ${SOURCES})
leak_track_target_enable(ohmcmp CONDITION OHM_LEAK_TRACK)

set_target_properties(ohmcmp PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmcmp PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmcmp SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmcmp PUBLIC ohm ohmutil ${ZLIB_LIBRARIES})
clang_tidy_target(ohmcmp)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmcmp DESTINATION bin)
