find_package(GLM)
find_package(ZLIB)

set(SOURCES
  ohmsubmap.cpp
)

add_executable(ohmsubmap ${SOURCES})
leak_track_target_enable(ohmsubmap CONDITION OHM_LEAK_TRACK)

set_target_properties(ohmsubmap PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmsubmap PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmsubmap SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmsubmap PUBLIC ohm ohmutil ${ZLIB_LIBRARIES})

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmsubmap DESTINATION bin)
