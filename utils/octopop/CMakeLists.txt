
if(NOT TARGET slamio)
  message(STATUS "Skipping octopop. slamio not built")
  return()
endif()

find_package(octomap CONFIG REQUIRED)

configure_file(OctoPopConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OctoPopConfig.h")

set(SOURCES
  octopop.cpp
  OctoPopConfig.in.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmpop/OctoPopConfig.h"
)

add_executable(octopop ${SOURCES})
leak_track_target_enable(octopop CONDITION OHM_LEAK_TRACK)

set_target_properties(octopop PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(octopop PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(octopop
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmpop>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(octopop SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${OCTOMAP_INCLUDE_DIRS}"
)

target_link_libraries(octopop PUBLIC slamio ohmutil ${OCTOMAP_LIBRARIES})
target_compile_definitions(octopop PUBLIC "-DOHMPOP_CPU")
install(TARGETS octopop DESTINATION bin)

clang_tidy_target(octopop)
leak_track_suppress(octopop CONDITION OHM_LEAK_TRACK
  "libpdal_base"
)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
