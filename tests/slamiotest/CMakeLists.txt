# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GLM)

# Eigen required to support some tests - NDT in particular
find_package(Eigen3 QUIET)

set(SOURCES
  Loader.cpp
  SlamCloudLoader.cpp
)

add_executable(slamiotest ${SOURCES})
leak_track_target_enable(slamiotest CONDITION OHM_LEAK_TRACK)
# leak_track_suppress(slamiotest CONDITION OHM_LEAK_TRACK
#   ${OHM_LEAK_SUPPRESS_TBB}
# )

set_target_properties(slamiotest PROPERTIES FOLDER tests)
if(MSVC)
  set_target_properties(slamiotest PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(slamiotest
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(slamiotest SYSTEM
  PRIVATE
    "${GTEST_INCLUDE_DIRS}" # Must include downloaded GTest before potentially including system directories.
)

target_link_libraries(slamiotest PUBLIC slamio ohmutil ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

add_test(NAME slamiotest COMMAND slamiotest --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS slamiotest DESTINATION bin)
